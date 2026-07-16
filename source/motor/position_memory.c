#include "position_memory.h"

#include "config.h"
#include "control.h"

#include "hc32_ll.h"

#include <stddef.h>

#define POSMEM_MAGIC                    (0x504F5331UL)
#define POSMEM_VERSION                  (1UL)
#define POSMEM_EMPTY_WORD               (0xFFFFFFFFUL)
#define POSMEM_DEG_PER_TURN_X100        (36000L)
#define POSMEM_HALF_TURN_X100           (18000L)

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t seq; // 记录序号，从1开始递增
    int32_t motor_total_deg_x100; // 电机侧累计角度，单位0.01度
    int32_t motor_single_deg_x100; // 编码器当前角度
    int32_t output_total_deg_x100;  // 减速器输出轴累计角度。
    uint32_t reserved;
    uint32_t crc;  // 记录的CRC32校验值，计算时不包含crc字段本身。
} posmem_record_t;

static int32_t m_i32MotorTotalDegX100 = 0;
static int32_t m_i32MotorSingleDegX100 = 0;
static int32_t m_i32LastSavedMotorTotalDegX100 = 0;
static uint32_t m_u32NextSeq = 1UL;
static uint32_t m_u32NextRecordIndex = 0UL;
static uint32_t m_u32LastSaveTick = 0UL;
static uint8_t m_u8HasFlashRecord = 0U;
static uint8_t m_u8EncoderReady = 0U;
static int32_t m_i32LastFlashStatus = LL_OK;

static uint32_t PositionMemory_SectorAddr(void)
{
    return EFM_SECTOR_ADDR(APP_POSMEM_FLASH_SECTOR);
}

static uint32_t PositionMemory_RecordCount(void)
{
    return EFM_SECTOR_SIZE / (uint32_t)sizeof(posmem_record_t);
}

static uint32_t PositionMemory_RecordAddr(uint32_t index)
{
    return PositionMemory_SectorAddr() + (index * (uint32_t)sizeof(posmem_record_t));
}

static int32_t PositionMemory_Wrap360X100(int32_t value)
{
    int32_t out = value % POSMEM_DEG_PER_TURN_X100;

    if (out < 0) {
        out += POSMEM_DEG_PER_TURN_X100;
    }

    return out;
}

static int32_t PositionMemory_OutputFromMotor(int32_t motor_total_deg_x100)
{
    return (int32_t)(((int64_t)motor_total_deg_x100 * (int64_t)APP_GEAR_RATIO_DEN) /
                     (int64_t)APP_GEAR_RATIO_NUM);
}

static uint32_t PositionMemory_Crc(const posmem_record_t *record)
{
    const uint32_t *words = (const uint32_t *)record;
    uint32_t crc = 0xA5A55A5AUL;
    uint32_t i;

    for (i = 0UL; i < ((uint32_t)sizeof(posmem_record_t) / 4UL) - 1UL; i++) {
        crc ^= words[i] + 0x9E3779B9UL + (crc << 6U) + (crc >> 2U);
    }

    return crc;
}

static uint8_t PositionMemory_IsRecordEmpty(const posmem_record_t *record)
{
    const uint32_t *words = (const uint32_t *)record;
    uint32_t i;

    for (i = 0UL; i < ((uint32_t)sizeof(posmem_record_t) / 4UL); i++) {
        if (words[i] != POSMEM_EMPTY_WORD) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t PositionMemory_IsRecordValid(const posmem_record_t *record)
{
    if ((record->magic != POSMEM_MAGIC) ||
        (record->version != POSMEM_VERSION)) {
        return 0U;
    }

    return (record->crc == PositionMemory_Crc(record)) ? 1U : 0U;
}

static void PositionMemory_WaitReady(void)
{
    while (SET != EFM_GetStatus(EFM_FLAG_RDY)) {
        ;
    }
}

static int32_t PositionMemory_OpenFlash(void)
{
    PositionMemory_WaitReady();
    LL_PERIPH_WE(LL_PERIPH_EFM);
    EFM_ClearStatus(EFM_FLAG_ERR);
    EFM_FWMC_Cmd(ENABLE);
    EFM_SingleSectorOperateCmd(APP_POSMEM_FLASH_SECTOR, ENABLE);

    return LL_OK;
}

static void PositionMemory_CloseFlash(void)
{
    EFM_SingleSectorOperateCmd(APP_POSMEM_FLASH_SECTOR, DISABLE);
    EFM_FWMC_Cmd(DISABLE);
    LL_PERIPH_WP(LL_PERIPH_EFM);
}

static int32_t PositionMemory_EraseSector(void)
{
    int32_t ret;

    (void)PositionMemory_OpenFlash();
    ret = EFM_SectorErase(PositionMemory_SectorAddr());
    PositionMemory_CloseFlash();

    return ret;
}

static int32_t PositionMemory_WriteRecord(uint32_t index, const posmem_record_t *record)
{
    const uint32_t *words = (const uint32_t *)record;
    uint32_t addr = PositionMemory_RecordAddr(index);
    uint32_t i;
    int32_t ret = LL_OK;

    (void)PositionMemory_OpenFlash();

    for (i = 0UL; i < ((uint32_t)sizeof(posmem_record_t) / 4UL); i++) {
        ret = EFM_ProgramWordReadBack(addr + (i * 4UL), words[i]);
        if (ret != LL_OK) {
            break;
        }
    }

    PositionMemory_CloseFlash();
    return ret;
}

void PositionMemory_Init(void)
{
    const posmem_record_t *best = NULL;
    uint32_t best_seq = 0UL;
    uint32_t i;

    m_u8HasFlashRecord = 0U; // 
    m_u8EncoderReady = 0U; //
    m_u32NextRecordIndex = 0UL;
    m_u32NextSeq = 1UL;
    m_i32LastFlashStatus = LL_OK;

#if (APP_POSMEM_ENABLE != DDL_OFF)
    for (i = 0UL; i < PositionMemory_RecordCount(); i++) {
        const posmem_record_t *record = (const posmem_record_t *)PositionMemory_RecordAddr(i);

        if (PositionMemory_IsRecordEmpty(record) != 0U) {
            m_u32NextRecordIndex = i;
            break;
        }

        if ((PositionMemory_IsRecordValid(record) != 0U) &&
            ((best == NULL) || (record->seq > best_seq))) {
            best = record;
            best_seq = record->seq;
        }
    }

    if (i >= PositionMemory_RecordCount()) {
        m_u32NextRecordIndex = PositionMemory_RecordCount();
    }

    if (best != NULL) {
        m_i32MotorTotalDegX100 = best->motor_total_deg_x100;
        m_i32MotorSingleDegX100 = best->motor_single_deg_x100;
        m_i32LastSavedMotorTotalDegX100 = best->motor_total_deg_x100;
        m_u32NextSeq = best->seq + 1UL;
        m_u8HasFlashRecord = 1U;
    }
#endif
}

void PositionMemory_UpdateMotorSingleTurn(int32_t motor_single_deg_x100)
{
    int32_t single = PositionMemory_Wrap360X100(motor_single_deg_x100);

    if (m_u8EncoderReady == 0U) {
        if (m_u8HasFlashRecord != 0U) {
            int32_t base = m_i32MotorTotalDegX100 -
                           PositionMemory_Wrap360X100(m_i32MotorTotalDegX100);
            int32_t candidate = base + single;
            int32_t diff = candidate - m_i32MotorTotalDegX100;

            if (diff > POSMEM_HALF_TURN_X100) {
                candidate -= POSMEM_DEG_PER_TURN_X100;
            } else if (diff < -POSMEM_HALF_TURN_X100) {
                candidate += POSMEM_DEG_PER_TURN_X100;
            }
            m_i32MotorTotalDegX100 = candidate;
        } else {
            m_i32MotorTotalDegX100 = single;
            m_i32LastSavedMotorTotalDegX100 = single;
        }

        m_i32MotorSingleDegX100 = single;
        m_u8EncoderReady = 1U;
        return;
    }

    {
        int32_t delta = single - m_i32MotorSingleDegX100;

        if (delta > POSMEM_HALF_TURN_X100) {
            delta -= POSMEM_DEG_PER_TURN_X100;
        } else if (delta < -POSMEM_HALF_TURN_X100) {
            delta += POSMEM_DEG_PER_TURN_X100;
        }

        m_i32MotorTotalDegX100 += delta;
        m_i32MotorSingleDegX100 = single;
    }
}

void PositionMemory_Task(void)
{
#if (APP_POSMEM_ENABLE != DDL_OFF)
    const uint32_t now_tick = Motor_ControlGetTick();
    int32_t delta;
    posmem_record_t record;

    if (m_u8EncoderReady == 0U) {
        return;
    }

    if ((now_tick - m_u32LastSaveTick) < APP_POSMEM_SAVE_INTERVAL_TICKS) {
        return;
    }

    delta = m_i32MotorTotalDegX100 - m_i32LastSavedMotorTotalDegX100;
    if ((delta < APP_POSMEM_SAVE_DELTA_X100) &&
        (delta > -APP_POSMEM_SAVE_DELTA_X100)) {
        return;
    }

    m_u32LastSaveTick = now_tick;

    if (m_u32NextRecordIndex >= PositionMemory_RecordCount()) {
        m_i32LastFlashStatus = PositionMemory_EraseSector();
        if (m_i32LastFlashStatus != LL_OK) {
            return;
        }
        m_u32NextRecordIndex = 0UL;
    }

    record.magic = POSMEM_MAGIC;
    record.version = POSMEM_VERSION;
    record.seq = m_u32NextSeq;
    record.motor_total_deg_x100 = m_i32MotorTotalDegX100;
    record.motor_single_deg_x100 = m_i32MotorSingleDegX100;
    record.output_total_deg_x100 = PositionMemory_OutputFromMotor(m_i32MotorTotalDegX100);
    record.reserved = 0UL;
    record.crc = PositionMemory_Crc(&record);

    m_i32LastFlashStatus = PositionMemory_WriteRecord(m_u32NextRecordIndex, &record);
    if (m_i32LastFlashStatus == LL_OK) {
        m_i32LastSavedMotorTotalDegX100 = m_i32MotorTotalDegX100;
        m_u32NextRecordIndex++;
        m_u32NextSeq++;
        m_u8HasFlashRecord = 1U;
    }
#endif
}

int32_t PositionMemory_GetMotorTotalDegX100(void)
{
    return m_i32MotorTotalDegX100;
}

int32_t PositionMemory_GetOutputTotalDegX100(void)
{
    return PositionMemory_OutputFromMotor(m_i32MotorTotalDegX100);
}

uint8_t PositionMemory_HasValidFlashRecord(void)
{
    return m_u8HasFlashRecord;
}

int32_t PositionMemory_GetLastFlashStatus(void)
{
    return m_i32LastFlashStatus;
}
