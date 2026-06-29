#include "spi.h"

void SpiCfg(void)
{
    stc_spi_init_t spi_init;
    stc_spi_delay_t spi_delay;

    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_SPI3, ENABLE);

    SPI_StructInit(&spi_init);
    spi_init.u32WireMode = SPI_4_WIRE;
    spi_init.u32TransMode = SPI_FULL_DUPLEX;
    spi_init.u32MasterSlave = SPI_MASTER;
    spi_init.u32Parity = SPI_PARITY_INVD;
    spi_init.u32SpiMode = SPI_MD_3;
    spi_init.u32BaudRatePrescaler = SPI_BR_CLK_DIV16;
    spi_init.u32DataBits = SPI_DATA_SIZE_16BIT;
    spi_init.u32FirstBit = SPI_FIRST_MSB;
    spi_init.u32SuspendMode = SPI_COM_SUSP_FUNC_OFF;
    spi_init.u32FrameLevel = SPI_1_FRAME;
    (void)SPI_Init(CM_SPI3, &spi_init);
    SPI_SetCommMode(CM_SPI3, SPI_COMM_MD_NORMAL);

    SPI_DelayStructInit(&spi_delay);
    spi_delay.u32IntervalDelay = SPI_INTERVAL_TIME_1SCK;
    spi_delay.u32ReleaseDelay = SPI_RELEASE_TIME_1SCK;
    spi_delay.u32SetupDelay = SPI_SETUP_TIME_1SCK;
    (void)SPI_DelayTimeConfig(CM_SPI3, &spi_delay);

    SPI_SetLoopbackMode(CM_SPI3, SPI_LOOPBACK_INVD);
    SPI_ParityCheckCmd(CM_SPI3, DISABLE);
    SPI_SSPinSelect(CM_SPI3, SPI_PIN_SS0);
    SPI_SetSSValidLevel(CM_SPI3, SPI_PIN_SS0, SPI_SS_VALID_LVL_LOW);
    SPI_Cmd(CM_SPI3, ENABLE);
}
