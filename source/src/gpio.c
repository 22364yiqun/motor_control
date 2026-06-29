/*lic*/
#include "gpio.h"

#define APP_LED_PORT        (GPIO_PORT_H)
#define APP_LED_PIN         (GPIO_PIN_02)
#define APP_LED_ACTIVE_LOW  (1U)

void PortCfg(void)
{
    /* GPIO initialize */
    stc_gpio_init_t stcGpioInit;
    /* PH2 set to GPIO-Output LED */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(GPIO_PORT_H, GPIO_PIN_02, &stcGpioInit);

    /* PH0 set to XTAL-IN */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_H, GPIO_PIN_00, &stcGpioInit);

    /* PH1 set to XTAL-EXT/XTAL-OUT */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_H, GPIO_PIN_01, &stcGpioInit);

    /* PA0 set to ADC1-IN0 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_A, GPIO_PIN_00, &stcGpioInit);

    /* PA4 set to ADC12-IN4 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_A, GPIO_PIN_04, &stcGpioInit);

    /* PA6 set to ADC123-IN6 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_A, GPIO_PIN_06, &stcGpioInit);

    /* PB5 set to GPIO-Output */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(GPIO_PORT_B, GPIO_PIN_05, &stcGpioInit);
    GPIO_SetPins(GPIO_PORT_B, GPIO_PIN_05); // 拉高，active low

    GPIO_SetFunc(GPIO_PORT_B,GPIO_PIN_13,GPIO_FUNC_2);//TIM4-1-OUL
    
    GPIO_SetFunc(GPIO_PORT_B,GPIO_PIN_14,GPIO_FUNC_2);//TIM4-1-OVL
    
    GPIO_SetFunc(GPIO_PORT_B,GPIO_PIN_15,GPIO_FUNC_2);//TIM4-1-OWL
    
    GPIO_SetFunc(GPIO_PORT_A,GPIO_PIN_08,GPIO_FUNC_2);//TIM4-1-OUH
    
    GPIO_SetFunc(GPIO_PORT_A,GPIO_PIN_09,GPIO_FUNC_2);//TIM4-1-OVH
    
    GPIO_SetFunc(GPIO_PORT_A,GPIO_PIN_10,GPIO_FUNC_2);//TIM4-1-OWH
    
    GPIO_SetFunc(GPIO_PORT_A,GPIO_PIN_11,GPIO_FUNC_32);//USART1-TX
    
    GPIO_SetFunc(GPIO_PORT_A,GPIO_PIN_12,GPIO_FUNC_33);//USART1-RX
    
    GPIO_SetDebugPort(GPIO_PIN_TRST, DISABLE);
    GPIO_SetFunc(GPIO_PORT_B,GPIO_PIN_04,GPIO_FUNC_41);//SPI3-MISO
    
    GPIO_SetFunc(GPIO_PORT_B,GPIO_PIN_06,GPIO_FUNC_51);//CAN1-RX
    
    GPIO_SetFunc(GPIO_PORT_B,GPIO_PIN_07,GPIO_FUNC_50);//CAN1-TX
    
    GPIO_SetFunc(GPIO_PORT_B,GPIO_PIN_08,GPIO_FUNC_40);//SPI3-MOSI
    
    GPIO_SetFunc(GPIO_PORT_B,GPIO_PIN_09,GPIO_FUNC_43);//SPI3-SCK
    
}

void LED_On(void)
{
#if (APP_LED_ACTIVE_LOW != 0U)
    GPIO_ResetPins(APP_LED_PORT, APP_LED_PIN);
#else
    GPIO_SetPins(APP_LED_PORT, APP_LED_PIN);
#endif
}

void LED_Off(void)
{
#if (APP_LED_ACTIVE_LOW != 0U)
    GPIO_SetPins(APP_LED_PORT, APP_LED_PIN);
#else
    GPIO_ResetPins(APP_LED_PORT, APP_LED_PIN);
#endif
}

void LED_Toggle(void)
{
    GPIO_TogglePins(APP_LED_PORT, APP_LED_PIN);
}

/*eof*/
