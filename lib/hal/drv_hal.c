#include "hal.h"

extern void SetSysClock(void);
#define AFIO_MAPR_SWJ_CFG_NO_JTAG_SW            (0x2 << 24)

void lib_hal_init(void)
{
    gpio_config_t gpio;
    drv_pwm_config_t pwm;

    // Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers
    // Configure the Flash Latency cycles and enable prefetch buffer
    SetSysClock();

    // Turn on clocks for stuff we use
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_ClearFlag();

    // Make all GPIO in by default to save power and reduce noise
    gpio.pin = Pin_All;
    gpio.mode = Mode_AIN;
    gpioInit(GPIOA, &gpio);
    gpioInit(GPIOB, &gpio);
    gpioInit(GPIOC, &gpio);

    // Turn off JTAG port 'cause we're using the GPIO for leds
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_NO_JTAG_SW;

    pwm.airplane = false;
    pwm.useUART = false;
    pwm.usePPM = true;
    pwm.enableInput = true;
    pwm.useServos = false;
    pwm.extraServos = false;
    pwm.motorPwmRate = 498;
    pwm.servoPwmRate = 50;

    pwmInit(&pwm);
}

#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 128
#endif

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
#define FLASH_WRITE_ADDR                (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 1))       // use the last KB for storage
#define EEP_SIZE                        (FLASH_PAGE_SIZE)

static uint8_t eep[EEP_SIZE];

size_t eeprom_write_block (const void *src, uint16_t index, size_t size)
{
    memcpy(eep + index, src, size);
    return size;
}

void eeprom_commit(void)
{
    int tries = 0;
    int i;
    FLASH_Status status;

retry:
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if (FLASH_ErasePage(FLASH_WRITE_ADDR) == FLASH_COMPLETE) {
        for (i = 0; i < EEP_SIZE; i += 4) {
            status = FLASH_ProgramWord(FLASH_WRITE_ADDR + i, *(uint32_t *) ((char *)eep + i));
            if (status != FLASH_COMPLETE) {
                FLASH_Lock();
                tries++;
                if (tries < 3)
                    goto retry;
                else
                    break;
            }
        }
    }
    FLASH_Lock();
}

size_t eeprom_read_block (void *dst, uint16_t index, size_t size)
{
    memcpy(dst, (char *)FLASH_WRITE_ADDR + index, size);
    return size;
}
