/*-
 * Copyright (c) 2022, David Kalliecharan <dave@dal.ca>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 */

/* SPDX-License-Identifier: BSD-2-Clause */

#include "sd.h"

#include <soc.h>
#include <stm32_ll_adc.h>
#include <stm32_ll_dma.h>
#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>

#include <disk/disk_access.h>
#include <logging/log.h>
#include <fs/fs.h>
#include <ff.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>

#define SLEEP_TIME_MS	1
#define PRIORITY_DMA	0

#define DATA_BUFFER_LEN	2 << 13
/* Sample rate should not be a multiple of 40 kHz */
#define SAMPLE_RATE_HZ	169000

/*
 * Get button configuration from the devicetree sw0 alias.
 *
 * At least a GPIO device and pin number must be provided. The 'flags'
 * cell is optional.
 */
#define LED_PWR_NODE	DT_ALIAS(ledpower)
#define LED_STATUS_NODE	DT_ALIAS(ledstatus)
#define SW0_NODE	DT_ALIAS(sw0)
#define SW1_NODE	DT_ALIAS(sw1)

#if DT_NODE_HAS_STATUS(LED_PWR_NODE, okay) && DT_NODE_HAS_PROP(LED_PWR_NODE, gpios)
#define LED_PWR_GPIO_LABEL	DT_GPIO_LABEL(LED_PWR_NODE, gpios)
#define LED_PWR_GPIO_PIN	DT_GPIO_PIN(LED_PWR_NODE, gpios)
#define LED_PWR_GPIO_FLAGS	(GPIO_OUTPUT | DT_GPIO_FLAGS(LED_PWR_NODE, gpios))
#endif

#if DT_NODE_HAS_STATUS(LED_STATUS_NODE, okay) && DT_NODE_HAS_PROP(LED_STATUS_NODE, gpios)
#define LED_STATUS_GPIO_LABEL	DT_GPIO_LABEL(LED_STATUS_NODE, gpios)
#define LED_STATUS_GPIO_PIN	DT_GPIO_PIN(LED_STATUS_NODE, gpios)
#define LED_STATUS_GPIO_FLAGS	(GPIO_OUTPUT | DT_GPIO_FLAGS(LED_STATUS_NODE, gpios))
#endif

#define LED_NUM 2
enum leds_available {POWER, STATUS};

#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
#define SW0_GPIO_LABEL	DT_GPIO_LABEL(SW0_NODE, gpios)
#define SW0_GPIO_PIN	DT_GPIO_PIN(SW0_NODE, gpios)
#define SW0_GPIO_FLAGS	(GPIO_INPUT | DT_GPIO_FLAGS(SW0_NODE, gpios))
#else
#error "Unsupported board: sw0 devicetree alias is not defined"
#define SW0_GPIO_LABEL	""
#define SW0_GPIO_PIN	0
#define SW0_GPIO_FLAGS	0
#endif

#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
#define SW1_GPIO_LABEL	DT_GPIO_LABEL(SW1_NODE, gpios)
#define SW1_GPIO_PIN	DT_GPIO_PIN(SW1_NODE, gpios)
#define SW1_GPIO_FLAGS	(GPIO_INPUT | DT_GPIO_FLAGS(SW1_NODE, gpios))
#else
#error "Unsupported board: sw1 devicetree alias is not defined"
#define SW1_GPIO_LABEL	""
#define SW1_GPIO_PIN	0
#define SW1_GPIO_FLAGS	0
#endif

enum timeout_flag {
	TIMEOUT_DEFAULT_MS = 1000,
	TIMEOUT_ERROR_ADC  = 500,
	TIMEOUT_ERROR_DMA  = 100,
	TIMEOUT_ERROR_SD   = 50,
};

/* SD card related definitions */
LOG_MODULE_REGISTER(main);

static struct gpio_callback button_cb_data;

static volatile uint16_t data_buffer[2][DATA_BUFFER_LEN];
static volatile bool dma_complete = false;
static volatile bool dma_error = false;
static uint16_t *pfull_buffer = NULL;

static struct k_timer status_timer;
static const struct device *led_status;
enum status_state_enum {
	STATUS_IDLE = 0,
	STATUS_DATA_ACQ,
	STATUS_ERROR_ADC = -1,
	STATUS_ERROR_DMA = -2,
} status_state = STATUS_IDLE;

enum timer_state_enum {
	TIMER_DISABLED = 0,
	TIMER_ENABLED = 1,
	TIMER_SOFT_STOP,
} timer_state = TIMER_DISABLED;

static volatile uint32_t tim_reload;

static void
DMA2_Stream0_IRQHandler(void *args)
{
	if (DMA2->LISR & DMA_LISR_TCIF0) {
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
		dma_complete = true;
		/* Double buffer check */
		if (DMA2_Stream0->CR & DMA_SxCR_CT) {
			pfull_buffer = (uint16_t*)&data_buffer[0][0];
		} else {
			pfull_buffer = (uint16_t*)&data_buffer[1][0];
		}
	}

	if (DMA2->LISR & DMA_LISR_TEIF0) {
		DMA2->LIFCR |= DMA_LIFCR_CTEIF0;
		dma_error = true;
	}

	if (timer_state == TIMER_SOFT_STOP) {
		TIM2->CR1 &= ~TIM_CR1_CEN;
		ADC1->CR2 &= ~ADC_CR2_ADON;
		timer_state = TIMER_DISABLED;
		LOG_DBG("TIM2 | Stopped\n");
		sd_close();
	}
}

static inline
void _gpio_init()
{
	/* Enable GPIOA PA0 as analog */
	GPIOA->MODER |= (GPIO_MODER_MODER3_0 | GPIO_MODER_MODER3_1);
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
}

static inline void
_adc_pre_init(void)
{
	volatile uint32_t tmpreg;

	/* Enable ADC 1 peripheral clock, wait 2 clock cycles before read  */
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	tmpreg = (RCC->APB2ENR & RCC_APB2ENR_ADC1EN);
	(void)tmpreg;

	/* ADC prescaler set to 2 */
	ADC123_COMMON->CCR &= (0x00 << ADC_CCR_ADCPRE_Pos);
	/* Disable continous mode */
	ADC1->CR2 |= ADC_CR2_CONT;
}

static inline void
_adc_post_init(void)
{
	/* External trigger on Timer2 TRGO */
	ADC1->CR2 = (0b1011 << ADC_CR2_EXTSEL_Pos) | (0x1 << ADC_CR2_EXTEN_Pos);
	/* One sequence implying a single conversion */
	ADC1->SQR1 = (0x0 << ADC_SQR1_L_Pos);
	/* Put ADC1 Channel 3 on Sequence 1 */
	ADC1->SQR3 = (0x3 << ADC_SQR3_SQ1_Pos);
	/* Sample time 15 cycles on ADC1 PA3 */
	ADC1->SMPR2 = (ADC_SMPR2_SMP3_1 | ADC_SMPR2_SMP3_0);
	/* For DMA enable multplie conversions */
	ADC1->CR1 |= ADC_CR1_SCAN;
	/* Enable DMA transfers */
	ADC1->CR2 |= ADC_CR2_DDS;
	ADC1->CR2 |= ADC_CR2_DMA;
}

static inline void
_timer_init(void)
{
	uint32_t tmpreg;
	uint32_t apb1_ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
	uint32_t tim_clk_freq = SystemCoreClock >> APBPrescTable[apb1_ppre1];
	/* tim_clk_freq @ 36 MHz in STM32F767ZI default state.
	 * for simplicity set timer update every 1 MHz
	 */
	uint32_t tim_prescaler = 1;

	if (apb1_ppre1 != RCC_CFGR_PPRE1_DIV1) {
		tim_clk_freq *= 2;
	}
	LOG_DBG("TIM | PPRE1           : 0x%02x\n", apb1_ppre1);
	LOG_DBG("TIM | CLK             : %i\n", tim_clk_freq);

	/* Enable TIM2 peripheral clock, wait 2 clock cycles before read */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	tmpreg = RCC->APB1ENR & RCC_APB1ENR_TIM2EN;
	(void)tmpreg;

	/* TIM2 is a 32-bit timer, but prescaler is 16-bits wide
	 * This implies timer range is tim_clk_freq/0xffff -> tim_clk_freq
	 */
	tim_reload = tim_clk_freq / (tim_prescaler * SAMPLE_RATE_HZ);
	/* WARN Adjust register by -1 since they count from zero */
	TIM2->PSC = tim_prescaler - 1;
	TIM2->ARR = tim_reload - 1;

	/* Upcounter, and edge aligned */
	TIM2->CR1 &= ~(TIM_CR1_DIR & TIM_CR1_CMS);
	/* Update event for trigger output TRGO */
	TIM2->CR2 |= (0x2 << TIM_CR2_MMS_Pos);
	/* Update generation, re-initialize the counter */
	TIM2->EGR |= TIM_EGR_UG;
}

static inline void
_dma_init(void)
{
	/* Goal select DMA2 to use ADC1 on stream 0 channel 0 */

	volatile uint32_t tmpreg;
	/* Enable peripherial clock for DMA */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	/* Give 2 clock cycles to stabilize */
	tmpreg = (RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN);
	(void)tmpreg;

	/* Select Channel 0 for Stream 0 on DMA2 */
	//DMA2_Stream0->CR = (0x0 << DMA_SxCR_CHSEL_Pos);

	/* Configuration transfer */
	/* Reset assumes periph. to memory, perih. no inc., DMA controls flow */
	const uint32_t cr = (DMA_SxCR_MINC 	| /* Memory inc. */
			     DMA_SxCR_CIRC 	| /* Circular Buffer */
			     DMA_SxCR_DBM 	| /* Double buffer */
			     DMA_SxCR_MSIZE_0 	| /* 16-bit */
			     DMA_SxCR_PSIZE_0 	| /* 16-bit */
			     DMA_SxCR_PL_1);	  /* Priority high */

	/* Specify transfer addresses and size for ADC and data */
	DMA2_Stream0->PAR = (uint32_t)&(ADC1->DR);
	DMA2_Stream0->M0AR = (uint32_t)&data_buffer[0][0];
	DMA2_Stream0->M1AR = (uint32_t)&data_buffer[1][0];

	/* Do not exceed DMA byte transfer limit */
	if (DATA_BUFFER_LEN > 0xffff) {
		DMA2_Stream0->NDTR = 0xffff;
	} else {
		DMA2_Stream0->NDTR = DATA_BUFFER_LEN;
	}

	/* XXX Zephyr style IRQ */
	IRQ_CONNECT(DMA2_Stream0_IRQn, PRIORITY_DMA,
		    DMA2_Stream0_IRQHandler, 0, 0);
	irq_enable(DMA2_Stream0_IRQn);

	/* Enable transfer error+complete interrupts */
	DMA2_Stream0->CR = (cr | DMA_SxCR_TCIE | DMA_SxCR_TEIE);

	/* Enable transfers + Clear interrupt flags (datasheet suggestion) */
	DMA2->LIFCR |= (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CTEIF0 |
			DMA_LIFCR_CTCIF1 | DMA_LIFCR_CTEIF1 |
			DMA_LIFCR_CTCIF2 | DMA_LIFCR_CTEIF2 |
			DMA_LIFCR_CTCIF3 | DMA_LIFCR_CTEIF3);
	DMA2->HIFCR |= (DMA_HIFCR_CTCIF4 | DMA_HIFCR_CTEIF4 |
			DMA_HIFCR_CTCIF5 | DMA_HIFCR_CTEIF5 |
			DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTEIF6 |
			DMA_HIFCR_CTCIF7 | DMA_HIFCR_CTEIF7);
	//DMA2_Stream0->FCR |= DMA_SxFCR_DMDIS;
	printk("Starting DMA\n");
	DMA2_Stream0->CR |= DMA_SxCR_EN;
}

void
button_pressed(const struct device *dev, struct gpio_callback *cb,
	uint32_t pins)
{
	uint16_t status_timeout = TIMEOUT_DEFAULT_MS;
	LOG_DBG("[%" PRIu32 "] Button pressed\n", k_cycle_get_32());

	/* Toggle TIM2 state */
	if (timer_state == TIMER_DISABLED) {
		status_state = STATUS_DATA_ACQ;
		/* Turn on LED solid to indicate data collection */
		gpio_pin_set(led_status, LED_STATUS_GPIO_PIN, 1);
		k_timer_stop(&status_timer);
		sd_open("pwj%05i.bin", FS_O_CREATE | FS_O_WRITE);
		TIM2->CR1 |= TIM_CR1_CEN;
		/* Required for SWSTART */
		ADC1->CR2 |= ADC_CR2_ADON;
		timer_state = TIMER_ENABLED;
		LOG_DBG("TIM2 | Starting\n");
	} else {
		status_state = STATUS_IDLE;
		k_timer_start(&status_timer, K_MSEC(status_timeout), K_MSEC(status_timeout));
		timer_state = TIMER_SOFT_STOP;
		LOG_DBG("TIM2 | Stopping\n");
	}
}

void
status_led_handler(struct k_timer *timer_id)
{
	bool led_state;
	led_state = gpio_pin_get(led_status, LED_STATUS_GPIO_PIN);
	gpio_pin_set(led_status, LED_STATUS_GPIO_PIN, !led_state);
}

void
main(void)
{
	/* Required for STM32F767, breaks DCache breaks DMA on Zephyr */
	 SCB_DisableDCache();

	const struct device *button_stm32;
	const struct device *button_pwj;
	const struct device *led_power;
	size_t buf_byte_size = sizeof(uint16_t)*DATA_BUFFER_LEN;
	int ret;
	int sd_status;
	uint16_t status_timeout = TIMEOUT_DEFAULT_MS;

	_gpio_init();
	_adc_pre_init();
	_timer_init();
	_dma_init();
	_adc_post_init();

	/* Button on STM32 board */
	button_stm32 = device_get_binding(SW0_GPIO_LABEL);
	if (button_stm32 == NULL) {
		printk("Error: didn't find %s device\n", SW0_GPIO_LABEL);
		return;
	}

	ret = gpio_pin_configure(button_stm32, SW0_GPIO_PIN, SW0_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, SW0_GPIO_LABEL, SW0_GPIO_PIN);
		return;
	}

	ret = gpio_pin_interrupt_configure(button_stm32,
					   SW0_GPIO_PIN,
					   GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, SW0_GPIO_LABEL, SW0_GPIO_PIN);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(SW0_GPIO_PIN));
	gpio_add_callback(button_stm32, &button_cb_data);
	printk("Set up button_stm32 at %s pin %d\n", SW0_GPIO_LABEL, SW0_GPIO_PIN);

	/* Button on PWJ signal board */
	button_pwj = device_get_binding(SW1_GPIO_LABEL);
	if (button_pwj == NULL) {
		printk("Error: didn't find %s device\n", SW1_GPIO_LABEL);
		return;
	}

	ret = gpio_pin_configure(button_pwj, SW1_GPIO_PIN, SW1_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, SW1_GPIO_LABEL, SW1_GPIO_PIN);
		return;
	}

	ret = gpio_pin_interrupt_configure(button_pwj,
					   SW1_GPIO_PIN,
					   GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, SW1_GPIO_LABEL, SW1_GPIO_PIN);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(SW1_GPIO_PIN));
	gpio_add_callback(button_pwj, &button_cb_data);
	printk("Set up button_pwj at %s pin %d\n", SW1_GPIO_LABEL, SW1_GPIO_PIN);

	/* LEDs */
	led_status = device_get_binding(LED_STATUS_GPIO_LABEL);
	if (led_status == NULL) {
		printk("Didn't find STATUS LED device %s\n", LED_STATUS_GPIO_LABEL);
		return;
	}

	ret = gpio_pin_configure(led_status, LED_STATUS_GPIO_PIN,
				 LED_STATUS_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure STATUS LED device %s pin %d\n",
		       ret, LED_STATUS_GPIO_LABEL, LED_STATUS_GPIO_PIN);
		return;
	}

	printk("Set up STATUS LED at %s pin %d\n",
	       LED_STATUS_GPIO_LABEL, LED_STATUS_GPIO_PIN);

	led_power = device_get_binding(LED_PWR_GPIO_LABEL);
	if (led_power == NULL) {
		printk("Didn't find POWER LED device %s\n", LED_PWR_GPIO_LABEL);
		return;
	}

	ret = gpio_pin_configure(led_power, LED_PWR_GPIO_PIN, LED_PWR_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure POWER LED d<M-PageUp>evice %s pin %d\n",
		       ret, LED_PWR_GPIO_LABEL, LED_PWR_GPIO_PIN);
		return;
	}

	LOG_DBG("POWER LED @ %s pin %d\n", LED_PWR_GPIO_LABEL, LED_PWR_GPIO_PIN);
	LOG_DBG("System Clock (Hz)    : %i\n", SystemCoreClock);
	LOG_DBG("RCC->CR            : 0x%08x\n", RCC->CR);
	LOG_DBG("RCC->CFGR          : 0x%08x\n", RCC->CFGR);
	LOG_DBG("RCC->PLLCFGR       : 0x%08x\n", RCC->PLLCFGR);
	LOG_DBG("RCC->AHB1ENR       : 0x%08x\n", RCC->AHB1ENR);
	LOG_DBG("DMA2_Stream0->CR   : 0x%08x\n", DMA2_Stream0->CR);
	LOG_DBG("DMA2_Stream0->NDTR : 0x%08x\n", DMA2_Stream0->NDTR);
	LOG_DBG("DMA2_Stream0->PAR  : 0x%08x\n", DMA2_Stream0->PAR);
	LOG_DBG("DMA2_Stream0->MOAR : 0x%08x\n", DMA2_Stream0->M0AR);
	LOG_DBG("DMA2_Stream0->FCR  : 0x%08x\n", DMA2_Stream0->FCR);

	printk("PWJ Strain Logger Ready\n");

	sd_status = sd_init();
	if (sd_status == SD_DISK_MNT_FAIL) {
		status_timeout = TIMEOUT_ERROR_SD;
	}

	gpio_pin_set(led_power, LED_PWR_GPIO_PIN, 1);
	gpio_pin_set(led_status, LED_STATUS_GPIO_PIN, 0);

	/* Setup Status LED */
	k_timer_init(&status_timer, status_led_handler, NULL);
	k_timer_start(&status_timer, K_MSEC(status_timeout), K_MSEC(status_timeout));
	while (1) {
		if (ADC1->SR & ADC_SR_OVR) {
			ADC1->SR &= ~ADC_SR_OVR;
			status_timeout = TIMEOUT_ERROR_ADC;
			k_timer_stop(&status_timer);
			k_timer_start(&status_timer, K_MSEC(status_timeout), K_MSEC(status_timeout));
			status_state = STATUS_ERROR_ADC;
			printk("ADC Error!\n");
		}

		if (dma_error) {
			dma_error = false;
			status_timeout = TIMEOUT_ERROR_DMA;
			k_timer_stop(&status_timer);
			k_timer_start(&status_timer, K_MSEC(status_timeout), K_MSEC(status_timeout));
			status_state = STATUS_ERROR_DMA;
			printk("DMA Error!\n");
		}

		if (dma_complete) {
			dma_complete = false;
			sd_save((uint8_t*)pfull_buffer, buf_byte_size);
		}
	}
}

