/*
 * Copyright (c) 2020 David Kalliecharan <david@david.science>
 */

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

/* My Priorities */
//#define PRIORITY_ADC	0
#define PRIORITY_DMA	0

/*
 * Get button configuration from the devicetree sw0 alias.
 *
 * At least a GPIO device and pin number must be provided. The 'flags'
 * cell is optional.
 */
#define LED_PWR_NODE	DT_ALIAS(ledpower)
#define LED_STATUS_NODE	DT_ALIAS(ledstatus)
#define SW0_NODE	DT_ALIAS(sw0)

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

/* SD card related definitions */
#define FS_END_OF_DIR	0

LOG_MODULE_REGISTER(main);

static void sd_init(void);
static int sd_ls(const char *path);

static FATFS fat_fs;
static struct fs_mount_t sdroot = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};

/* NB fatfs able to mount only strings inside _VOLUME_STRS in ffconf.h */
static const char *disk_mount_pt = "/sd:";

/* My garbage */
//#define BUFFER_SIZE 10

static volatile uint16_t dma_adc_sample;

static volatile bool dma_complete = false;
static volatile bool dma_error = false;

static void start_dma_adc(void)
{
	/* Required for SWSTART */
	ADC1->CR2 |= ADC_CR2_ADON;
	/* See Sect. 15.8.1 to restart ADC/DMA transfers */
	ADC1->CR2 &= ~ADC_CR2_DMA;
	ADC1->CR2 |= ADC_CR2_DMA;

	/* Manually start ADC conversion */
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

static void DMA2_Stream0_IRQHandler(void *args)
{
	if (DMA2->LISR & DMA_LISR_TCIF0) {
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
		dma_complete = true;
	}
	
	if (DMA2->LISR & DMA_LISR_TEIF0) {
		DMA2->LIFCR |= DMA_LIFCR_CTEIF0;
		dma_error = true;
	}
	
}

static inline void _gpio_init()
{
	/* Enable GPIOA PA0 as analog */
	GPIOA->MODER |= (GPIO_MODER_MODER3_0 | GPIO_MODER_MODER3_1);
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
}

static inline void _adc_pre_init()
{
	volatile uint32_t tmpreg;

	/* Enable ADC 1 */
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	/* Required to wait the 2 clock cycles before reading */
	tmpreg = (RCC->APB2ENR & RCC_APB2ENR_ADC1EN);
	(void)tmpreg;

	/* ADC prescaler set to 2 */
	ADC123_COMMON->CCR &= (0x00 << ADC_CCR_ADCPRE_Pos);
	/* Disable continous mode */
	ADC1->CR2 &= ~ADC_CR2_CONT;
}

static inline void _adc_post_init()
{
	/* No external trigger */
	//ADC1->CR2 = (0 << ADC_CR2_EXTEN_Pos);
	/* External trigger on Timer2 TRGO */
	//ADC1->CR2 = (0b1011 << ADC_CR2_EXTSEL_Pos) | (0x1 << ADC_CR2_EXTEN_Pos);
	/* One sequence implying a single conversion */
	ADC1->SQR1 = (0x0 << ADC_SQR1_L_Pos);
	/* Put ADC1 Channel 3 on Sequence 1 */
	ADC1->SQR3 = (0x3 << ADC_SQR3_SQ1_Pos);
	/* Sample time 15 cycles on ADC1 PA3 */
	ADC1->SMPR2 = (ADC_SMPR2_SMP3_1 | ADC_SMPR2_SMP3_0);
	/* For DMA enable multplie conversions */
	ADC1->CR1 |= ADC_CR1_SCAN;
	//ADC1->CR2 |= ADC_CR2_DDS;
}

static inline void _dma_init()
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
			     //DMA_SxCR_PINC 	|
			     DMA_SxCR_CIRC 	| /* Circular Buffer */
			     DMA_SxCR_MSIZE_0 	| /* 16-bit */
			     DMA_SxCR_PSIZE_0 	| /* 16-bit */
			     DMA_SxCR_PL_1);	  /* Priority high */

	/* Specify transfer addresses and size for ADC and data */
	DMA2_Stream0->PAR = (uint32_t)&(ADC1->DR);
	DMA2_Stream0->M0AR = (uint32_t)&dma_adc_sample;

	/* Single byte transfer */
	DMA2_Stream0->NDTR = 1;

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
	printk("DMA2_Stream0->FCR  : 0x%08x\n", DMA2_Stream0->FCR);
	printk("DMA2_Stream0->NDTR : 0x%08x\n", DMA2_Stream0->NDTR);
	printk("Starting DMA\n");
	DMA2_Stream0->CR |= DMA_SxCR_EN;
}

/* LED helpers, which use the led0 devicetree alias if it's available. */
//static const struct device *initialize_led(const struct device *led[LED_NUM]);
static void match_led_to_button(const struct device *button,
				const struct device *led);

static struct gpio_callback button_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
	//printk("ADC: 0x%04x\n", poll_adc());
	start_dma_adc();
}

void main(void)
{
	/* Required for STM32F767, breaks DCache breaks DMA on Zephyr */
	 SCB_DisableDCache();

	_gpio_init();
	_adc_pre_init();
	_dma_init();
	_adc_post_init();

	const struct device *button;
	const struct device *led_status;
	const struct device *led_power;
	int ret;



	button = device_get_binding(SW0_GPIO_LABEL);
	if (button == NULL) {
		printk("Error: didn't find %s device\n", SW0_GPIO_LABEL);
		return;
	}

	ret = gpio_pin_configure(button, SW0_GPIO_PIN, SW0_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, SW0_GPIO_LABEL, SW0_GPIO_PIN);
		return;
	}

	ret = gpio_pin_interrupt_configure(button,
					   SW0_GPIO_PIN,
					   GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, SW0_GPIO_LABEL, SW0_GPIO_PIN);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(SW0_GPIO_PIN));
	gpio_add_callback(button, &button_cb_data);
	printk("Set up button at %s pin %d\n", SW0_GPIO_LABEL, SW0_GPIO_PIN);

	//initialize_led();

	led_status = device_get_binding(LED_STATUS_GPIO_LABEL);
	if (led_status == NULL) {
		printk("Didn't find STATUS LED device %s\n", LED_STATUS_GPIO_LABEL);
		return;
	}

	ret = gpio_pin_configure(led_status, LED_STATUS_GPIO_PIN, LED_STATUS_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure STATUS LED device %s pin %d\n",
		       ret, LED_STATUS_GPIO_LABEL, LED_STATUS_GPIO_PIN);
		return;
	}

	printk("Set up STATUS LED at %s pin %d\n", LED_STATUS_GPIO_LABEL, LED_STATUS_GPIO_PIN);
	
	led_power = device_get_binding(LED_PWR_GPIO_LABEL);
	if (led_power == NULL) {
		printk("Didn't find POWER LED device %s\n", LED_PWR_GPIO_LABEL);
		return;
	}

	ret = gpio_pin_configure(led_power, LED_PWR_GPIO_PIN, LED_PWR_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure POWER LED device %s pin %d\n",
		       ret, LED_PWR_GPIO_LABEL, LED_PWR_GPIO_PIN);
		return;
	}

	printk("Set up POWER at %s pin %d\n", LED_PWR_GPIO_LABEL, LED_PWR_GPIO_PIN);

	printk("Press the button\n");
	printk("RCC->CR            : 0x%08x\n", RCC->CR);
	printk("RCC->CFGR          : 0x%08x\n", RCC->CFGR);
	printk("RCC->PLLCFGR       : 0x%08x\n", RCC->PLLCFGR);
	printk("RCC->AHB1ENR       : 0x%08x\n", RCC->AHB1ENR);
	printk("DMA2_Stream0->CR   : 0x%08x\n", DMA2_Stream0->CR);
	printk("DMA2_Stream0->NDTR : 0x%08x\n", DMA2_Stream0->NDTR);
	printk("DMA2_Stream0->PAR  : 0x%08x\n", DMA2_Stream0->PAR);
	printk("DMA2_Stream0->MOAR : 0x%08x\n", DMA2_Stream0->M0AR);
	printk("DMA2_Stream0->FCR  : 0x%08x\n", DMA2_Stream0->FCR);

	printk("PWJ Strain Logger Ready\n");

	sd_init();

	gpio_pin_set(led_power, LED_PWR_GPIO_PIN, 1);
	while (1) {
		match_led_to_button(button, led_status);
		k_msleep(SLEEP_TIME_MS);
		if (ADC1->SR & ADC_SR_OVR) {
			ADC1->SR &= ~ADC_SR_OVR;
			printk("ADC Error!\n");
		}

		if (dma_error) {
			dma_error = false;
			printk("DMA Error!\n");
		}

		if (dma_complete) {
			dma_complete = false;
			printk("DMA transfer complete?\n");
			ADC1->CR2 &= ~ADC_CR2_ADON;
			printk("DMA+ADC : %x\n", dma_adc_sample);
			//printk("test_src\tDMA+ADC\n");
			//arch_dcache_all(K_CACHE_INVD);
			//for (int i = 0; i < MY_MAX_SIZE; i++) {
			//	printk("%x\t\t%x\n",
			//		(uint32_t)test_src[i],
			//		(uint32_t)dma_adc_sample[i]);
			//}
		}
	}
}

static void sd_init(void)
{
	static const char *disk_pdrv = "sd";
	uint64_t memory_size_mb;
	uint32_t block_count;
	uint32_t block_size;

	/* do { ... } while (0); required for multiline macros, i.e., LOG_* */
	do {
		if (disk_access_init(disk_pdrv) != 0) {
			LOG_ERR("SD: Initialization error!");
			break;
		}

		if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT,
			&block_count)) {
			LOG_ERR("SD: Unable to get sector count");
			break;
		}
		LOG_INF("Block count %u", block_count);

		if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
			LOG_ERR("SD: Unable to get sector size");
			break;
		}
		printk("Sector size %u\n", block_size);

		memory_size_mb = (uint64_t)block_count * block_size;
		/*
		 * >> 10 is the same as divide by 1000 but faster
		 * >> 20 is the same as divide by 1000*1000 or 1000000
		 */
		printk("Memory Size is %u MB\n", (uint32_t)(memory_size_mb >> 20));
	} while (0);

	sdroot.mnt_point = disk_mount_pt;

	int status = fs_mount(&sdroot);

	if (status == FR_OK) {
		printk("microSD disk mounted.\n");
		/* List contents of microSD for debug output */
		sd_ls(disk_mount_pt);
	} else {
		printk("Error mounting microSD disk.\n");
	}
}

static int sd_ls(const char *path)
{
	int status;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;

	fs_dir_t_init(&dirp);

	status = fs_opendir(&dirp, path);
	if (status) {
		printk("Error opening dir %s [E:%d]\n", path, status);
		return status;
	}

	printk("\nDirectory %s contents:\n", path);
	for (;;) {
		status = fs_readdir(&dirp, &entry);
		if (status || entry.name[0] == FS_END_OF_DIR) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			printk("[DIR ] %s\n", entry.name);
		} else {
			printk("[FILE] %s (size = %zu)\n", entry.name, entry.size);
		}
	}

	fs_closedir(&dirp);

	return status;
}

/*
 * The led0 devicetree alias is optional. If present, we'll use it
 * to turn on the LED whenever the button is pressed.
 */
#ifdef LED_STATUS_GPIO_LABEL
/*static const struct device *initialize_led(const struct device *led[LED_NUM])
{
	int ret;

	led[STATUS] = device_get_binding(LED_STATUS_GPIO_LABEL);
	if (led == NULL) {
		printk("Didn't find STATUS LED device %s\n", LED_STATUS_GPIO_LABEL);
		return NULL;
	}

	ret = gpio_pin_configure(led[STATUS], LED_STATUS_GPIO_PIN, LED_STATUS_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure STATUS LED device %s pin %d\n",
		       ret, LED_STATUS_GPIO_LABEL, LED_STATUS_GPIO_PIN);
		return NULL;
	}

	printk("Set up STATUS LED at %s pin %d\n", LED_STATUS_GPIO_LABEL, LED_STATUS_GPIO_PIN);
	
	led[POWER] = device_get_binding(LED_PWR_GPIO_LABEL);
	if (led == NULL) {
		printk("Didn't find POWER LED device %s\n", LED_PWR_GPIO_LABEL);
		return NULL;
	}

	ret = gpio_pin_configure(led[POWER], LED_PWR_GPIO_PIN, LED_PWR_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure POWER LED device %s pin %d\n",
		       ret, LED_PWR_GPIO_LABEL, LED_PWR_GPIO_PIN);
		return NULL;
	}

	printk("Set up POWER at %s pin %d\n", LED_PWR_GPIO_LABEL, LED_PWR_GPIO_PIN);
	return NULL;
}*/

static void match_led_to_button(const struct device *button,
				const struct device *led)
{
	bool val;

	val = gpio_pin_get(button, SW0_GPIO_PIN);
	gpio_pin_set(led, LED_STATUS_GPIO_PIN, val);
}
#else  /* !defined(LED_STATUS_GPIO_LABEL) */
/*static const struct device *initialize_led(const struct device *led[LED_NUM])
{
	printk("No LED device was defined\n");
	return NULL;
}*/

static void match_led_to_button(const struct device *button,
				const struct device *led)
{
	return;
}
#endif	/* LED_STATUS_GPIO_LABEL */


