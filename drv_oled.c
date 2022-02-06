#include "drv_oled.h"

#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "app_timer.h"
#include "nrf_log.h"
//#include "disp_menu.h"

#define SPI_INSTANCE  1 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

u8g2_t u8g2;
APP_TIMER_DEF(oled_auto_off_timer);

bool oled_power_en = false;
uint32_t auto_off_time = 40000;

static void oled_auto_off_timer_handler(void * p_context)
{
		drv_oled_off();
	
}

static nrf_drv_spi_config_t spi_config = {                                                            \
    .sck_pin      = OLED_PIN_SCL,                \
    .mosi_pin     = OLED_PIN_SDA,                \
    .miso_pin     = NRF_DRV_SPI_PIN_NOT_USED,                \
    .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,                \
    .irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,         \
    .orc          = 0xFF,                                    \
    .frequency    = NRF_DRV_SPI_FREQ_8M,                     \
    .mode         = NRF_DRV_SPI_MODE_0,                      \
    .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         \
};

static void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
    spi_xfer_done = true;
}

static uint8_t u8x8_byte_arduino_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
	uint8_t *data;
  switch(msg) {
    case U8X8_MSG_BYTE_SEND:
			spi_xfer_done = false;
			nrf_drv_spi_transfer(&spi, data, arg_int, NULL, 0);
      break;
    case U8X8_MSG_BYTE_INIT:
			nrf_gpio_pin_write(OLED_PIN_CS, u8x8->display_info->chip_enable_level);	
			
      break;
    case U8X8_MSG_BYTE_SET_DC:
			nrf_gpio_pin_write(OLED_PIN_DC, arg_int);
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
      
			nrf_gpio_pin_write(OLED_PIN_CS, u8x8->display_info->chip_enable_level);
      break;
    case U8X8_MSG_BYTE_END_TRANSFER:
			nrf_gpio_pin_write(OLED_PIN_CS, u8x8->display_info->chip_enable_level);	
      break;
    default:
      return 0;
  }  
	
  return 1;
	
}

static uint8_t u8x8_gpio_and_delay_template(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:	// called once during init phase of u8g2/u8x8
			nrf_delay_ms(1);
      break;							// can be used to setup pins
    case U8X8_MSG_DELAY_MILLI:			// delay arg_int * 1 milli second
			nrf_delay_ms(arg_int);
      break;
    case U8X8_MSG_DELAY_I2C:				// arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
			nrf_delay_us(arg_int);
      break;							// arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
    default:
      u8x8_SetGPIOResult(u8x8, 1);			// default return value
      break;
  }
  return 1;
}

static uint8_t buf[128*64/8];

void drv_oled_begin(void)
{
	nrf_gpio_cfg_output(OLED_PIN_VDISP);
	nrf_gpio_cfg_output(OLED_PIN_RESET);
	nrf_gpio_cfg_output(OLED_PIN_DC);
	nrf_gpio_cfg_output(OLED_PIN_CS);
	
	//u8g2_Setup_ssd1306_128x64_noname_1(&u8g2, U8G2_R0,u8x8_byte_arduino_hw_spi, u8x8_gpio_and_delay_template);
	u8g2_SetupDisplay(&u8g2, u8x8_d_ssd1306_128x64_noname, u8x8_cad_001, u8x8_byte_arduino_hw_spi, u8x8_gpio_and_delay_template);
  
  u8g2_SetupBuffer(&u8g2, buf, 8, u8g2_ll_hvline_vertical_top_lsb, U8G2_R0);
	
	APP_ERROR_CHECK(app_timer_create(&oled_auto_off_timer, APP_TIMER_MODE_SINGLE_SHOT, oled_auto_off_timer_handler));
	
	nrf_gpio_pin_clear(OLED_PIN_RESET);
	nrf_gpio_pin_set(OLED_PIN_VDISP);
	APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, NULL, NULL));
	nrf_gpio_pin_set(OLED_PIN_RESET);	
	u8g2_InitDisplay(&u8g2);
    u8g2_ClearDisplay(&u8g2);
	
}

void drv_oled_on(void)
{
	if(!oled_power_en)
	{
			
			u8g2_SetPowerSave(&u8g2, 0);
			oled_power_en = true;
		
	}
			app_timer_stop(oled_auto_off_timer);
			//app_timer_start(oled_auto_off_timer, auto_off_time, NULL);
			
			//disp_menu_draw();
			
}

void drv_oled_off(void)
{
	if(oled_power_en)
	{
			u8g2_SetPowerSave(&u8g2, 1);
			oled_power_en = false;
	}
			//app_timer_stop(oled_auto_off_timer);
	
}

//void drv_oled_on(void)
//{
//	if(!oled_power_en)
//	{
//			nrf_gpio_pin_clear(OLED_PIN_RESET);
//			nrf_gpio_pin_set(OLED_PIN_VDISP);
//			APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, NULL, NULL));
//			nrf_gpio_pin_set(OLED_PIN_RESET);	
//			u8g2_InitDisplay(&u8g2);
//			u8g2_SetPowerSave(&u8g2, 0);
//			
//	}
//			app_timer_stop(oled_auto_off_timer);
//			app_timer_start(oled_auto_off_timer, 40000, NULL);
//	
//			oled_power_en = true;
//}

//void drv_oled_off(void)
//{
//	if(oled_power_en)
//	{
//			u8g2_SetPowerSave(&u8g2, 1);
//			nrf_drv_spi_uninit(&spi);
//			nrf_gpio_pin_clear(OLED_PIN_VDISP);
//			nrf_gpio_pin_clear(OLED_PIN_RESET);
//			oled_power_en = false;
//	}
//			app_timer_stop(oled_auto_off_timer);
//	
//}
