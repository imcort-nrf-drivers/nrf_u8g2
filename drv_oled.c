#include "drv_oled.h"

#include "transfer_handler.h"

u8g2_t u8g2;

static uint8_t u8x8_byte_arduino_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) 
{
    uint8_t *data;
    
    switch(msg) {
        
        case U8X8_MSG_BYTE_SEND:
            data = (uint8_t *)arg_ptr;
            spi_transfer(data, arg_int, NULL, 0);
            break;
        
        case U8X8_MSG_BYTE_INIT:
            digitalWrite(OLED_PIN_CS, u8x8->display_info->chip_enable_level);	
            break;
        
        case U8X8_MSG_BYTE_SET_DC:
            digitalWrite(OLED_PIN_DC, arg_int);
            break;
        
        case U8X8_MSG_BYTE_START_TRANSFER:
            digitalWrite(OLED_PIN_CS, u8x8->display_info->chip_enable_level);
            break;
        
        case U8X8_MSG_BYTE_END_TRANSFER:
            digitalWrite(OLED_PIN_CS, u8x8->display_info->chip_enable_level);	
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
            delay(1);
            break;							// can be used to setup pins
        case U8X8_MSG_DELAY_MILLI:			// delay arg_int * 1 milli second
            delay(arg_int);
            break;
        case U8X8_MSG_DELAY_I2C:				// arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
            delayMicroseconds(arg_int);
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
	pinMode(OLED_PIN_VDISP, OUTPUT);
	pinMode(OLED_PIN_RESET, OUTPUT);
	pinMode(OLED_PIN_DC, OUTPUT);
	pinMode(OLED_PIN_CS, OUTPUT);

	//u8g2_Setup_ssd1306_128x64_noname_1(&u8g2, U8G2_R0,u8x8_byte_arduino_hw_spi, u8x8_gpio_and_delay_template);
	u8g2_SetupDisplay(&u8g2, u8x8_d_ssd1306_128x64_noname, u8x8_cad_001, u8x8_byte_arduino_hw_spi, u8x8_gpio_and_delay_template);
  
    u8g2_SetupBuffer(&u8g2, buf, 8, u8g2_ll_hvline_vertical_top_lsb, U8G2_R0);
	
	digitalWrite(OLED_PIN_RESET, 0);
	digitalWrite(OLED_PIN_VDISP, 1);
    
    spi_init();
    
	digitalWrite(OLED_PIN_RESET, 1);	
    
	u8g2_InitDisplay(&u8g2);
    u8g2_ClearDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);
    
}

void drv_oled_on(void)
{

    u8g2_SetPowerSave(&u8g2, 0);
			
}

void drv_oled_sleep(void)
{

    u8g2_SetPowerSave(&u8g2, 1);
			
}
