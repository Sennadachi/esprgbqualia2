#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_types.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_rgb.h"
#include <freertos/projdefs.h>
#include "driver/i2c_master.h"
#include "esp_err.h"
//#include "lvgl.h"


#define B4 0
#define PCLK 1
#define DE 2
#define R5 3
#define espSCK 5
#define espMISO 6
#define espMOSI 7
#define SDA 8
#define R3 9
#define R2 10
#define R1 11
#define G5 12
#define G4 13
#define G3 14
#define espCS 15
#define SCL 18
#define G2 21
#define B3 38
#define B2 39
#define B1 40
#define HSYNC 41
#define VSYNC 42
#define B5 45
#define R4 46

/*
=============== RGB Timing ===============
H Active = 720
H Back Porch (Sync Width Not Included) = 44
H Front Porch = 46
H Pulse Width = 2
V Active = 672
V Back Porch (Sync Width Not Included) = 14
V Front Porch = 16
V Pulse Width = 2 */

//PCA9554 pins
#define PCA_ADDR 0x38
#define TFT_SCK 0x0
#define TFT_CS 0x1
#define TFT_RESET 0x2
#define TFT_IRQ 0x3 //input
#define TFT_BACKLIGHT 0x4
#define TFT_MOSI 0x7

i2c_master_bus_handle_t i2cBusHandle;
i2c_master_dev_handle_t pcaHandle;
esp_lcd_panel_handle_t panelHandle;

i2c_master_bus_config_t i2cBusConfig = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = SDA,
    .scl_io_num = SCL,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .flags.enable_internal_pullup = true,
};

i2c_device_config_t i2cDeviceConfig = {
    .dev_addr_length = 7,
    .device_address = 0x38,
    .scl_speed_hz = 100000,
    .scl_wait_us = 0,
};

//send any byte to any reg of the PCA expander through I2C
esp_err_t pcaWriteRegister(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(pcaHandle, data, sizeof(data), 100);
}

//Initialise PCA expander through I2C
void pcaInit(void) {
    uint8_t config = 0x8; //all outputs except IRQ
    pcaWriteRegister(0x3, config); //Push config byte to config register
    pcaWriteRegister(0x2, 0x0); //Disable polarity inversion
    pcaWriteRegister(0x1, 0x0); //Set initial output state as 0x00
    //0x00 register is for reading inputs of pca9554
}

static uint8_t pcaOut = 0x0;

//send pcaOut to the PCA expander through I2C
static inline void pcaWriteOut(void)
{
    uint8_t buf[2] = { 0x01, pcaOut };
    ESP_ERROR_CHECK(i2c_master_transmit(pcaHandle, buf, 2, -1));
}

//set a bit of the PCA expander
static inline void pcaSet(uint8_t mask) {
    pcaOut |= mask;
    pcaWriteOut();
}

//clear a bit of the PCA expander
static inline void pcaClear(uint8_t mask) {
    pcaOut &= ~mask;
    pcaWriteOut();
}

static inline void tftWriteBit(uint8_t bit) {
    if (bit) {
        pcaSet(TFT_MOSI);
    }
    else {
        pcaClear(TFT_MOSI);
    }
    pcaClear(TFT_SCK);
    pcaSet(TFT_SCK);
}

static void tftWriteByte(uint8_t data) {
    for(int i = 7; i >= 0; i--) {
        tftWriteBit((data >> i) & 1); //for each bit in byte "data", and with 1.
    }
}

static void tftWriteCommand(uint8_t cmd, uint8_t data) {
    pcaClear(TFT_CS);
    tftWriteByte(cmd);
    tftWriteByte(data);
    pcaSet(TFT_CS);
}

void displayInit(){
    pcaClear(TFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));
    pcaSet(TFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(120));

    tftWriteCommand(0xFF,0x30); tftWriteCommand(0xFF,0x52); tftWriteCommand(0xFF,0x01);
    tftWriteCommand(0xE3,0x00); tftWriteCommand(0x0A,0x11); tftWriteCommand(0x23,0xA0); //A2
    tftWriteCommand(0x24,0x32); tftWriteCommand(0x25,0x12); tftWriteCommand(0x26,0x2E);
    tftWriteCommand(0x27,0x2E); tftWriteCommand(0x29,0x02); tftWriteCommand(0x2A,0xCF);
    tftWriteCommand(0x32,0x34); tftWriteCommand(0x38,0x9C); tftWriteCommand(0x39,0xA7);
    tftWriteCommand(0x3A,0x27); tftWriteCommand(0x3B,0x94); tftWriteCommand(0x42,0x6D);
    tftWriteCommand(0x43,0x83); tftWriteCommand(0x81,0x00); tftWriteCommand(0x91,0x67);
    tftWriteCommand(0x92,0x67); tftWriteCommand(0xA0,0x52); tftWriteCommand(0xA1,0x50);
    tftWriteCommand(0xA4,0x9C); tftWriteCommand(0xA7,0x02); tftWriteCommand(0xA8,0x02);
    tftWriteCommand(0xA9,0x02); tftWriteCommand(0xAA,0xA8); tftWriteCommand(0xAB,0x28);
    tftWriteCommand(0xAE,0xD2); tftWriteCommand(0xAF,0x02); tftWriteCommand(0xB0,0xD2);
    tftWriteCommand(0xB2,0x26); tftWriteCommand(0xB3,0x26); tftWriteCommand(0xFF,0x30);
    tftWriteCommand(0xFF,0x52); tftWriteCommand(0xFF,0x02); tftWriteCommand(0xB1,0x0A);
    tftWriteCommand(0xD1,0x0E); tftWriteCommand(0xB4,0x2F); tftWriteCommand(0xD4,0x2D);
    tftWriteCommand(0xB2,0x0C); tftWriteCommand(0xD2,0x0C); tftWriteCommand(0xB3,0x30);
    tftWriteCommand(0xD3,0x2A); tftWriteCommand(0xB6,0x1E); tftWriteCommand(0xD6,0x16);
    tftWriteCommand(0xB7,0x3B); tftWriteCommand(0xD7,0x35); tftWriteCommand(0xC1,0x08);
    tftWriteCommand(0xE1,0x08); tftWriteCommand(0xB8,0x0D); tftWriteCommand(0xD8,0x0D);
    tftWriteCommand(0xB9,0x05); tftWriteCommand(0xD9,0x05); tftWriteCommand(0xBD,0x15);
    tftWriteCommand(0xDD,0x15); tftWriteCommand(0xBC,0x13); tftWriteCommand(0xDC,0x13);
    tftWriteCommand(0xBB,0x12); tftWriteCommand(0xDB,0x10); tftWriteCommand(0xBA,0x11);
    tftWriteCommand(0xDA,0x11); tftWriteCommand(0xBE,0x17); tftWriteCommand(0xDE,0x17);
    tftWriteCommand(0xBF,0x0F); tftWriteCommand(0xDF,0x0F); tftWriteCommand(0xC0,0x16);
    tftWriteCommand(0xE0,0x16); tftWriteCommand(0xB5,0x2E); tftWriteCommand(0xD5,0x3F);
    tftWriteCommand(0xB0,0x03); tftWriteCommand(0xD0,0x02); tftWriteCommand(0xFF,0x30);
    tftWriteCommand(0xFF,0x52); tftWriteCommand(0xFF,0x03); tftWriteCommand(0x08,0x09);		
    tftWriteCommand(0x09,0x0A);	tftWriteCommand(0x0A,0x0B); tftWriteCommand(0x0B,0x0C);		
    tftWriteCommand(0x28,0x22);	tftWriteCommand(0x2A,0xE9);	tftWriteCommand(0x2B,0xE9);							  				  
    tftWriteCommand(0x34,0x51); tftWriteCommand(0x35,0x01); tftWriteCommand(0x36,0x26);  
    tftWriteCommand(0x37,0x13); tftWriteCommand(0x40,0x07); tftWriteCommand(0x41,0x08);  
    tftWriteCommand(0x42,0x09); tftWriteCommand(0x43,0x0A); tftWriteCommand(0x44,0x22);
    tftWriteCommand(0x45,0xDB); tftWriteCommand(0x46,0xdC); tftWriteCommand(0x47,0x22);
    tftWriteCommand(0x48,0xDD); tftWriteCommand(0x49,0xDE); tftWriteCommand(0x50,0x0B);  
    tftWriteCommand(0x51,0x0C); tftWriteCommand(0x52,0x0D); tftWriteCommand(0x53,0x0E); 
    tftWriteCommand(0x54,0x22); tftWriteCommand(0x55,0xDF); tftWriteCommand(0x56,0xE0);  
    tftWriteCommand(0x57,0x22); tftWriteCommand(0x58,0xE1); tftWriteCommand(0x59,0xE2); 
    tftWriteCommand(0x80,0x1E); tftWriteCommand(0x81,0x1E); tftWriteCommand(0x82,0x1F);   
    tftWriteCommand(0x83,0x1F); tftWriteCommand(0x84,0x05); tftWriteCommand(0x85,0x0A);   
    tftWriteCommand(0x86,0x0A); tftWriteCommand(0x87,0x0C); tftWriteCommand(0x88,0x0C);   
    tftWriteCommand(0x89,0x0E); tftWriteCommand(0x8A,0x0E); tftWriteCommand(0x8B,0x10);   
    tftWriteCommand(0x8C,0x10); tftWriteCommand(0x8D,0x00); tftWriteCommand(0x8E,0x00);   
    tftWriteCommand(0x8F,0x1F); tftWriteCommand(0x90,0x1F); tftWriteCommand(0x91,0x1E);   
    tftWriteCommand(0x92,0x1E); tftWriteCommand(0x93,0x02); tftWriteCommand(0x94,0x04); 
    tftWriteCommand(0x96,0x1E); tftWriteCommand(0x97,0x1E); tftWriteCommand(0x98,0x1F);   
    tftWriteCommand(0x99,0x1F); tftWriteCommand(0x9A,0x05); tftWriteCommand(0x9B,0x09);   
    tftWriteCommand(0x9C,0x09); tftWriteCommand(0x9D,0x0B); tftWriteCommand(0x9E,0x0B);   
    tftWriteCommand(0x9F,0x0D); tftWriteCommand(0xA0,0x0D); tftWriteCommand(0xA1,0x0F);   
    tftWriteCommand(0xA2,0x0F); tftWriteCommand(0xA3,0x00); tftWriteCommand(0xA4,0x00); 
    tftWriteCommand(0xA5,0x1F); tftWriteCommand(0xA6,0x1F); tftWriteCommand(0xA7,0x1E);   
    tftWriteCommand(0xA8,0x1E); tftWriteCommand(0xA9,0x01); tftWriteCommand(0xAA,0x03);  
    tftWriteCommand(0xFF,0x30); tftWriteCommand(0xFF,0x52); tftWriteCommand(0xFF,0x00);
    tftWriteCommand(0x36,0x0A); tftWriteCommand(0x11,0x00);
    vTaskDelay(pdMS_TO_TICKS(200)); 
    tftWriteCommand(0x29,0x00);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void panelDef(){
esp_lcd_rgb_panel_config_t panelConfig = {
    .data_width = 16,
    .clk_src = LCD_CLK_SRC_DEFAULT,
    .disp_gpio_num = -1, //not used
    .pclk_gpio_num = PCLK,
    .hsync_gpio_num = HSYNC,
    .vsync_gpio_num = VSYNC,
    .de_gpio_num = DE,
    .data_gpio_nums = {R5, R4, R3, R2, R1,
                        G5, G4, G3, G2,
                        B5, B4, B3, B2, B1},
        
    .timings = {
        .pclk_hz = 9 * 1000 * 1000,
        .h_res = 720,
        .v_res = 720,
        .hsync_pulse_width = 2,
        .hsync_back_porch = 44,
        .hsync_front_porch = 46,
        .vsync_pulse_width = 2,
        .vsync_back_porch = 14,
        .vsync_front_porch = 16,
    },
    .flags.fb_in_psram = true,
    .flags.double_fb = true,
};
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panelConfig, &panelHandle));
}

void app_main() {
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2cBusConfig, &i2cBusHandle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cBusHandle, &i2cDeviceConfig, &pcaHandle));
    pcaInit();
    displayInit();
    pcaSet(TFT_BACKLIGHT); //enable backlight
    panelDef(); //initialise panel
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panelHandle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panelHandle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panelHandle, true));
    

    vTaskDelay(1000/portTICK_PERIOD_MS);
}