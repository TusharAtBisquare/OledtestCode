#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "u8g2.h"
#include "u8g2_esp32_hal.h"

// --- PIN CONFIGURATION FOR YOUR ESP32-C3 MINI ---
// Physical Pin 5 = GPIO 2 (SDA)
// Physical Pin 6 = GPIO 3 (SCK/SCL)
#define I2C_SDA_PIN     2
#define I2C_SCL_PIN     3
#define I2C_ADDRESS     0x78 // 0x3C << 1

static u8g2_t u8g2;

void app_main(void)
{
    // --- 1. Initialize the u8g2 HAL ---
    // Note: The structure here is specific to the mkfrey component
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    
    // Set the specific I2C pins
    u8g2_esp32_hal.bus.i2c.sda = I2C_SDA_PIN;
    u8g2_esp32_hal.bus.i2c.scl = I2C_SCL_PIN;
    
    // Initialize the HAL
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    // --- 2. Setup the Display Driver (SH1106) ---
    // Rotation R0 (Standard), R2 (180 deg flipped)
    u8g2_Setup_sh1106_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb
    );

    // --- 3. Set I2C Address ---
    u8x8_SetI2CAddress(&u8g2.u8x8, I2C_ADDRESS);

    // --- 4. Begin Display ---
    u8g2_InitDisplay(&u8g2); 
    u8g2_SetPowerSave(&u8g2, 0); // Wake up

    // --- 5. Draw Loop ---
    printf("Setup done. Drawing...\n");

    while (1) {
        u8g2_ClearBuffer(&u8g2);

        // Draw "Welcome"
        u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
        u8g2_DrawStr(&u8g2, 10, 25, "Welcome");

        // Draw "to BSQ"
        u8g2_DrawStr(&u8g2, 20, 50, "to BSQ");

        u8g2_SendBuffer(&u8g2);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}