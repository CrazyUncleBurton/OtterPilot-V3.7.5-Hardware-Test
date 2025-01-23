// Pin Mapping for LilyGo T-Display ESP32-S3 Touch (TFT, not AMOLED)
// This is the board-specific config file.  There are more application-specific
// pin definitions in the main.cpp file.
// Working as of 3/24/2024

#pragma once

#define WIFI_SSID "Your-ssid"
#define WIFI_PASSWORD "Your-passworld"

/*ESP32S3*/
#define PIN_LCD_BL 38 // Backlight

#define PIN_LCD_D0 39
#define PIN_LCD_D1 40
#define PIN_LCD_D2 41
#define PIN_LCD_D3 42
#define PIN_LCD_D4 45
#define PIN_LCD_D5 46
#define PIN_LCD_D6 47
#define PIN_LCD_D7 48

#define PIN_POWER_ON 15 //Power Supply

#define PIN_LCD_RES 5
#define PIN_LCD_CS 6
#define PIN_LCD_DC 7
#define PIN_LCD_WR 8
#define PIN_LCD_RD 9

#define PIN_BUTTON_1 0 
#define PIN_BUTTON_2 14
#define PIN_BAT_VOLT 4

#define PIN_IIC_SCL 17 
#define PIN_IIC_SDA 18 

#define PIN_TOUCH_INT 16
#define PIN_TOUCH_RES 21