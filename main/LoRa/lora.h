/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/



#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "lmic.h"
//#include "gatts/gatts.h"
//#include "I2CMaster.h"
//#include "SSD1306.h"
//#include "HTU21D.h"
//#include "CayenneLPP.h"



void lora_main(void);
bool lora_send(uint8_t* data);







