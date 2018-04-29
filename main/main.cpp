#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "../ecies/encrypt.h"
//#include "display/oled.h"
extern "C" {
#include "gatts/gatts.h"
#include "../lora/lora.h"
}




extern "C" void app_main(void)
{

	/* Do all used peripherals here */

	/* Init HW for the OLED */
	//i2c_master_init();
	//ssd1306_init();

	/* Init the LoRA app function */
	lora_main();

	/* Call the bluetooth app function */
	gatts_main();
}

