
//**********FOR OLED************************
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"

#include "sdkconfig.h" // generated by "make menuconfig"

#include "ssd1366.h"

extern "C" void i2c_master_init(void);
extern "C" void ssd1306_init(void);
