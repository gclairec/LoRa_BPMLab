#include "SPILMIC.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "esp_log.h"

#define ESP_INTR_FLAG_DEFAULT 0

static const char* TAG = "SPILMIC";

xQueueHandle SPILMIC::m_evtqueue = NULL;

void  IRAM_ATTR SPILMIC::dio_isr_handler(void *arg)
{
}

void SPILMIC::init()
{
    // Call base initialization
    SPIDevice::init();
    // Initialize gpios
    //

    // RST Pin
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL<< m_rst;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Setup all DIO pins to use interrupts
    // DIOs are activated on the rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = ((1ULL<< m_dio0) | (1ULL<< m_dio1) | (1ULL<< m_dio2));
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Create a queue to handle the event from ISR
    m_evtqueue = xQueueCreate(10, sizeof(uint32_t));

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // hook isr handler to the specific GPIO pins
    gpio_isr_handler_add(m_dio0, &SPILMIC::dio_isr_handler, (void *) 0);
    gpio_isr_handler_add(m_dio1, &SPILMIC::dio_isr_handler, (void *) 1);
    gpio_isr_handler_add(m_dio2, &SPILMIC::dio_isr_handler, (void *) 2);
      
}
