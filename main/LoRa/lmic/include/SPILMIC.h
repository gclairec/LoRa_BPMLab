#ifndef _SPILMIC_H_
#define _SPILMIC_H_

#include "lmic.h"
#include "SPIDevice.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define LMIC_SPI_MODE   1
#define LMIC_SPI_SPEED 10000000

class SPILMIC : public SPIDevice
{
    public:
        SPILMIC(SPIMaster & spi, gpio_num_t cs, gpio_num_t rst, gpio_num_t dio0, gpio_num_t dio1, gpio_num_t dio2)
            :SPIDevice(spi, cs, LMIC_SPI_MODE, LMIC_SPI_SPEED, 0, 8), // mode = 1, 10Mhz, 0 (cmd bits), 8 (address bits)
            m_rst(rst),
            m_dio0(dio0),
            m_dio1(dio1),
            m_dio2(dio2)
        {}

        void init();

    private: 
        gpio_num_t m_rst;
        gpio_num_t m_dio0;
        gpio_num_t m_dio1;
        gpio_num_t m_dio2;

        // Event handler for ISR
        static xQueueHandle m_evtqueue;
        // The ISR handler for DIO's
        static void  IRAM_ATTR dio_isr_handler(void *);
};

#endif
