
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
#include "gatts/gatts.h"
//#include "I2CMaster.h"
//#include "SSD1306.h"
//#include "HTU21D.h"
//#include "CayenneLPP.h"

u1_t NWKSKEY[16] = { 0xD6, 0x48, 0x78, 0x85, 0x8A, 0x36, 0xE3, 0x75, 0x5B, 0x48, 0xA6, 0xE8, 0xF4, 0x6C, 0x51, 0xBB };
u1_t APPSKEY[16] = { 0xFB, 0x76, 0xE0, 0xF8, 0x46, 0xFB, 0xB2, 0xA7, 0x49, 0xB3, 0x94, 0x4E, 0x85, 0xAE, 0x14, 0xFC };
u4_t DEVADDR = 0x26011110 ;

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static const char *TAG = "MAIN";
// static uint8_t mydata[] = "MUGRADUATE LAGI MI";
static osjob_t sendjob;
static uint8_t* eData;


/* Variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and
 * maintains its value when ESP32 wakes from deep sleep.
 */
RTC_DATA_ATTR static int boot_count = 0;
static const int deep_sleep_sec = 60; // Sleep every five minutes

// Using the TTGO ESP32 Lora or Heltec ESP32 Lora board
// https://www.thethingsnetwork.org/forum/t/big-esp32-sx127x-topic-part-2/11973
const lmic_pinmap_t lmic_pins = {
    .nss = 18,
    .rst = 16,
    .dio = {26, 32, 33},
    // MISO, MOSI, SCK
    .spi = {19, 27, 5},
};

//I2CMaster (I2C_Num, SDA, SCL);
//I2CMaster i2c(I2C_NUM_1, GPIO_NUM_4, GPIO_NUM_15);
//SSD1306   ssd(i2c, GPIO_NUM_16);
//HTU21D    htu(i2c);
//CayenneLPP lpp;


// Time to linger before going to deep sleep
const unsigned LINGER_TIME = 15;
void check_data_Tosend(void);
bool lora_send(uint8_t* data);
void do_deepsleep(osjob_t * arg)
{
    // Turn off oled display
//    ssd.PowerOff();
    // Enter deep sleep
    ESP_LOGI(TAG, "Entering deep sleep for %d seconds", deep_sleep_sec);
    esp_deep_sleep(1000000LL * deep_sleep_sec);
}

void do_send()
{
//    char tmpbuff[50];
//    lpp.reset();
    ESP_LOGD(TAG, "do_send() called! ... Sending data!");
    if (LMIC.opmode & OP_TXRXPEND) {
        ESP_LOGI(TAG, "OP_TXRXPEND, not sending!");
    } else {
      LMIC_setTxData2(1, eData, 51, 0);
//        float temperature;
//        float humidity;
//        ssd.Fill(SSD1306::Black);
        // Prepare upstream data transmission at the next possible time.
//        if (htu.readTemperature(&temperature))
//        {
//            sprintf(tmpbuff, "Trying to send");
//            ssd.GotoXY(0, 15);
//            ssd.Puts(&tmpbuff[0], &Font_7x10, SSD1306::White);
//            lpp.addTemperature(1, temperature);
//            ESP_LOGI(TAG, "HTU21D Temperature : %.2f C", temperature);
//        }
//        if (htu.readHumidity(&humidity))
//        {
//            sprintf(tmpbuff, "Humidity   : %.2f%%", humidity);
//            ssd.GotoXY(0,27);
//            ssd.Puts(&tmpbuff[0], &Font_7x10, SSD1306::White);
//            lpp.addRelativeHumidity(2, humidity);
//            ESP_LOGI(TAG, "HTU21D Humidity : %.2f %%", humidity);
//        }
//        ssd.UpdateScreen();

//        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        ESP_LOGD(TAG, "Packet queued");
    }
}

void do_receive()
{
    if (LMIC.dataLen > 0)
    {
        // TODO: Copy and process the data from LMIC.dataBeg to a buffer
        ESP_LOGD(TAG, "Received %d of data\n", LMIC.dataLen);
    }
}

// Callbacks from lmic, needs C linkage
void onEvent (ev_t ev) {
    ESP_LOGI(TAG, "Event Time: %lld, %d", os_getTime(), ev);
    switch(ev) {
        case EV_TXCOMPLETE:
            ESP_LOGI(TAG, "EV_TXCOMPLETE (includes waiting for RX windows)");
            if (LMIC.txrxFlags & TXRX_ACK)
              ESP_LOGI(TAG, "Received ack");
            if (LMIC.dataLen) {
              ESP_LOGI(TAG, "Received %d bytes of payload\n", LMIC.dataLen);
            }
            // Schedule the send job at some dela
            // do_send();
            LMIC_setTxData2(1, eData, 51, 0);
//            LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
            ESP_LOGD(TAG, "Packet queued");
           // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(LINGER_TIME), FUNC_ADDR(do_deepsleep));
            break;
         case EV_RXCOMPLETE:
            // data received in ping slot
            ESP_LOGI(TAG, "EV_RXCOMPLETE");
            do_receive();
            break;
          default:
            ESP_LOGI(TAG, "Unknown event: %d", ev);
            break;
    }
}

bool lora_send(uint8_t *data)
{
  eData = data;
  ESP_LOGI(TAG, "Sending data thru LoRa!");
                 for(int i=0;i<39;i++){
                     printf("%x", (unsigned int)data[i]);
                 }


  if (LMIC.opmode & OP_TXRXPEND) {
        ESP_LOGI(TAG, "OP_TXRXPEND, not sending!");
        
        return false;
    } else {
     LMIC_setTxData2(1, data, 51, 0);
  //ESP_LOGI(TAG, "LoRa data sending done!");
    return true;
  }
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

void os_runloop(void * arg) 
{
  // Send an Update to TTN
  ESP_LOGI(TAG, "Sending TTN Update!");
  // do_send();
  check_data_Tosend();
  // Run the lmic state machine
  // wait until we receive a TX_COMPLETE (transmission complete) event
  // and go in to deep sleep...
  while(true) {
    os_run();
//    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

void check_data_Tosend(void)
{
  if(data_avail_chk()==DATA_AVAIL_YES)
  {
    ESP_LOGI(TAG, "DATA TO SEND IS AVAILABLE");
    /*
     * If data is available for sending, get the data
     */
    uint8_t* data = getData_to_send();
    // send data over LoRa
    bool res=lora_send(data);
    if(res){
      ESP_LOGI(TAG,"DATA SENT TO LORA SUCCESSFULLY");
      //clear the data_avail flag
      clear_avail_flag();
    }
      else
      {
        ESP_LOGE(TAG, "DATA SENT TO LORA FAILED!!!");
        //resend
        uint8_t res=lora_send(data);
      }

  }
}

void lora_main(void)
{
  ++boot_count;
  printf("\nboot count: %d", boot_count);
  ESP_LOGI(TAG, "Wake(%d) initializing ....", boot_count);

  os_init();
//  i2c.init();
//  ssd.init();
//  htu.init();
  ESP_LOGI(TAG, "Initialize peripherals, doing LMIC reset ...");

  LMIC_reset();
  ESP_LOGI(TAG, "LMIC RESET");
#ifdef PROGMEM
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  LMIC_setupChannel(0, 433175000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 433375000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 433575000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 433775000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 433975000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 434175000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 434375000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 434575000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 434775000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI); // g2-band
#elif defined(CFG_us915)
  LMIC_selectSubBand(1);
#endif

  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF7;
  LMIC_setDrTxpow(DR_SF7,14);
  
  // Disable channel 1 to 8
//  for(int i = 1; i <= 8; i++) LMIC_disableChannel(i);

  xTaskCreate(os_runloop, "os_runloop", 1024 * 2, (void* )0, 10, NULL);
}
