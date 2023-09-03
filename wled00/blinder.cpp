#include <esp_dmx.h>
#include <dmx_types.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>
#include <esp_intr_alloc.h>

TaskHandle_t dmxSendTaskHandle;

void IRAM_ATTR dmxSendTask(void *instance)
{
  const uint8_t TX_PIN = 32; // the pin we are using to TX with
  const uint8_t RX_PIN = 33; // the pin we are using to RX with
  const uint8_t EN_PIN = 19; // the pin we are using to enable TX on the DMX transceiver
  uint8_t data[DMX_PACKET_SIZE] = {0};

  const dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_driver_install(DMX_NUM_1, &config, DMX_INTR_FLAGS_DEFAULT);
  dmx_set_pin(DMX_NUM_1, TX_PIN, RX_PIN, EN_PIN);

  uint8_t d = 0;
  uint32_t brightness = 0;
  while (true)
  {
    if (xTaskNotifyWaitIndexed(0, 0, 0, &brightness, pdMS_TO_TICKS(1000)) == pdPASS)
    {
      dmx_write_slot(DMX_NUM_1, 1, uint8_t(brightness));
    }
    else
    {
      // timeout, disable blinder
      dmx_write_slot(DMX_NUM_1, 1, 0);
    }
    // FIXME there is probably no need to send a full frame
    dmx_send(DMX_NUM_1, DMX_PACKET_SIZE);
  }
}

void initBlinder()
{
  dmxSendTaskHandle = NULL;
  xTaskCreatePinnedToCore(dmxSendTask, "DMX_SEND_TASK", 10240, nullptr, 2, &dmxSendTaskHandle, 0);
}

void setBlinderBrightness(uint8_t bri)
{
  xTaskNotifyIndexed(dmxSendTaskHandle, 0, bri, eSetValueWithOverwrite);
}
