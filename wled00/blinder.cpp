#include <esp_dmx.h>
#include <dmx_types.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>
#include <esp_intr_alloc.h>

#include "wled.h"

static TaskHandle_t dmxSendTaskHandle;

static void IRAM_ATTR dmxSendTask(void *instance)
{
  uint8_t data[DMX_PACKET_SIZE] = {0};
  const uint8_t TX_PIN = 32; // the pin we are using to TX with
  const uint8_t RX_PIN = 33; // the pin we are using to RX with
  const uint8_t EN_PIN = 19; // the pin we are using to enable TX on the DMX transceiver
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  config.dmx_start_address = 42; // just to disable nvs
  config.pd_size = 50;           // disable rdm
  if (!dmx_driver_install(DMX_NUM_1, &config, DMX_INTR_FLAGS_DEFAULT))
  {
    USER_PRINTF("DRIVER INSTALL FAIL!!!!\n");
    return;
  }
  dmx_set_pin(DMX_NUM_1, TX_PIN, RX_PIN, EN_PIN);
  uint8_t d = 0;
  uint32_t brightness = 0;
  while (true)
  {
    if (xTaskNotifyWaitIndexed(0, 0, 0, &brightness, pdMS_TO_TICKS(1000)) == pdPASS)
    {
      for (int i = 1; i < DMX_PACKET_SIZE; i++)
      {
        data[i] = brightness;
      }
      dmx_write(DMX_NUM_1, data, DMX_PACKET_SIZE);
    }
    else
    {
      USER_PRINTF("dmx send timeout\n");
      // timeout, disable blinder
      for (int i = 1; i < DMX_PACKET_SIZE; i++)
      {
        data[i] = 0;
      }
      dmx_write(DMX_NUM_1, data, DMX_PACKET_SIZE);
    }
    // FIXME there is probably no need to send a full frame
    dmx_send(DMX_NUM_1, DMX_PACKET_SIZE);
  }
}

void initBlinder()
{
  dmxSendTaskHandle = nullptr;
  const int ret = xTaskCreatePinnedToCore(dmxSendTask, "DMX_SEND_TASK", 10240, nullptr, 2, &dmxSendTaskHandle, 0);
  if (ret != pdPASS)
  {
    ESP_LOGE("dmx_send", "Cannot create dmx send task! error: %d", ret);
  }
  else
  {
    USER_PRINTF("Blinder Send Task Initialized\n");
  }
  if (dmxSendTaskHandle == nullptr)
  {
    ESP_LOGE("dmx_send", "Cannot create dmx send task! ptr is null");
  }
}

void setBlinderBrightness(uint8_t bri)
{
  if (dmxSendTaskHandle)
  {
    xTaskNotifyIndexed(dmxSendTaskHandle, 0, bri, eSetValueWithOverwrite);
  }
}
