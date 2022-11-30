#pragma once

#include "wled.h"
#include "esp_dmx.h"
#include "esp_system.h"
#include "esp_rdm_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void startAddressChangedCb(uint16_t newAddr);
void dmxTask(void *instance);

class RdmDmx;
// poor mans singleton, but this is good enough for now
RdmDmx *rdmDmx = nullptr;

class RdmDmx : public Usermod
{

public:
    int dmxAddr;
    int newDmxAddr;
    uint8_t dmxData[13] = {0}; // FIXME hardcoded 13

    void setup()
    {
        rdmDmx = this;
        // FIXME use correct upper bound
        if (dmxAddr < 1 || dmxAddr > 512)
        {
            dmxAddr = 1;
            ESP_LOGE("RdmDmx", "dmx address out of bounds. Resetting to 1");
        }
        newDmxAddr = dmxAddr;

        TaskHandle_t dmxTaskHandle = NULL;
        // pin to core 0 because wled is running on core 1
        xTaskCreatePinnedToCore(dmxTask, "DMX_TASK", 10240, nullptr, 2, &dmxTaskHandle, 0);
        if (!dmxTaskHandle)
        {
            ESP_LOGE("RdmDmx", "Failed to create dmx task");
        }
    }

    void loop()
    {
        // update saved dmx addr
        if (newDmxAddr != dmxAddr)
        {
            dmxAddr = newDmxAddr;
            serializeConfig();
        }

        updateEffect();
    }

    void newDmxData(uint8_t *data)
    {
        // FIXME race conditions etc.
        memcpy(dmxData, data + dmxAddr, 13); // FIXME harcoded 13
    }

    void updateEffect()
    {
        if (bri != dmxData[0])
        {
            bri = dmxData[0];
        }
        if (dmxData[1] < strip.getModeCount())
            effectCurrent = dmxData[1];
        effectSpeed = dmxData[2]; // flickers
        effectIntensity = dmxData[3];
        effectPalette = dmxData[4];
        col[0] = dmxData[5];
        col[1] = dmxData[6];
        col[2] = dmxData[7];
        colSec[0] = dmxData[8];
        colSec[1] = dmxData[9];
        colSec[2] = dmxData[10];

        col[3] = dmxData[11]; // white
        colSec[3] = dmxData[12];
        transitionDelayTemp = 0;              // act fast
        colorUpdated(CALL_MODE_NOTIFICATION); // don't send UDP
    }

    void addToConfig(JsonObject &root)
    {
        JsonObject top = root.createNestedObject("dmx_rdm_mod");
        top["dmx_addr"] = dmxAddr;
    }

    void updateDmxAddr(uint16_t newAddr)
    {
        if (newAddr > 0 && newAddr <= 512)
        {
            // storing needs to happen in next loop
            newDmxAddr = newAddr;
        }
    }

    bool readFromConfig(JsonObject &root)
    {
        JsonObject top = root["dmx_rdm_mod"];
        bool configComplete = !top.isNull();
        configComplete &= getJsonValue(top["dmx_addr"], dmxAddr, 42);
        return configComplete;
    }

    uint16_t getId()
    {
        return USERMOD_ID_RDM_DMX;
    }
};

void dmxTask(void *)
{
    const uint8_t TX_PIN = 17; // the pin we are using to TX with
    const uint8_t RX_PIN = 16; // the pin we are using to RX with
    const uint8_t EN_PIN = 21; // the pin we are using to enable TX on the DMX transceiver

    uint8_t data[DMX_MAX_PACKET_SIZE] = {0};
    ESP_ERROR_CHECK(dmx_set_pin(DMX_NUM_2, TX_PIN, RX_PIN, EN_PIN));
    ESP_ERROR_CHECK(dmx_driver_install(DMX_NUM_2, DMX_DEFAULT_INTR_FLAGS));

    // TODO use SP_ERROR_CHECK
    rdm_client_init(DMX_NUM_2, rdmDmx->dmxAddr, 13, "LICHTAUSGANG Blinder");
    rdm_client_set_start_address_changed_cb(DMX_NUM_2, [](uint16_t newAddr)
                                            { rdmDmx->updateDmxAddr(newAddr); });

    while (true)
    {
        dmx_packet_t dmxPacket;
        size_t ret = 0;
        ret = dmx_receive(DMX_NUM_2, &dmxPacket, DMX_TIMEOUT_TICK);
        if (ret)
        {
            if (dmxPacket.err == ESP_OK)
            {
                dmx_read(DMX_NUM_2, data, dmxPacket.size);
                if (dmxPacket.is_rdm)
                {
                    rdm_client_handle_rdm_message(DMX_NUM_2, &dmxPacket, data, dmxPacket.size);
                }
                else
                {
                    //FIXME only read part of the buffer directly into rdmDmx?!
                    rdmDmx->newDmxData(data);
                }
            }
            else
            {
                ESP_LOGE(TAG, "dmx error: %s", esp_err_to_name(dmxPacket.err));
            }
        }
    }
}