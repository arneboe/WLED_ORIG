#pragma once

#include "wled.h"
#include "esp_dmx.h"
#include "esp_system.h"
#include "esp_rdm_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>

void startAddressChangedCb(uint16_t newAddr);
void dmxTask(void *instance);
class RdmDmx;

struct DmxTaskParams
{
    RdmDmx *rdmDmx;
    uint16_t numPixels;
    std::string defaultPersonalityName;
    uint16_t defaultPersonalityFootprint;
    std::atomic<uint8_t> personality = 0;

} dmxParams; // This is global because the esp_dmx api uses c-callbacks and I cannot capture state in the callback lambdas

class RdmDmx : public Usermod
{

public:
    int dmxAddr;
    std::atomic<int> newDmxAddr;
    std::atomic<bool> identify = false;

    /** personality:
     *  0 = wled fixture
     *  1 = pixel mapping no pixels grouped
     *  2 = pixel mapping 2 pixels grouped
     *  3 = pixel mapping 3 pixels grouped
     *  etc..
     */
    uint8_t dmxData[13] = {0}; // FIXME hardcoded 13

    void setup()
    {

        // FIXME use correct upper bound
        if (dmxAddr < 1 || dmxAddr > 512)
        {
            dmxAddr = 1;
            ESP_LOGE("RdmDmx", "dmx address out of bounds. Resetting to 1");
        }
        newDmxAddr = dmxAddr;

        dmxParams.rdmDmx = this;
        dmxParams.numPixels = strip.getLengthTotal();
        // the default personality will get id 1.
        dmxParams.defaultPersonalityName = "WLED Mode";
        dmxParams.defaultPersonalityFootprint = 13;

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

        dmxParams.numPixels = strip.getLengthTotal();
        if (dmxParams.numPixels != strip.getLengthTotal())
        {
            // TODO
            // num pixels has changed, re-calculate personalities
            ESP_LOGE("RdmDmx", "num pixels has changed. please restart to calculate correct personalities");
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

        // setPixelColor
        // show
        //  auf strip

        if (identify)
        {
            bri = 255;
            effectCurrent = 0;
            effectPalette = 0;
            col[0] = 255;
            col[1] = 255;
            col[2] = 255;
            col[3] = 255;
            colSec[0] = 255;
            colSec[1] = 255;
            colSec[2] = 255;
            colSec[3] = 255;
            transitionDelayTemp = 0;
            colorUpdated(CALL_MODE_NOTIFICATION);
        }
        else
        {
            if (bri != dmxData[0])
            {
                bri = dmxData[0];
            }
            if (dmxData[1] < strip.getModeCount())
                effectCurrent = dmxData[1];
            effectSpeed = dmxData[2];
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
    }

    void addToConfig(JsonObject &root)
    {
        JsonObject top = root.createNestedObject("dmx_rdm_mod");
        top["dmx_addr"] = dmxAddr;
        top["personality"] = dmxParams.personality.load();
    }

    void updateDmxAddr(uint16_t newAddr)
    {
        if (newAddr > 0 && newAddr <= 512)
        {
            // storing needs to happen in next loop
            newDmxAddr = newAddr;
        }
    }

    void setIdentify(bool _identify)
    {
        identify = _identify;
    }

    bool readFromConfig(JsonObject &root)
    {
        JsonObject top = root["dmx_rdm_mod"];
        bool configComplete = !top.isNull();
        configComplete &= getJsonValue(top["dmx_addr"], dmxAddr, 42);
        uint8_t person = 0;
        configComplete &= getJsonValue(top["personality"], person, 0);
        dmxParams.personality = person;
        rdm_client_set_personality(DMX_NUM_2, person + 1); //+1 because rmd is 1-based
        return configComplete;
    }

    uint16_t getId()
    {
        return USERMOD_ID_RDM_DMX;
    }

    // callback coming from rdm client
    void personalityChanged(uint8_t newPersonality)
    {
        // rmd personalities are 1-based but we are 0-based
        dmxParams.personality = newPersonality - 1;
        serializeConfig();
    }
};

void initPersonalities()
{
    // generate pixel mapping personalities
    for (int i = 1; i <= 20; ++i)
    {
        const uint16_t divisor = i;
        const uint16_t numPixelsGrouped = (dmxParams.numPixels + divisor - 1) / divisor; // integer math ceil
        const uint16_t footprint = numPixelsGrouped * 3;                                 // FIXME support RGBW?
        if (footprint > 512)
        {
            ESP_LOGE(TAG, "dmx error: Too many pixels for footprint %d. Skipping...", divisor);
            continue;
        }
        rdm_client_add_personality(DMX_NUM_2, footprint, ("Pixmap (grp " + std::to_string(divisor) + ")").c_str());
    }

    rdm_client_set_personality(DMX_NUM_2, dmxParams.personality + 1); //+1 because rmd is 1-based

    rdm_client_set_personality_changed_cb(DMX_NUM_2, [](uint8_t personality)
                                          { dmxParams.rdmDmx->personalityChanged(personality); });
}

void dmxTask(void *)
{
    const uint8_t TX_PIN = 17; // the pin we are using to TX with
    const uint8_t RX_PIN = 16; // the pin we are using to RX with
    const uint8_t EN_PIN = 21; // the pin we are using to enable TX on the DMX transceiver

    uint8_t data[DMX_MAX_PACKET_SIZE] = {0};
    ESP_ERROR_CHECK(dmx_set_pin(DMX_NUM_2, TX_PIN, RX_PIN, EN_PIN));
    ESP_ERROR_CHECK(dmx_driver_install(DMX_NUM_2, DMX_DEFAULT_INTR_FLAGS));
    // TODO use SP_ERROR_CHECK
    rdm_client_init(DMX_NUM_2, dmxParams.rdmDmx->dmxAddr, dmxParams.defaultPersonalityFootprint, "LICHTAUSGANG Blinder", dmxParams.defaultPersonalityName.c_str());

    rdm_client_set_start_address_changed_cb(DMX_NUM_2, [](uint16_t newAddr)
                                            { dmxParams.rdmDmx->updateDmxAddr(newAddr); });
    rdm_client_set_notify_cb(DMX_NUM_2, [](bool identify)
                             { dmxParams.rdmDmx->setIdentify(identify); });

    initPersonalities();

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
                    // FIXME only read part of the buffer directly into rdmDmx?!
                    dmxParams.rdmDmx->newDmxData(data);
                }
            }
            else
            {
                ESP_LOGE("RdmDmx", "dmx error: %s", esp_err_to_name(dmxPacket.err));
            }
        }
    }
}