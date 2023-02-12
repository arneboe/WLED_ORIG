#pragma once

#include "wled.h"
#include "esp_dmx.h"
#include "esp_system.h"
#include "esp_rdm_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>

void startAddressChangedCb(uint16_t newAddr);
void dmxReceiverTask(void *instance);
void IRAM_ATTR dmxSendTask(void *instance);
class RdmDmx;

struct DmxReceiveTaskParams
{
    RdmDmx *rdmDmx;
    uint16_t numPixels;
    std::string defaultPersonalityName;
    uint16_t defaultPersonalityFootprint;
    bool threadRunning = false; // is true when the dmx thread is running
} dmxReceiveParams;             // This is global because the esp_dmx api uses c-callbacks and I cannot capture state in the callback lambdas

class RdmDmx : public Usermod
{

public:
    int dmxAddr;
    std::atomic<int> dmxAddrByRdm;
    int dmxAddrByConfig;
    std::atomic<bool> identify = false;
    TaskHandle_t dmxRcvTaskHandle;
    TaskHandle_t dmxSendTaskHandle;

    /** personality:
     *  0 = wled fixture
     *  1 = pixel mapping no pixels grouped
     *  2 = pixel mapping 2 pixels grouped
     *  3 = pixel mapping 3 pixels grouped
     *  etc..
     */
    uint16_t personality;
    uint16_t personalityByConfig;
    std::atomic<uint8_t> personalityByRdm = 0;

    uint8_t dmxData[513] = {0};

    unsigned long lastDmxPacket = 0;
    bool noDmx = true;

    void setup()
    {
        // FIXME use correct upper bound
        //       should also work in pixel mode
        //       and also check this every time the addr changed
        //       or the personality changed
        if (dmxAddrByConfig < 1 || dmxAddrByConfig > 512)
        {
            dmxAddrByConfig = 1;
            ESP_LOGE("RdmDmx", "dmx address out of bounds. Resetting to 1");
        }
        dmxAddr = dmxAddrByConfig;
        dmxAddrByRdm = dmxAddrByConfig;

        personality = personalityByConfig;
        personalityByRdm = personalityByConfig;

        dmxReceiveParams.rdmDmx = this;
        dmxReceiveParams.numPixels = strip.getLengthTotal();
        // the default personality will get id 1.
        dmxReceiveParams.defaultPersonalityName = "WLED Mode";
        dmxReceiveParams.defaultPersonalityFootprint = 14;

        dmxRcvTaskHandle = NULL;

        // pin to core 0 because wled is running on core 1
        xTaskCreatePinnedToCore(dmxReceiverTask, "DMX_RCV_TASK", 10240, nullptr, 2, &dmxRcvTaskHandle, 0);
        if (!dmxRcvTaskHandle)
        {
            ESP_LOGE("RdmDmx", "Failed to create dmx rcv task");
        }

        dmxSendTaskHandle = NULL;
        xTaskCreatePinnedToCore(dmxSendTask, "DMX_SEND_TASK", 10240, nullptr, 2, &dmxSendTaskHandle, 0);
        if (!dmxSendTaskHandle)
        {
            ESP_LOGE("RdmDmx", "Failed to create dmx send task");
        }
    }

    void loop()
    {
        if (!dmxReceiveParams.threadRunning)
        {
            return;
        }
        if (dmxAddrByRdm != dmxAddr)
        {
            dmxAddr = dmxAddrByRdm;
            dmxAddrByConfig = dmxAddrByRdm;
            serializeConfig();
        }

        if (dmxAddrByConfig != dmxAddr)
        {
            dmxAddr = dmxAddrByConfig;
            dmxAddrByRdm = dmxAddrByConfig;
            rdm_client_set_start_address(DMX_NUM_2, dmxAddr);
        }

        // web interface changed personality, update rdm
        if (personality != personalityByConfig)
        {
            personality = personalityByConfig;
            personalityByRdm = personalityByConfig;
            rdm_client_set_personality(DMX_NUM_2, personality + 1); //+1 because rmd is 1-based
        }

        // rdm changed personality, update web interface
        if (personalityByRdm != personality)
        {
            personality = personalityByRdm;
            personalityByConfig = personalityByRdm;
            serializeConfig();
        }

        noDmx = millis() - lastDmxPacket > 2000;

        dmxReceiveParams.numPixels = strip.getLengthTotal();
        if (dmxReceiveParams.numPixels != strip.getLengthTotal())
        {
            // TODO
            // num pixels has changed, re-calculate personalities
            ESP_LOGE("RdmDmx", "num pixels has changed. please restart to calculate correct personalities");
        }

        updateEffect();
    }

    // notify that new data has been received
    void newDmxData()
    {
        lastDmxPacket = millis();
        //send new dimmer value to dmx task and wake the task up
        xTaskNotifyIndexed(dmxSendTaskHandle, 0, dmxData[dmxAddr], eSetValueWithOverwrite);
    }

    void updateEffect()
    {
        // disable leds if we have not seen dmx signals for some time
        if (noDmx)
        {
            bri = 0;
            transitionDelayTemp = 0;
            colorUpdated(CALL_MODE_NOTIFICATION);
            return;
        }

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
        else if (personality == 0) // WLED effect mode
        {
            //+1 to skip dimmer channel
            const uint16_t addr = std::min(499, dmxAddr) + 1;
            const uint8_t *data = &dmxData[addr];

            realtimeMode = REALTIME_MODE_INACTIVE;
            bri = data[0];
            if (data[1] < strip.getModeCount())
                effectCurrent = data[1];
            effectSpeed = data[2];
            effectIntensity = data[3];
            effectPalette = data[4];
            col[0] = data[5];
            col[1] = data[6];
            col[2] = data[7];
            colSec[0] = data[8];
            colSec[1] = data[9];
            colSec[2] = data[10];

            col[3] = data[11]; // white
            colSec[3] = data[12];
            transitionDelayTemp = 0;              // act fast
            colorUpdated(CALL_MODE_NOTIFICATION); // don't send UDP
        }
    }

    void handleOverlayDraw() override
    {

        if (personality != 0) // pixel mapping mode
        {
            realtimeLock(realtimeTimeoutMs, REALTIME_MODE_E131);
            if (realtimeOverride && !(realtimeMode && useMainSegmentOnly))
                return;

            // FIXMe implement getFootprint instead
            const uint8_t groupPixels = personality;
            // const uint16_t numPixelsGrouped = (dmxParams.numPixels + groupPixels - 1) / groupPixels; // integer math ceil
            // const uint16_t footprint = numPixelsGrouped * 3;

            const uint16_t numPixels = strip.getLengthTotal();
            uint16_t addr = dmxAddr + 1;
            uint8_t currentGrpPixel = 1;

            if (noDmx)
            {
                for (uint16_t i = 0; i < numPixels; ++i)
                {
                    setRealtimePixel(i, 0, 0, 0, 0);
                }
            }
            else
            {
                for (uint16_t i = 0; i < numPixels; ++i)
                {
                    setRealtimePixel(i, dmxData[addr], dmxData[addr + 1], dmxData[addr + 2], 0);
                    currentGrpPixel++;

                    if (currentGrpPixel > groupPixels)
                    {
                        addr += 3;
                        currentGrpPixel = 1;
                        if (addr > 510)
                        { // FIXME calculate beforehand
                            break;
                        }
                    }
                }
            }
            e131NewData = true;
        }
    }

    void addToConfig(JsonObject &root)
    {
        JsonObject top = root.createNestedObject("dmx_rdm_mod");
        top["dmx_addr"] = dmxAddrByConfig;
        top["personality"] = personalityByConfig;
    }

    void updateDmxAddr(uint16_t newAddr)
    {
        if (newAddr > 0 && newAddr <= 512)
        {
            // storing needs to happen in next loop
            dmxAddrByRdm = newAddr;
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
        configComplete &= getJsonValue(top["dmx_addr"], dmxAddrByConfig, 42);
        configComplete &= getJsonValue(top["personality"], personalityByConfig, 0);

        return configComplete;
    }

    uint16_t getId()
    {
        return USERMOD_ID_RDM_DMX;
    }

    // callback coming from rdm client
    void personalityChanged(uint8_t p)
    {
        // rmd personalities are 1-based but we are 0-based
        personalityByRdm = p - 1;
    }
};

void initPersonalities()
{
    // generate pixel mapping personalities
    for (int i = 1; i <= 20; ++i)
    {
        const uint16_t divisor = i;
        const uint16_t numPixelsGrouped = (dmxReceiveParams.numPixels + divisor - 1) / divisor; // integer math ceil
        const uint16_t footprint = numPixelsGrouped * 3;                                        // FIXME support RGBW?
        if (footprint > 511)
        {
            ESP_LOGE(TAG, "dmx error: Too many pixels for footprint %d. Skipping...", divisor);
            continue;
        }
        //+ 1 because first channel is dimmer
        rdm_client_add_personality(DMX_NUM_2, footprint + 1, ("Pixmap (grp " + std::to_string(divisor) + ")").c_str());
    }

    rdm_client_set_personality(DMX_NUM_2, dmxReceiveParams.rdmDmx->personality + 1); //+1 because rmd is 1-based

    rdm_client_set_personality_changed_cb(DMX_NUM_2, [](uint8_t personality)
                                          { dmxReceiveParams.rdmDmx->personalityChanged(personality); });
}

void dmxSendTask(void *)
{

    const uint8_t TX_PIN = 32; // the pin we are using to TX with
    const uint8_t RX_PIN = 33; // the pin we are using to RX with
    const uint8_t EN_PIN = 19; // the pin we are using to enable TX on the DMX transceiver
    uint8_t data[DMX_MAX_PACKET_SIZE] = {0};
    ESP_ERROR_CHECK(dmx_set_pin(DMX_NUM_1, TX_PIN, RX_PIN, EN_PIN));
    ESP_ERROR_CHECK(dmx_driver_install(DMX_NUM_1, DMX_DEFAULT_INTR_FLAGS));

    // FIXME i dont need rdm client here
    rdm_client_init(DMX_NUM_1, dmxReceiveParams.rdmDmx->dmxAddr, dmxReceiveParams.defaultPersonalityFootprint, "LICHTAUSGANG Blinder", dmxReceiveParams.defaultPersonalityName.c_str());

    uint8_t d = 0;
    uint32_t brightness = 0;
    while (true)
    {
        if(xTaskNotifyWaitIndexed(0, 0, 0, &brightness,  pdMS_TO_TICKS(1000)) == pdPASS)
        {
            dmx_write_slot(DMX_NUM_1, 1, uint8_t(brightness));
        }
        else
        {
            //timeout, disable blinder
            dmx_write_slot(DMX_NUM_1, 1, 0);
        }
        //FIXME there is probably no need to send a full frame
        dmx_send(DMX_NUM_1, 513);
    }
}

void dmxReceiverTask(void *)
{
    const uint8_t TX_PIN = 17; // the pin we are using to TX with
    const uint8_t RX_PIN = 16; // the pin we are using to RX with
    const uint8_t EN_PIN = 21; // the pin we are using to enable TX on the DMX transceiver

    uint8_t data[DMX_MAX_PACKET_SIZE] = {0};
    ESP_ERROR_CHECK(dmx_set_pin(DMX_NUM_2, TX_PIN, RX_PIN, EN_PIN));
    ESP_ERROR_CHECK(dmx_driver_install(DMX_NUM_2, DMX_DEFAULT_INTR_FLAGS));
    // TODO use SP_ERROR_CHECK
    rdm_client_init(DMX_NUM_2, dmxReceiveParams.rdmDmx->dmxAddr,
                    dmxReceiveParams.defaultPersonalityFootprint, "LICHTAUSGANG Blinder",
                    dmxReceiveParams.defaultPersonalityName.c_str());

    rdm_client_set_start_address_changed_cb(DMX_NUM_2, [](uint16_t newAddr)
                                            { dmxReceiveParams.rdmDmx->updateDmxAddr(newAddr); });
    rdm_client_set_notify_cb(DMX_NUM_2, [](bool identify)
                             { dmxReceiveParams.rdmDmx->setIdentify(identify); });

    initPersonalities();
    dmxReceiveParams.threadRunning = true;
    while (true)
    {
        dmx_packet_t dmxPacket;
        size_t ret = 0;
        ret = dmx_receive(DMX_NUM_2, &dmxPacket, DMX_TIMEOUT_TICK);
        if (ret)
        {
            if (dmxPacket.err == ESP_OK)
            {
                if (dmxPacket.is_rdm)
                {
                    dmx_read(DMX_NUM_2, data, dmxPacket.size);
                    rdm_client_handle_rdm_message(DMX_NUM_2, &dmxPacket, data, dmxPacket.size);
                }
                else
                {
                    // FIXME only read part of the buffer directly into rdmDmx?!
                    dmx_read(DMX_NUM_2, dmxReceiveParams.rdmDmx->dmxData, dmxPacket.size);
                    // TODO replace with some kind of thread signal feature
                    dmxReceiveParams.rdmDmx->newDmxData();
                }
            }
            else
            {
                ESP_LOGE("RdmDmx", "dmx error: %s", esp_err_to_name(dmxPacket.err));
            }
        }
    }
}