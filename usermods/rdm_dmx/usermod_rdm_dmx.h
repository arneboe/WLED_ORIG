#pragma once

#include "wled.h"
#include "esp_dmx.h"
#include "esp_system.h"
#include "esp_rdm_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>

/**
 * TODO:
 *  - blink first and last led red when no data is received on any input
 *  - strobo channel overlay for all effects
 *  - fix wifi crash
 */

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
    bool oldIdentify = false;
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
    uint8_t strobe = 0;

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
        dmxReceiveParams.defaultPersonalityFootprint = 17;

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

    void checkConfigChanges()
    {
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

        dmxReceiveParams.numPixels = strip.getLengthTotal();
        if (dmxReceiveParams.numPixels != strip.getLengthTotal())
        {
            // num pixels has changed, we need to re-calculate the personalities (just reset the chip for now)
            ESP_LOGE("RdmDmx", "num pixels has changed. please restart to calculate correct personalities");
            ESP.restart();
        }
    }

    void loop()
    {
        if (!dmxReceiveParams.threadRunning)
        {
            return;
        }
        checkConfigChanges();

        if (identify)
        {
            oldIdentify = true;
            strobe = 0;
            setEffect(255, 2, 128, 255, 0, 0, 255, 255, 0, 0, 0, 0, 0, 0, 0);
            return;
        }
        else if(oldIdentify)
        {
            oldIdentify = false;
            setBrightness(0); //turn it back off, even if dmx is not connected
        }

        // dmx timeout
        if (millis() - lastDmxPacket < 2000)
        {
            updateEffect();
        }
        else
        {
            // dimmer will be disabled once dmxSendTask timeouts (after 1 sec)
            // leds will remain in their last state
        }
    }

    /// notify that new data has been received
    void newDmxData()
    {
        lastDmxPacket = millis();
        // send new dimmer value to dmx task and wake the task up
        xTaskNotifyIndexed(dmxSendTaskHandle, 0, dmxData[dmxAddr], eSetValueWithOverwrite);
    }

    /// @brief sets brightness but honors strobe channel
    /// @return true if brightness changed
    bool setBrightness(uint8_t brightness)
    {
        // how long the strobe-on flash is
        static const float strobeOnTime = 0.008f;
        const float currentTime = millis() / 1000.0f;
        static float lastTime = currentTime; //<< static!!!
        static bool strobeOn = false;

        if (strobe > 0)
        {
            if (strobeOn)
            {
                const float timeSinceOn = currentTime - lastTime;
                if (timeSinceOn > strobeOnTime)
                {
                    strobeOn = false;
                    bri = 0;
                    strip.setBrightness(0, true);
                    lastTime = currentTime;
                    return true;
                }
            }
            else
            {
                // strobe is always at least 1 when we are in this case
                const float strobeOffTime = (255 - strobe) * 0.01f;
                const float timeSinceOff = currentTime - lastTime;
                if (timeSinceOff > strobeOffTime)
                {
                    strobeOn = true;
                    bri = brightness;
                    strip.setBrightness(brightness, true);
                    lastTime = currentTime;
                    return true;
                }
            }
        }
        else
        {
            if (brightness != strip.getBrightness())
            {
                bri = brightness; // need to set the global bri aswell for wled internals to work
                strip.setBrightness(brightness, true);
                return true;
            }
        }
        return false;
    }

    void updateEffect()
    {

        strobe = dmxData[dmxAddr + 1];

        if (personality == 0) // WLED effect mode
        {
            // 15 channels [bri,effectCurrent,effectSpeed,effectIntensity,effectPalette,effectOption,R,G,B,R2,G2,B2,R3,G3,B3]
            const int numEffectChannels = 15;
            const int maxAddr = 513 - numEffectChannels;
            const uint16_t addr = std::min(maxAddr, dmxAddr + 2); // +2 because 0=blinder, 1=strobe
            const uint8_t *data = &dmxData[addr];

            realtimeMode = REALTIME_MODE_INACTIVE; // TODO check if still need

            // static color fx is not a real fx, it is handled somehow differently in wled (this breaks our strobe)
            // we implement it ourselfs...
            // TODO not sure if I really need this anymore
            if (data[0] == FX_MODE_STATIC)
            {
                fxStatic(data[0], data[6], data[7], data[8]);
            }
            else
            {

                setEffect(data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
                          data[8], data[9], data[10], data[11], data[12], data[13], data[14]);
            }
        }
        else if (personality > 0)
        {
            pixelMapping();
        }
    }

    void fxStatic(uint8_t brightness, uint8_t r, uint8_t g, uint8_t b)
    {
        const uint16_t numPixels = strip.getLengthTotal();

        setBrightness(brightness);

        for (uint16_t i = 0; i < numPixels; ++i)
        {
            const uint8_t gr = gamma8(r);
            const uint8_t gg = gamma8(g);
            const uint8_t gb = gamma8(b);

            strip.setPixelColor(i, gr, gg, gb, 0);
        }

        strip.show();
    }

    void pixelMapping()
    {

        // DEBUG_PRINTF("map\n");

        // FIXMe implement getFootprint instead
        const uint8_t groupPixels = personality;
        // const uint16_t numPixelsGrouped = (dmxParams.numPixels + groupPixels - 1) / groupPixels; // integer math ceil
        // const uint16_t footprint = numPixelsGrouped * 3;

        const uint16_t numPixels = strip.getLengthTotal();
        uint16_t addr = dmxAddr + 2; // +2 because 0=blinder, 1=strobe
        uint8_t currentGrpPixel = 1;

        setBrightness(255);

        for (uint16_t i = 0; i < numPixels; ++i)
        {
            const uint8_t r = gamma8(dmxData[addr]);
            const uint8_t g = gamma8(dmxData[addr + 1]);
            const uint8_t b = gamma8(dmxData[addr + 2]);

            strip.setPixelColor(i, r, g, b, 0);
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

        strip.show();
    }

    void setEffect(uint8_t masterBrightness, uint8_t effectCurrent, uint8_t effectSpeed, uint8_t effectIntensity, uint8_t effectPalette, uint8_t effectOption,
                   uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2, uint8_t r3, uint8_t g3, uint8_t b3)
    {
        //  DEBUG_PRINTF("set effect: %i %i %i %i %i %i %i %i %i \n", masterBrightness, effectCurrent, effectSpeed, effectIntensity,
        //               effectPalette, effectOption, r1, g1, b1);
        //  this loop is mostly copy&paste from e131.cpp

        bool changed = false;
        for (uint8_t id = 0; id < strip.getSegmentsNum(); id++)
        {
            // DEBUG_PRINTF("Segment: %i\n", id);
            Segment &seg = strip.getSegment(id);

            const uint8_t mode = std::min(strip.getModeCount(), effectCurrent);
            if (seg.mode != mode)
            {
                seg.setMode(mode);
                // When running several esps in a large installation, their time drifts apart significantly.
                // To get synchronized effects in that case, we reset the time every time the effect changes.
                // This can be used by the dmx controller to synchronize severeal esps.

                //all esps receive the dmx packet at the same time. But the setEffect function
                //is called at different times.
                //We want to reset the time to the point where the dmx packet was received
                const unsigned long currentTime = millis();
                const unsigned long diff = currentTime - lastDmxPacket;
                strip.resetTime(diff);
                changed = true;
            }
            if (seg.speed != effectSpeed)
            {
                seg.speed = effectSpeed;
                changed = true;
            }
            if (seg.intensity != effectIntensity)
            {
                seg.intensity = effectIntensity;
                changed = true;
            }
            if (seg.palette != effectPalette)
            {
                seg.setPalette(effectPalette);
                changed = true;
            }
            const uint8_t segOption = (uint8_t)floor(effectOption / 64.0);
            if (segOption == 0 && (seg.mirror || seg.reverse))
            {
                seg.setOption(SEG_OPTION_MIRROR, false);
                seg.setOption(SEG_OPTION_REVERSED, false);
                changed = true;
            }
            if (segOption == 1 && (seg.mirror || !seg.reverse))
            {
                seg.setOption(SEG_OPTION_MIRROR, false);
                seg.setOption(SEG_OPTION_REVERSED, true);
                changed = true;
            }
            if (segOption == 2 && (!seg.mirror || seg.reverse))
            {
                seg.setOption(SEG_OPTION_MIRROR, true);
                seg.setOption(SEG_OPTION_REVERSED, false);
                changed = true;
            }
            if (segOption == 3 && (!seg.mirror || !seg.reverse))
            {
                seg.setOption(SEG_OPTION_MIRROR, true);
                seg.setOption(SEG_OPTION_REVERSED, true);
                changed = true;
            }

            uint32_t colors[3];
            colors[0] = RGBW32(r1, g1, b1, 0);
            colors[1] = RGBW32(r2, g2, b2, 0);
            colors[2] = RGBW32(r3, g3, b3, 0);
            if (colors[0] != seg.colors[0])
            {
                seg.setColor(0, colors[0]);
                changed = true;
            }
            if (colors[1] != seg.colors[1])
            {
                seg.setColor(1, colors[1]);
                changed = true;
            }
            if (colors[2] != seg.colors[2])
            {
                seg.setColor(2, colors[2]);
                changed = true;
            }

            // all segments are always fully visible
            if (seg.opacity != 255)
            {
                seg.setOpacity(255);
                changed = true;
            }
        }

        changed |= setBrightness(masterBrightness);

        if (changed)
        {
            fadeTransition = false;
            transitionDelay = 0;
            transitionDelayTemp = 0;
            stateUpdated(CALL_MODE_NOTIFICATION);
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
        // +2 because 0=blinder, 1=strobe
        rdm_client_add_personality(DMX_NUM_2, footprint + 2, ("Pixmap (grp " + std::to_string(divisor) + ")").c_str());
    }

    rdm_client_set_personality(DMX_NUM_2, dmxReceiveParams.rdmDmx->personality + 1); //+1 because rmd is 1-based

    rdm_client_set_personality_changed_cb(DMX_NUM_2, [](uint8_t personality)
                                          { dmxReceiveParams.rdmDmx->personalityChanged(personality); });
}

/// This task forwards the dimmer data to the external dimmer
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
                             { 
                                // DEBUG_PRINTF("Ident: %i\n", identify);
                                dmxReceiveParams.rdmDmx->setIdentify(identify); });

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