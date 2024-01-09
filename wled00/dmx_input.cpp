#include "wled.h"

#ifdef WLED_ENABLE_DMX_INPUT

#ifdef ESP8266
#error DMX input is only supported on ESP32
#endif

#include "dmx_input.h"
#include "blinder.h"
#include <rdm/responder.h>

static void enabledWifi()
{
  apBehavior = AP_BEHAVIOR_ALWAYS;
}

static void disableWifi()
{
  apBehavior = AP_BEHAVIOR_BUTTON_ONLY;
}

void wifiStateChangedCb(dmx_port_t dmxPort, const rdm_header_t *header,
                        void *context)
{
  if (header->cc == RDM_CC_SET_COMMAND_RESPONSE)
  {
    DMXInput *dmx = static_cast<DMXInput *>(context);

    if (!dmx)
    {
      USER_PRINTLN("DMX: Error: no context in rdmPersonalityChangedCb");
      return;
    }

    if (dmx->wifiState == 1 && apBehavior != AP_BEHAVIOR_ALWAYS)
    {
      // reset ssid to default
      WLED_SET_AP_SSID();

      //disable wifi client
      strcpy(clientSSID, "");

      USER_PRINTLN("wifi data:");
      USER_PRINTLN(apSSID); 
      USER_PRINTLN(apPass); 
      apBehavior = AP_BEHAVIOR_ALWAYS;
      doSerializeConfig = true;
      USER_PRINTF("Wifi enabled via RDM\n");
    }
    else if (dmx->wifiState == 0 && apBehavior != AP_BEHAVIOR_BUTTON_ONLY)
    {
      apBehavior = AP_BEHAVIOR_BUTTON_ONLY;
      doSerializeConfig = true;
      USER_PRINTF("Wifi disabled via RDM\n");
    }
  }

  // dont need to do anything in here
}

void rdmPersonalityChangedCb(dmx_port_t dmxPort, const rdm_header_t *header,
                             void *context)
{
  DMXInput *dmx = static_cast<DMXInput *>(context);

  if (!dmx)
  {
    USER_PRINTLN("DMX: Error: no context in rdmPersonalityChangedCb");
    return;
  }

  if (header->cc == RDM_CC_SET_COMMAND_RESPONSE)
  {
    const uint8_t personality = dmx_get_current_personality(dmx->inputPortNum);
    DMXMode = std::min(DMX_MODE_PRESET, std::max(DMX_MODE_SINGLE_RGB, int(personality)));
    doSerializeConfig = true;
    USER_PRINTF("DMX personality changed to to: %d\n", DMXMode);
  }
}

void rdmAddressChangedCb(dmx_port_t dmxPort, const rdm_header_t *header,
                         void *context)
{
  DMXInput *dmx = static_cast<DMXInput *>(context);

  if (!dmx)
  {
    USER_PRINTLN("DMX: Error: no context in rdmAddressChangedCb");
    return;
  }

  if (header->cc == RDM_CC_SET_COMMAND_RESPONSE)
  {
    const uint16_t addr = dmx_get_start_address(dmx->inputPortNum);
    DMXAddress = std::min(512, int(addr));
    doSerializeConfig = true;
    USER_PRINTF("DMX start addr changed to: %d\n", DMXAddress);
  }
}

static dmx_config_t createConfig()
{
  dmx_config_t config;
  config.pd_size = 255;
  config.dmx_start_address = DMXAddress; // TODO split between input and output address
  config.model_id = 0;
  config.product_category = RDM_PRODUCT_CATEGORY_FIXTURE;
  config.software_version_id = 6;
  strcpy(config.device_label, "LAG_BLINDER");

  const std::string versionString = "BLIND_V6";
  strncpy(config.software_version_label, versionString.c_str(), 32);
  config.software_version_label[32] = '\0'; // zero termination in case versionString string was longer than 32 chars

  config.personalities[0].description = "SINGLE_RGB";
  config.personalities[0].footprint = 4;
  config.personalities[1].description = "SINGLE_DRGB";
  config.personalities[1].footprint = 6;
  config.personalities[2].description = "EFFECT";
  config.personalities[2].footprint = 16;
  config.personalities[3].description = "MULTIPLE_RGB";
  config.personalities[3].footprint = std::min(512, int(strip.getLengthTotal()) * 3 + 1);
  config.personalities[4].description = "MULTIPLE_DRGB";
  config.personalities[4].footprint = std::min(512, int(strip.getLengthTotal()) * 3 + 2);
  config.personalities[5].description = "MULTIPLE_RGBW";
  config.personalities[5].footprint = std::min(512, int(strip.getLengthTotal()) * 4 + 1);
  config.personalities[6].description = "EFFECT_W";
  config.personalities[6].footprint = 19;
  config.personalities[7].description = "EFFECT_SEGMENT";
  config.personalities[7].footprint = std::min(512, strip.getSegmentsNum() * 15 + 1);
  config.personalities[8].description = "EFFECT_SEGMENT_W";
  config.personalities[8].footprint = std::min(512, strip.getSegmentsNum() * 18 + 1);
  config.personalities[9].description = "PRESET";
  config.personalities[9].footprint = 2;

  config.personality_count = 10;
  // rdm personalities are numbered from 1, thus we can just set the DMXMode directly.
  config.current_personality = DMXMode;

  return config;
}

void dmxReceiverTask(void *context)
{
  DMXInput *instance = static_cast<DMXInput *>(context);
  if (instance == nullptr)
  {
    return;
  }

  if (instance->installDriver())
  {
    while (true)
    {
      instance->updateInternal();
    }
  }
}

bool DMXInput::installDriver()
{

  const auto config = createConfig();
  if (!dmx_driver_install(inputPortNum, &config, DMX_INTR_FLAGS_DEFAULT))
  {
    USER_PRINTF("Error: Failed to install dmx driver\n");
    return false;
  }

  USER_PRINTF("Listening for DMX on pin %u\n", rxPin);
  USER_PRINTF("Sending DMX on pin %u\n", txPin);
  USER_PRINTF("DMX enable pin is: %u\n", enPin);
  dmx_set_pin(inputPortNum, txPin, rxPin, enPin);

  rdm_register_dmx_start_address(inputPortNum, rdmAddressChangedCb, this);
  rdm_register_dmx_personality(inputPortNum, rdmPersonalityChangedCb, this);

  // wifi enable parameter
  rdm_pid_description_t desc;
  desc.pid = RDM_PID_MANUFACTURER_SPECIFIC_BEGIN;
  desc.pdl_size = 0x1;
  desc.data_type = RDM_DS_UNSIGNED_BYTE;
  desc.cc = RDM_CC_GET_SET;
  desc.unit = RDM_UNITS_NONE;
  desc.prefix = RDM_PREFIX_NONE;
  desc.min_value = 0;
  desc.max_value = 1;
  desc.default_value = wifiState;
  strlcpy(desc.description, "Wifi Enabled", strlen("Wifi Enabled") + 1);
  const char *param_str = "b$";
  rdm_register_manufacturer_specific_simple(DMX_NUM_2, desc, &wifiState, param_str, wifiStateChangedCb, this);

  initialized = true;
  return true;
}

void DMXInput::init(uint8_t rxPin, uint8_t txPin, uint8_t enPin, uint8_t inputPortNum)
{

#ifdef WLED_ENABLE_DMX_OUTPUT
  // TODO add again once dmx output has been merged
  //  if(inputPortNum == dmxOutputPort)
  //  {
  //    USER_PRINTF("DMXInput: Error: Input port == output port");
  //    return;
  //  }
#endif

  if (apBehavior == AP_BEHAVIOR_BUTTON_ONLY)
  {
    wifiState = 0;
  }
  else
  {
    wifiState = 1;
  }

  if (inputPortNum < 3 && inputPortNum > 0)
  {
    this->inputPortNum = inputPortNum;
  }
  else
  {
    USER_PRINTF("DMXInput: Error: invalid inputPortNum: %d\n", inputPortNum);
    return;
  }

  if (rxPin > 0 && enPin > 0 && txPin > 0)
  {

    const managed_pin_type pins[] = {
        {(int8_t)txPin, false}, // these are not used as gpio pins, thus isOutput is always false.
        {(int8_t)rxPin, false},
        {(int8_t)enPin, false}};
    const bool pinsAllocated = pinManager.allocateMultiplePins(pins, 3, PinOwner::DMX_INPUT);
    if (!pinsAllocated)
    {
      USER_PRINTF("DMXInput: Error: Failed to allocate pins for DMX_INPUT. Pins already in use:\n");
      USER_PRINTF("rx in use by: %s\n", pinManager.getPinOwnerText(rxPin).c_str());
      USER_PRINTF("tx in use by: %s\n", pinManager.getPinOwnerText(txPin).c_str());
      USER_PRINTF("en in use by: %s\n", pinManager.getPinOwnerText(enPin).c_str());
      return;
    }

    this->rxPin = rxPin;
    this->txPin = txPin;
    this->enPin = enPin;

    // put dmx receiver into seperate task because it should not be blocked
    // pin to core 0 because wled is running on core 1
    xTaskCreatePinnedToCore(dmxReceiverTask, "DMX_RCV_TASK", 10240, this, 2, &task, 0);
    if (!task)
    {
      USER_PRINTF("Error: Failed to create dmx rcv task");
    }
  }
  else
  {
    USER_PRINTLN("DMX input disabled due to rxPin, enPin or txPin not set");
    return;
  }
}

void DMXInput::updateInternal()
{
  if (!initialized)
  {
    return;
  }

  checkAndUpdateConfig();

  dmx_packet_t packet;
  unsigned long now = millis();
  if (dmx_receive(inputPortNum, &packet, DMX_TIMEOUT_TICK))
  {
    if (!packet.err)
    {
      connected = true;
      identify = isIdentifyOn();
      if (!packet.is_rdm)
      {
        const std::lock_guard<std::mutex> lock(dmxDataLock);
        dmx_read(inputPortNum, dmxdata, packet.size);
        // forward dimmer channel directly
        setBlinderBrightness(dmxdata[DMXAddress]);

        // reset the effect timebase when the effect changes.
        // This kinda synchronizes the fixtures
        const uint8_t currentEffect = dmxdata[DMXAddress + 2];
        if (currentEffect != lastEffectId)
        {
          lastEffectId = currentEffect;
          strip.resetTime(1);
        }
      }
    }
    else
    {
      connected = false;
    }
  }
  else
  {
    connected = false;
  }
}

void DMXInput::update()
{
  const uint16_t addr = DMXAddress + 1;

  if (identify)
  {
    // white pulsing
    const std::lock_guard<std::mutex> lock(dmxDataLock);
    dmxdata[addr] = 255;     // bri
    dmxdata[addr + 1] = 2;   // effect id
    dmxdata[addr + 2] = 255; // effect speed
    dmxdata[addr + 6] = 255; // r
    dmxdata[addr + 7] = 255; // g
    dmxdata[addr + 8] = 255; // b
    dmxdata[addr + 9] = 0;
    dmxdata[addr + 10] = 0;
    dmxdata[addr + 11] = 0;
    dmxdata[addr + 12] = 0;
    dmxdata[addr + 13] = 0;
    dmxdata[addr + 14] = 0;
    handleDMXData(1, 512, dmxdata + 1, REALTIME_MODE_DMX, 0);
  }
  else if(oldIdentify && !identify)
  {
    //identify was turned off, disable brightness. Otherwise it just stays on
    //while the rdm tester is connected
    dmxdata[addr] = 0;
  }
  else if (connected)
  {
    const std::lock_guard<std::mutex> lock(dmxDataLock);
    // blinder: we move dmxdata one byte forward. This way we hack the blinder channel infront of the
    //          dmx data but we can keep the dmx addr. This could lead to out of bounds access.
    //          We have to ensure that that does not happen
    handleDMXData(1, 512, dmxdata + 1, REALTIME_MODE_DMX, 0);
  }
  else
  {
    //not connected animation
    const std::lock_guard<std::mutex> lock(dmxDataLock);
    dmxdata[addr] = 255;     // bri
    dmxdata[addr + 1] = 188;   // effect id
    //the not-connected effect ignores all other values

    handleDMXData(1, 512, dmxdata + 1, REALTIME_MODE_DMX, 0);
  }

  oldIdentify = identify;
}


void DMXInput::disable()
{
  if (initialized)
  {
    dmx_driver_disable(inputPortNum);
  }
}
void DMXInput::enable()
{
  if (initialized)
  {
    dmx_driver_enable(inputPortNum);
  }
}

bool DMXInput::isIdentifyOn() const
{

  uint8_t identify = 0;
  const bool gotIdentify = rdm_get_identify_device(inputPortNum, &identify);
  // gotIdentify should never be false because it is a default parameter in rdm
  // but just in case we check for it anyway
  return bool(identify) && gotIdentify;
}

void DMXInput::checkAndUpdateConfig()
{

  /**
   * The global configuration variables are modified by the web interface.
   * If they differ from the driver configuration, we have to update the driver
   * configuration.
   */

  const uint8_t currentPersonality = dmx_get_current_personality(inputPortNum);
  if (currentPersonality != DMXMode)
  {
    DEBUG_PRINTF("DMX personality has changed from %d to %d\n", currentPersonality, DMXMode);
    dmx_set_current_personality(inputPortNum, DMXMode);
  }

  const uint16_t currentAddr = dmx_get_start_address(inputPortNum);
  if (currentAddr != DMXAddress)
  {
    DEBUG_PRINTF("DMX address has changed from %d to %d\n", currentAddr, DMXAddress);
    dmx_set_start_address(inputPortNum, DMXAddress);
  }

  // TODO somehow figure if that the webinterface changed the wifi state
}

#endif