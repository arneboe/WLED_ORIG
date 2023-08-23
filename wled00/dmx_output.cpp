#include "wled.h"

/*
 * Support for DMX output via serial (e.g. MAX485).
 * Change the output pin in src/dependencies/ESPDMX.cpp, if needed (ESP8266)
 * ESP8266 Library from:
 * https://github.com/Rickgg/ESP-Dmx
 * ESP32 Library from:
 * https://github.com/someweisguy/esp_dmx
 */

#ifdef WLED_ENABLE_DMX_OUTPUT

// WLEDMM: seems that DMX output triggers watchdog resets when compiling for IDF 4.4.x
#ifdef ARDUINO_ARCH_ESP32
#include <esp_dmx.h>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 2, 0)
#warning DMX output support might cause watchdog reset when compiling with ESP-IDF V4.4.x
// E (24101) task_wdt: Task watchdog got triggered. The following tasks did not reset the watchdog in time:
// E (24101) task_wdt:  - IDLE (CPU 0)
// E (24101) task_wdt: Tasks currently running:
// E (24101) task_wdt: CPU 0: FFT
// E (24101) task_wdt: CPU 1: IDLE
// E (24101) task_wdt: Aborting.
// abort() was called at PC 0x40143b6c on core 0
#endif
#endif

static bool dmxOutputInitialized = false;

void dmxWrite(uint16_t addr, uint8_t data)
{
#ifdef ESP8266
  dmx.write(addr, data);
#else
  dmx_write_slot(dmxOutputPort, addr, data);
#endif
}

void dmxSend()
{
#ifdef ESP8266
  dmx.update();
#else
  dmx_send(dmxOutputPort, 512);
#endif
}

void handleDMXOutput()
{
  if (!dmxOutputInitialized)
    return;
  // don't act, when in DMX Proxy mode
  if (e131ProxyUniverse != 0)
    return;

  uint8_t brightness = strip.getBrightness();

  bool calc_brightness = true;

  // check if no shutter channel is set
  for (byte i = 0; i < DMXChannels; i++)
  {
    if (DMXFixtureMap[i] == 5)
      calc_brightness = false;
  }

  uint16_t len = strip.getLengthTotal();
  for (int i = DMXStartLED; i < len; i++)
  { // uses the amount of LEDs as fixture count

    uint32_t in = strip.getPixelColor(i); // get the colors for the individual fixtures as suggested by Aircoookie in issue #462
    byte w = W(in);
    byte r = R(in);
    byte g = G(in);
    byte b = B(in);

    int DMXFixtureStart = DMXStart + (DMXGap * (i - DMXStartLED));
    for (int j = 0; j < DMXChannels; j++)
    {
      int DMXAddr = DMXFixtureStart + j;
      switch (DMXFixtureMap[j])
      {
      case 0: // Set this channel to 0. Good way to tell strobe- and fade-functions to fuck right off.
        dmxWrite(DMXAddr, 0);
        break;
      case 1: // Red
        dmxWrite(DMXAddr, calc_brightness ? (r * brightness) / 255 : r);
        break;
      case 2: // Green
        dmxWrite(DMXAddr, calc_brightness ? (g * brightness) / 255 : g);
        break;
      case 3: // Blue
        dmxWrite(DMXAddr, calc_brightness ? (b * brightness) / 255 : b);
        break;
      case 4: // White
        dmxWrite(DMXAddr, calc_brightness ? (w * brightness) / 255 : w);
        break;
      case 5: // Shutter channel. Controls the brightness.
        dmxWrite(DMXAddr, brightness);
        break;
      case 6: // Sets this channel to 255. Like 0, but more wholesome.
        dmxWrite(DMXAddr, 255);
        break;
      }
    }
  }

  dmxSend(); // update the DMX bus
}

void initDMXOutput()
{
#ifdef ESP8266
  dmx.init(512); // initialize with bus length
  dmxOutputInitialized = true;
#else

  if (dmxOutputEnablePin > 0 && dmxOutputReceivePin > 0 && dmxOutputTransmitPin > 0)
  {
    const managed_pin_type pins[] = {
        {(int8_t)dmxOutputTransmitPin, false},
        {(int8_t)dmxOutputReceivePin, false},
        {(int8_t)dmxOutputEnablePin, false}};
    const bool pinsAllocated = pinManager.allocateMultiplePins(pins, 3, PinOwner::DMX_OUTPUT);
    if (!pinsAllocated)
    {
      USER_PRINTF("Error: Failed to allocate pins for DMX_OUTPUT. Pins already in use:\n");
      USER_PRINTF("rx in use by: %s\n", pinManager.getPinOwnerText(dmxOutputReceivePin).c_str());
      USER_PRINTF("tx in use by: %s\n", pinManager.getPinOwnerText(dmxOutputTransmitPin).c_str());
      USER_PRINTF("en in use by: %s\n", pinManager.getPinOwnerText(dmxOutputEnablePin).c_str());
      return;
    }

    const auto cfg = DMX_CONFIG_DEFAULT;
    if (!dmx_driver_install(dmxOutputPort, &cfg, DMX_INTR_FLAGS_DEFAULT))
    {
      USER_PRINTF("Error: Failed to install dmx driver\n");
      return;
    }
    dmx_set_pin(dmxOutputPort, dmxOutputTransmitPin, dmxOutputReceivePin, dmxOutputEnablePin);
    USER_PRINTF("DMX output initialized\n");
    dmxOutputInitialized = true;
  }
  else
  {
    USER_PRINTLN("DMX output disabled because rx pin, enable pin or tx pin not set");
    return;
  }

#endif
}
#else
void initDMXOutput() {}
void handleDMXOutput() {}
void dmxWrite(uint16_t addr, uint8_t data) {}
void dmxSend() {}

#endif