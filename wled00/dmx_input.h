#pragma once
#include <cstdint>
#include <esp_dmx.h>
#include <atomic>
#include <mutex>

/*
 * Support for DMX/RDM input via serial (e.g. max485) on ESP32
 * ESP32 Library from:
 * https://github.com/sparkfun/SparkFunDMX
 */
class DMXInput
{
public:
  void init(uint8_t rxPin, uint8_t txPin, uint8_t enPin, uint8_t inputPortNum);
  void update();

  /**disable dmx receiver (do this before disabling the cache)*/
  void disable();
  void enable();

private:
  /// @return true if rdm identify is active
  bool isIdentifyOn() const;

  /**
   * Checks if the global dmx config has changed and updates the changes in rdm
   */
  void checkAndUpdateConfig();


  /// installs the dmx driver
  /// @return false on fail
  bool installDriver();

  /// is called by the dmx receive task regularly to receive new dmx data
  void updateInternal();

  // is invoked whenver the dmx start address is changed via rdm
  friend void rdmAddressChangedCb(dmx_port_t dmxPort, const rdm_header_t *header,
                                  void *context);

  // is invoked whenever the personality is changed via rdm
  friend void rdmPersonalityChangedCb(dmx_port_t dmxPort, const rdm_header_t *header,
                                      void *context);

  // invoked whenever the user wants to enable/disable wifi via rdm
  friend void wifiStateChangedCb(dmx_port_t dmxPort, const rdm_header_t *header,
                                 void *context);

  /// The internal dmx task.
  /// This is the main loop of the dmx receiver. It never returns.
  friend void dmxReceiverTask(void *context);

  uint8_t inputPortNum = 255;
  uint8_t rxPin = 255;
  uint8_t txPin = 255;
  uint8_t enPin = 255;

  //contains the wifi state for rdm
  uint8_t wifiState;

  //the last effect that was set via dmx
  uint8_t lastEffectId = 255;

  /// is written to by the dmx receive task.
  // +1 because we shift it later and need to avoid out of bounds access
  byte dmxdata[DMX_PACKET_SIZE + 1]; // TODO add locking somehow? maybe double buffer?
  /// True once the dmx input has been initialized successfully
  bool initialized = false; // true once init finished successfully
  /// True if dmx is currently connected
  std::atomic<bool> connected{false};
  std::atomic<bool> identify{false};
  bool oldIdentify{false}; //used to detect if identify changed
  /// Timestamp of the last time a dmx frame was received
  unsigned long lastUpdate = 0;

  /// Taskhandle of the dmx task that is running in the background
  TaskHandle_t task;
  /// Guards access to dmxData
  std::mutex dmxDataLock;
};
