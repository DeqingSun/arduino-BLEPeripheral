// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef _BLE_DEVICE_H_
#define _BLE_DEVICE_H_

#if defined(__RFduino__)
  #include <utility/RFduino/ble_gatts.h>
  #include <utility/RFduino/ble_gattc.h>
#elif defined(NRF5) || defined(NRF51_S130)
  #include <ble_gatts.h>
  #include <ble_gattc.h>
  #include <nrf_soc.h>
#elif defined(NRF52) && defined(S132) // ARDUINO_RBL_nRF52832
  #ifndef ARDUINO_RBL_nRF52832
    #define ARDUINO_RBL_nRF52832
  #endif
  #define NRF5

  #include <sdk/softdevice/s132/headers/nrf_ble_gatts.h>
  #include <sdk/softdevice/s132/headers/nrf_ble_gattc.h>
  #include <sdk/softdevice/s132/headers/nrf_soc.h>
#else
  #include <s110/ble_gatts.h>
  #include <s110/ble_gattc.h>
#endif

#include "BLEBondStore.h"
#include "BLECharacteristic.h"
#include "BLELocalAttribute.h"
#include "BLERemoteAttribute.h"
#include "BLERemoteCharacteristic.h"
#include "BLERemoteService.h"

struct BLEEirData
{
  unsigned char length;
  unsigned char type;
  unsigned char data[BLE_EIR_DATA_MAX_VALUE_LENGTH];
};

class BLEDevice;

class BLEDeviceEventListener
{
  public:
    virtual void BLEDeviceConnected(BLEDevice& /*device*/, const unsigned char* /*address*/) { }
    virtual void BLEDeviceDisconnected(BLEDevice& /*device*/) { }
    virtual void BLEDeviceBonded(BLEDevice& /*device*/) { }
    virtual void BLEDeviceRemoteServicesDiscovered(BLEDevice& /*device*/) { }
    virtual void BLEDevicePasskeyReceived(BLEDevice& /*device*/) {}
    virtual void BLEDevicePasskeyRequested(BLEDevice& /*device*/) {}

    virtual void BLEDeviceCharacteristicValueChanged(BLEDevice& /*device*/, BLECharacteristic& /*characteristic*/, const unsigned char* /*value*/, unsigned char /*valueLength*/) { }
    virtual void BLEDeviceCharacteristicSubscribedChanged(BLEDevice& /*device*/, BLECharacteristic& /*characteristic*/, bool /*subscribed*/) { }

    virtual void BLEDeviceRemoteCharacteristicValueChanged(BLEDevice& /*device*/, BLERemoteCharacteristic& /*characteristic*/, const unsigned char* /*value*/, unsigned char /*valueLength*/) { }


    virtual void BLEDeviceAddressReceived(BLEDevice& /*device*/, const unsigned char* /*address*/) { }
    virtual void BLEDeviceTemperatureReceived(BLEDevice& /*device*/, float /*temperature*/) { }
    virtual void BLEDeviceBatteryLevelReceived(BLEDevice& /*device*/, float /*batteryLevel*/) { }
};


class BLEDevice
{
  friend class BLEPeripheral;

  protected:
    BLEDevice();

    virtual ~BLEDevice();

    void setEventListener(BLEDeviceEventListener* eventListener);

    void setAdvertisingInterval(unsigned short advertisingInterval);
    void setConnectionInterval(unsigned short minimumConnectionInterval, unsigned short maximumConnectionInterval);
    void setConnectable(bool connectable);
    void setBondStore(BLEBondStore& bondStore);
    void sendPasskey(char passkey[]);
    void confirmPasskey(bool confirm);
    void setStaticKeysOpt(char* key);
    

    virtual void begin(unsigned char /*advertisementDataSize*/,
                BLEEirData * /*advertisementData*/,
                unsigned char /*scanDataSize*/,
                BLEEirData * /*scanData*/,
                BLELocalAttribute** /*localAttributes*/,
                unsigned char /*numLocalAttributes*/,
                BLERemoteAttribute** /*remoteAttributes*/,
                unsigned char /*numRemoteAttributes*/) { }

    virtual void poll() { }

    virtual void end() { }

    virtual bool setTxPower(int /*txPower*/) { return false; }

    virtual void startAdvertising() { }
    virtual void disconnect() { }

    virtual bool updateCharacteristicValue(BLECharacteristic& /*characteristic*/) { return false; }
    virtual bool broadcastCharacteristic(BLECharacteristic& /*characteristic*/) { return false; }
    virtual bool canNotifyCharacteristic(BLECharacteristic& /*characteristic*/) { return false; }
    virtual bool canIndicateCharacteristic(BLECharacteristic& /*characteristic*/) { return false; }

    virtual bool canReadRemoteCharacteristic(BLERemoteCharacteristic& /*characteristic*/) { return false; }
    virtual bool readRemoteCharacteristic(BLERemoteCharacteristic& /*characteristic*/) { return false; }
    virtual bool canWriteRemoteCharacteristic(BLERemoteCharacteristic& /*characteristic*/) { return false; }
    virtual bool writeRemoteCharacteristic(BLERemoteCharacteristic& /*characteristic*/, const unsigned char /*value*/[], unsigned char /*length*/) { return false; }
    virtual bool canSubscribeRemoteCharacteristic(BLERemoteCharacteristic& /*characteristic*/) { return false; }
    virtual bool subscribeRemoteCharacteristic(BLERemoteCharacteristic& /*characteristic*/) { return false; }
    virtual bool canUnsubscribeRemoteCharacteristic(BLERemoteCharacteristic& /*characteristic*/) { return false; }
    virtual bool unsubcribeRemoteCharacteristic(BLERemoteCharacteristic& /*characteristic*/) { return false; }

    virtual void requestAddress() { }
    virtual void requestTemperature() { }
    virtual void requestBatteryLevel() { }

  protected:
    uint16_t                      _connectionHandle;
    unsigned short                _advertisingInterval;
    unsigned short                _minimumConnectionInterval;
    unsigned short                _maximumConnectionInterval;
    bool                          _connectable;
    bool                          _mitm;
    uint8_t                       _io_caps;
    uint8_t                       _passkey[6];
    bool                          _userConfirm;
    BLEBondStore*                 _bondStore;
    BLEDeviceEventListener*       _eventListener;
};

#endif
