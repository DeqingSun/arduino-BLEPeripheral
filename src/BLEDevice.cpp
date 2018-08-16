// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "Arduino.h"

#include "BLEDevice.h"

#include <ble.h>

#define DEFAULT_ADVERTISING_INTERVAL 100
#define DEFAULT_CONNECTABLE          true

BLEDevice::BLEDevice() :
  _connectionHandle(BLE_CONN_HANDLE_INVALID),
  _advertisingInterval(DEFAULT_ADVERTISING_INTERVAL),
  _minimumConnectionInterval(0),
  _maximumConnectionInterval(0),
  _connectable(DEFAULT_CONNECTABLE),
  _bondStore(NULL),
  _eventListener(NULL),
  _mitm(false),
  _io_caps(BLE_GAP_IO_CAPS_NONE),
  _passkey({0,0,0,0,0,0}),
  _userConfirm(false)
{
}

BLEDevice::~BLEDevice() {
   
}

void BLEDevice::setEventListener(BLEDeviceEventListener* eventListener) {
  this->_eventListener = eventListener;
}

void BLEDevice::setAdvertisingInterval(unsigned short advertisingInterval) {
  this->_advertisingInterval = advertisingInterval;
}

void BLEDevice::setConnectionInterval(unsigned short minimumConnectionInterval, unsigned short maximumConnectionInterval) {
  if (maximumConnectionInterval < minimumConnectionInterval) {
    maximumConnectionInterval = minimumConnectionInterval;
  }

  this->_minimumConnectionInterval = minimumConnectionInterval;
  this->_maximumConnectionInterval = maximumConnectionInterval;
}

void BLEDevice::setConnectable(bool connectable) {
  this->_connectable = connectable;
}

void BLEDevice::setBondStore(BLEBondStore& bondStore) {
  this->_bondStore = &bondStore;
}

void BLEDevice::sendPasskey(char passkey[]){
  sd_ble_gap_auth_key_reply(this->_connectionHandle, BLE_GAP_AUTH_KEY_TYPE_PASSKEY, (uint8_t *)passkey);
}

void BLEDevice::confirmPasskey(bool confirm){
  this->_userConfirm = confirm;
}

void BLEDevice::setStaticKeysOpt(char* key){
  static ble_opt_t  m_static_pin_option;
  m_static_pin_option.gap_opt.passkey.p_passkey = (uint8_t*) key;
  int err_code =  sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &m_static_pin_option);
}
