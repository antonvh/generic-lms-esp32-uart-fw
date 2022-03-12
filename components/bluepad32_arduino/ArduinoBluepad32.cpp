// Copyright 2021 - 2021, Ricardo Quesada, http://retro.moe
// SPDX-License-Identifier: Apache-2.0 or LGPL-2.1-or-later

#include "ArduinoBluepad32.h"

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Arduino.h>
#include <inttypes.h>
#include <uni_bluetooth.h>
#include <uni_debug.h>
#include <uni_platform_arduino.h>
#include <uni_version.h>

Bluepad32::Bluepad32() : _prevConnectedGamepads(0), _gamepads(), _onConnect(), _onDisconnect() {}

const char* Bluepad32::firmwareVersion() const {
    return "Bluepad32 for Arduino v" UNI_VERSION;
}

void Bluepad32::update() {
    int connectedGamepads = 0;
    for (int i = 0; i < CONFIG_BLUEPAD32_MAX_DEVICES; i++) {
        if (arduino_get_gamepad_data(i, &_gamepads[i]._state) == -1)
            continue;
        if (_gamepads[i]._state.idx != -1)
            connectedGamepads |= (1 << i);
    }

    // No changes in connected gamepads. No need to call onConnected or onDisconnected.
    if (connectedGamepads == _prevConnectedGamepads)
        return;

    logi("connected in total: %d\n", connectedGamepads);

    // Compare bit by bit, and find which one got connected and which one disconnected.
    for (int i = 0; i < CONFIG_BLUEPAD32_MAX_DEVICES; i++) {
        int bit = (1 << i);
        int current = connectedGamepads & bit;
        int prev = _prevConnectedGamepads & bit;

        // No changes in this gamepad, skip
        if (current == prev)
            continue;

        if (current) {
            _gamepads[i]._connected = true;
            _onConnect(&_gamepads[i]);
            logi("gamepad connected: %d\n", i);
        } else {
            _onDisconnect(&_gamepads[i]);
            _gamepads[i]._connected = false;
            logi("gamepad disconnected: %d\n", i);
        }
    }

    _prevConnectedGamepads = connectedGamepads;
}

void Bluepad32::forgetBluetoothKeys() {
    uni_bluetooth_del_keys_safe();
}

void Bluepad32::setup(const GamepadCallback& onConnect, const GamepadCallback& onDisconnect) {
    _onConnect = onConnect;
    _onDisconnect = onDisconnect;
}

Bluepad32 BP32;
