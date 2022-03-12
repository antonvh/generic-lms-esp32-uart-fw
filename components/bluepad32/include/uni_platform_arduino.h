/****************************************************************************
http://retro.moe/unijoysticle2

Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

#ifndef UNI_PLATFORM_ARDUINO_H
#define UNI_PLATFORM_ARDUINO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "uni_gamepad.h"
#include "uni_platform.h"

enum {
    UNI_ARDUINO_OK = 0,
    UNI_ARDUINO_ERROR = -1,
};

enum {
    UNI_ARDUINO_GAMEPAD_INVALID = -1,
};

typedef struct {
    // Indicates which gamepad it is. Goes from 0 to 3.
    int8_t idx;

    // Type of gamepad: PS4, PS3, Xbox, etc..?
    uint8_t type;

    // The gamepad data: buttons, axis, etc.
    uni_gamepad_t data;
} arduino_gamepad_t;

struct uni_platform* uni_platform_arduino_create(void);

int arduino_get_gamepad_data(int idx, arduino_gamepad_t* out_gp);
int arduino_set_player_leds(int idx, uint8_t leds);
int arduino_set_lightbar_color(int idx, uint8_t r, uint8_t g, uint8_t b);
int arduino_set_rumble(int idx, uint8_t force, uint8_t duration);
int arduino_forget_bluetooth_keys(void);

#ifdef __cplusplus
}
#endif

#endif  // UNI_PLATFORM_ARDUINO_H
