/****************************************************************************
http://retro.moe/unijoysticle2

Copyright 2019 Ricardo Quesada

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

// For more info about Android mappings see:
// https://developer.android.com/training/game-controllers/controller-input

#include "uni_hid_parser_android.h"

#include "hid_usage.h"
#include "uni_debug.h"
#include "uni_hid_device.h"
#include "uni_hid_parser.h"

void uni_hid_parser_android_init_report(uni_hid_device_t* d) {
    // Reset old state. Each report contains a full-state.
    uni_gamepad_t* gp = &d->gamepad;
    memset(gp, 0, sizeof(*gp));

    // It is safe to set the reported states just once, here:
    gp->updated_states = GAMEPAD_STATE_AXIS_X | GAMEPAD_STATE_AXIS_Y | GAMEPAD_STATE_AXIS_RX | GAMEPAD_STATE_AXIS_RY;
    gp->updated_states |= GAMEPAD_STATE_BRAKE | GAMEPAD_STATE_THROTTLE;
    gp->updated_states |= GAMEPAD_STATE_DPAD;
    gp->updated_states |=
        GAMEPAD_STATE_BUTTON_X | GAMEPAD_STATE_BUTTON_Y | GAMEPAD_STATE_BUTTON_A | GAMEPAD_STATE_BUTTON_B;
    gp->updated_states |= GAMEPAD_STATE_BUTTON_TRIGGER_L | GAMEPAD_STATE_BUTTON_TRIGGER_R |
                          GAMEPAD_STATE_BUTTON_SHOULDER_L | GAMEPAD_STATE_BUTTON_SHOULDER_R;
    gp->updated_states |= GAMEPAD_STATE_BUTTON_THUMB_L | GAMEPAD_STATE_BUTTON_THUMB_R;
    gp->updated_states |=
        GAMEPAD_STATE_MISC_BUTTON_BACK | GAMEPAD_STATE_MISC_BUTTON_HOME | GAMEPAD_STATE_MISC_BUTTON_SYSTEM;
}

void uni_hid_parser_android_parse_usage(uni_hid_device_t* d,
                                        hid_globals_t* globals,
                                        uint16_t usage_page,
                                        uint16_t usage,
                                        int32_t value) {
    // print_parser_globals(globals);
    uint8_t hat;
    uni_gamepad_t* gp = &d->gamepad;
    switch (usage_page) {
        case HID_USAGE_PAGE_GENERIC_DESKTOP:
            switch (usage) {
                case HID_USAGE_AXIS_X:
                    gp->axis_x = uni_hid_parser_process_axis(globals, value);
                    break;
                case HID_USAGE_AXIS_Y:
                    gp->axis_y = uni_hid_parser_process_axis(globals, value);
                    break;
                case HID_USAGE_AXIS_Z:
                    gp->axis_rx = uni_hid_parser_process_axis(globals, value);
                    break;
                case HID_USAGE_AXIS_RZ:
                    gp->axis_ry = uni_hid_parser_process_axis(globals, value);
                    break;
                case HID_USAGE_HAT:
                    hat = uni_hid_parser_process_hat(globals, value);
                    gp->dpad = uni_hid_parser_hat_to_dpad(hat);
                    break;
                case HID_USAGE_DPAD_UP:
                case HID_USAGE_DPAD_DOWN:
                case HID_USAGE_DPAD_RIGHT:
                case HID_USAGE_DPAD_LEFT:
                    uni_hid_parser_process_dpad(usage, value, &gp->dpad);
                    break;
                default:
                    // Only report unsupported values if they are 1.
                    if (value)
                        logi(
                            "Android: Unsupported page: 0x%04x, usage: 0x%04x, "
                            "value=0x%x\n",
                            usage_page, usage, value);
                    break;
            }
            break;
        case HID_USAGE_PAGE_SIMULATION_CONTROLS:
            switch (usage) {
                case HID_USAGE_ACCELERATOR:
                    gp->throttle = uni_hid_parser_process_pedal(globals, value);
                    break;
                case HID_USAGE_BRAKE:
                    gp->brake = uni_hid_parser_process_pedal(globals, value);
                    break;
                default:
                    // Only report unsupported values if they are 1.
                    if (value)
                        logi(
                            "Android: Unsupported page: 0x%04x, usage: 0x%04x, "
                            "value=0x%x\n",
                            usage_page, usage, value);
                    break;
            };
            break;
        case HID_USAGE_PAGE_GENERIC_DEVICE_CONTROLS:
            switch (usage) {
                case HID_USAGE_BATTERY_STRENGHT:
                    gp->battery = value;
                    break;
                default:
                    if (value)
                        logi(
                            "Android: Unsupported page: 0x%04x, usage: 0x%04x, "
                            "value=0x%x\n",
                            usage_page, usage, value);
                    break;
            }
            break;
        case HID_USAGE_PAGE_BUTTON: {
            switch (usage) {
                case 0x01:  // Button A
                    if (value)
                        gp->buttons |= BUTTON_A;
                    break;
                case 0x02:  // Button B
                    if (value)
                        gp->buttons |= BUTTON_B;
                    break;
                case 0x03:  // non-existant button C?
                    // unmapped
                    break;
                case 0x04:  // Button X
                    if (value)
                        gp->buttons |= BUTTON_X;
                    break;
                case 0x05:  // Button Y
                    if (value)
                        gp->buttons |= BUTTON_Y;
                    break;
                case 0x06:  // non-existant button Z?
                    // unmapped
                    break;
                case 0x07:
                    if (value)
                        gp->buttons |= BUTTON_SHOULDER_L;
                    break;
                case 0x08:
                    if (value)
                        gp->buttons |= BUTTON_SHOULDER_R;
                    break;
                case 0x09:
                    // Available on some Android gamepads like SteelSeries Stratus Duo.
                    if (value)
                        gp->buttons |= BUTTON_TRIGGER_L;
                    break;
                case 0x0a:
                    // Available on some Android gamepads like SteelSeries Stratus Duo.
                    if (value)
                        gp->buttons |= BUTTON_TRIGGER_R;
                    break;
                case 0x0b:
                    // Available on some Android gamepads like SteelSeries Stratus Duo.
                    if (value)
                        gp->misc_buttons |= MISC_BUTTON_BACK;
                    break;
                case 0x0c:
                    // Available on some Android gamepads like SteelSeries Stratus Duo.
                    if (value)
                        gp->misc_buttons |= MISC_BUTTON_HOME;
                    break;
                case 0x0d:
                    if (value)
                        gp->misc_buttons |= MISC_BUTTON_SYSTEM;
                    break;
                case 0x0e:
                    if (value)
                        gp->buttons |= BUTTON_THUMB_L;
                    break;
                case 0x0f:
                    if (value)
                        gp->buttons |= BUTTON_THUMB_R;
                    break;
                default:
                    // Only report unsupported values if they are 1.
                    if (value)
                        logi(
                            "Android: Unsupported page: 0x%04x, usage: 0x%04x, "
                            "value=0x%x\n",
                            usage_page, usage, value);
                    break;
            }
            break;
        }
        case HID_USAGE_PAGE_CONSUMER:
            switch (usage) {
                case HID_USAGE_AC_HOME:
                    // FIXME: Some devices, like SteelSeries Status Duo, use this value as
                    // BUTTON_SYSTEM. But others like Asus, use this one to report
                    // BUTTON_HOME. Instead of having a parser for Android / OUYA /
                    // 8Bitdo, we should have a HID parser and then mapping files for each
                    // VID / PID (similar to Android .kl files).
                    if (value)
                        gp->misc_buttons |= MISC_BUTTON_HOME;
                    break;
                case HID_USAGE_AC_BACK:
                    if (value)
                        gp->misc_buttons |= MISC_BUTTON_BACK;
                    break;
                default:
                    // Only report unsupported values if they are 1.
                    if (value)
                        logi(
                            "Android: Unsupported page: 0x%04x, usage: 0x%04x, "
                            "value=0x%x\n",
                            usage_page, usage, value);
                    break;
            }
            break;

        case 0xff01:
            // Ignore this report. Used by some Moga devices, where page 0xff01,
            // usage: 0x0001 is always 1.
            if (usage == 0x01)
                break;
            break;

        default:
            // Only report unsupported values if they are 1.
            if (value)
                logi("Android: Unsupported page: 0x%04x, usage: 0x%04x, value=0x%x\n", usage_page, usage, value);
            break;
    }
}

void uni_hid_parser_android_set_player_leds(uni_hid_device_t* d, uint8_t leds) {
#if 0
  static uint8_t report_id = 0;
  logi("using report id = 0x%02x\n", report_id);
  uint8_t report[] = {0xa2, 0, 0x00 /* LED */};
  report[2] = 0x02; /* d->joystick_port; */
  report[1] = report_id++;
  uni_hid_device_queue_report(d, report, sizeof(report));
  report[0] = 0x52;
  uni_hid_device_queue_report(d, report, sizeof(report));
#else
    UNUSED(d);
    UNUSED(leds);
#endif
}
