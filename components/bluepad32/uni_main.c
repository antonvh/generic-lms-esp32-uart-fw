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

#include "uni_main.h"

#include "uni_bt_setup.h"
#include "uni_config.h"
#include "uni_debug.h"
#include "uni_hid_device.h"
#include "uni_platform.h"
#include "uni_version.h"

// Main entry point, runs forever
int uni_main(int argc, const char** argv) {
    logi("Bluepad32 (C) 2016-2022 Ricardo Quesada and contributors.\n");
    logi("Version: v" UNI_VERSION "\n");

    // Honoring with BT copyright
    logi("BTStack: Copyright (C) 2017 BlueKitchen GmbH.\n");

    uni_platform_init(argc, argv);
    uni_hid_device_init();

    // Continue with bluetooth setup.
    uni_bt_setup();

    // Does not return.
    btstack_run_loop_execute();

    return 0;
}
