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

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

// watchdog structs
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#include "driver/i2s.h"


#include <Arduino.h>
#include <Wire.h>
#include <Bluepad32.h>
//#include <UartRemote.h>
#include <LPF2.h>
#include <Adafruit_NeoPixel.h>
#include <arduinoFFT.h>
#include <ESP32Servo.h>


static GamepadPtr myGamepad;


#define LED_PIN 12
#define LED_COUNT 64

#define RXD2 18
#define TXD2 19

EV3UARTEmulation sensor(RXD2, TXD2, 62, 115200);

// use pointer allows to dynamically change nrumber of leds or pin
// change strip.begin() to strip->begin(), etc.
// delete object before initiating a new one
Adafruit_NeoPixel* strip = new Adafruit_NeoPixel(LED_COUNT,LED_PIN); //, NEO_GRB + NEO_KHZ800);

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

int servo1Pin = 21;
int servo2Pin = 22;
int servo3Pin = 23;
int servo4Pin = 25;

int minUs = 1000;
int maxUs = 2000;

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
    // In this example we only use one gamepad at the same time.
    myGamepad = gp;
    /*
    args=uartremote.call("connected","B",0);
    if (args.error==0) { // no error
            printf("received ack\n");
        } else { printf("error from call connected\n");}
    */
    Serial.println("CALLBACK: Gamepad is connected!");
}

void onDisconnectedGamepad(GamepadPtr gp) {
    Serial.println("CALLBACK: Gamepad is disconnected!");
    /*
    args=uartremote.call("disconnected","B",0);
    if (args.error==0) { // no error
            printf("received ack\n");
        } else { printf("error from call isconnected\n");}
    */
    myGamepad = nullptr;
}
uint8_t old_led=0,rumble_force,rumble_duration;


void servo_neo_callback(byte buf[],byte s) {
byte nr_short=int(s/2);
  short vals[nr_short];
  // Serial.printf("size %d, nr short %d\n",s,nr_short);
  for (int i=0; i<nr_short; i++) {
    vals[i]=buf[i*2]+buf[i*2+1]*128;
    // Serial.printf("vals[%d]=%d\n",i,vals[i]);
  }

  // Serial.printf("servo %d %d %d %d\n",vals[0],vals[1],vals[2],vals[3]);
  servo1.write(vals[0]);
  servo2.write(vals[1]);
  servo3.write(vals[2]);
  servo4.write(vals[3]);
 
  if (buf[8]==65) { // write led
     strip->show();
  } else if (buf[8]==66) { // init led
     delete strip;
     strip=new Adafruit_NeoPixel(buf[9],buf[10]); //nr_leds, pin
  } else if (buf[8]==67) { // clear all leds led
      for (int i=0; i<strip->numPixels(); i++ ) {
        strip->setPixelColor(i,0,0,0);
      }
  } else {
      strip->setPixelColor(buf[8],buf[9],buf[10],buf[11]);
  }

}

void neopixel_callback(byte buf[],byte s) {
byte nr_short=int(s/2);
  short vals[nr_short];
  // Serial.println();
  for (int i=0; i<nr_short; i++) {
    vals[i]=buf[i*2]+buf[i*2+1]*128;
    // Serial.printf("vals[%d]=%d\n",i,vals[i]);
  }

  if (vals[0]==65) { // write led
     strip->show();
  } else if (vals[0]==66) { // init led
      delete strip;
     strip=new Adafruit_NeoPixel(vals[1],vals[2]); //nr_leds, pin
     
  }  else {
      strip->setPixelColor(vals[0],vals[1],vals[2],vals[3]);
  }
}



// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    sensor.create_mode("GAMEPAD", true, DATA16, 6, 5, 0,0.0f,512.0f,0.0f,1024.0f,0.0f,100.0f,"XYBD",0,ABSOLUTE); //map in and map out unit = "XYBD" = x, y, buttons, d-pad
    sensor.create_mode("NEOPIXEL", true, DATA16, 4, 5, 0,0.0f,500.0f,0.0f,100.0f,0.0f,1024.0f,"NRGB", 0,ABSOLUTE); //map in and map out, units='NRGB'
    sensor.create_mode("GAMEPAD2", true, DATA16, 4, 5, 0,0.0f,512.0f,0.0f,1024.0f,0.0f,100.0f,"XYBD",0,ABSOLUTE); //map in and map out unit = "XYBD" = x, y, buttons, d-pad
  
    sensor.get_mode(0)->setCallback(servo_neo_callback);  // attach call back function to mode 0
    sensor.get_mode(1)->setCallback(neopixel_callback);  // attach neopixel call back function to mode 1
    Wire.begin(5,4); // sda=pin(5), scl=Pin(4)
    strip->begin();
    strip->show(); // Initialize all pixels to 'off'


    String fv = BP32.firmwareVersion();
    Serial.print("Firmware: ");
    Serial.println(fv);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    sensor.reset();
    delay(200);


// servo's
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	Serial.begin(115200);
	servo1.setPeriodHertz(50);      // Standard 50hz servo
	servo2.setPeriodHertz(50);      // Standard 50hz servo
	servo3.setPeriodHertz(50);      // Standard 50hz servo
	servo4.setPeriodHertz(50);      // Standard 50hz servo
  servo1.attach(servo1Pin, minUs, maxUs);
	servo2.attach(servo2Pin, minUs, maxUs);
	servo3.attach(servo3Pin, minUs, maxUs);
	servo4.attach(servo4Pin, minUs, maxUs);


}

int refresh_BP32=0;
// Arduino loop function. Runs in CPU 1


unsigned long last_reading = 0;

void loop() {
    // This call fetches all the gamepad info from the NINA (ESP32) module.
    // Just call this function in your main loop.
    // The gamepads pointer (the ones received in the callbacks) gets updated
    // automatically.
    refresh_BP32++;
    if (refresh_BP32 == 10) {
        BP32.update();
        refresh_BP32=0;
    }


  sensor.heart_beat();
    if (millis() - last_reading > 20) {
      int mode=sensor.get_current_mode();
      if (mode==0) {
        short bb[8];

        if (myGamepad && myGamepad->isConnected()) {
          //myGamepad->buttons(),myGamepad->dpad(),
          bb[0]=myGamepad->axisX();
          bb[1]=myGamepad->axisY();
          bb[2]=myGamepad->axisRX();
          bb[3]=myGamepad->axisRY();
          bb[4]=myGamepad->buttons();
          bb[5]=myGamepad->dpad();
  //         myGamepad->axisRX(),myGamepad->axisRY());
        }
        sensor.send_data16(bb,8);
      } 
      else if (mode==2) {
        short bb[4];

        if (myGamepad && myGamepad->isConnected()) {
          //myGamepad->buttons(),myGamepad->dpad(),
          bb[0]=myGamepad->axisRX();
          bb[1]=myGamepad->axisRY();
          bb[2]=myGamepad->buttons();
          bb[3]=myGamepad->dpad();
        }

        sensor.send_data16(bb,4); 
      } 
      else   Serial.printf("mode=%d\n",mode);

      last_reading = millis();
    }


   


    // https://github.com/espressif/arduino-esp32/issues/595
    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed=1;
    TIMERG0.wdt_wprotect=0;


   
    
}
