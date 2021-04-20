/*
  Arduino-MAX30100 oximetry / heart rate integrated sensor library
  Copyright (C) 2016  OXullo Intersecans <x@brainrapers.org>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// This example must be used in conjunction with the Processing sketch located
// in extras/rolling_graph
#include <M5Core2.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#define REPORTING_PERIOD_MS     100
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SPO2_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"


BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLECharacteristic* spO2Characteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;
// PulseOximeter is the higher level interface to the sensor
// it offers:
//  * beat detection reporting
//  * heart rate calculation
//  * SpO2 (oxidation level) calculation
PulseOximeter pox;

uint32_t tsLastReport = 0;
uint32_t pwrLastReport = 0;
uint32_t pulseValueLastReport = 0;

int spO2 = 0; 
int heartRate = 0;
uint8_t redLedCurrent = 0;
// Callback (registered below) fired when a pulse is detected
TaskHandle_t Task1;

void onBeatDetected()
{
  M5.Lcd.setCursor(0, 200);
  M5.Lcd.print("@");
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup()
{
  M5.begin();
  Wire.begin();

  M5.Touch.addHandler(eventDisplay);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(3);

  BLEDevice::init("HeartMonitor");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

   spO2Characteristic = pService->createCharacteristic(
                      SPO2_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Heart");
  spO2Characteristic->addDescriptor(new BLE2902());
  spO2Characteristic->setValue("SpO2");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  // Initialize the PulseOximeter instance and register a beat-detected callback
  // The parameter passed to the begin() method changes the samples flow that
  // the library spews to the serial.
  // Options:
  //  * PULSEOXIMETER_DEBUGGINGMODE_PULSEDETECT : filtered samples and beat detection threshold
  //  * PULSEOXIMETER_DEBUGGINGMODE_RAW_VALUES : sampled values coming from the sensor, with no processing
  //  * PULSEOXIMETER_DEBUGGINGMODE_AC_VALUES : sampled values after the DC removal filter

  // Initialize the PulseOximeter instance
  // Failures are generally due to an improper I2C wiring, missing power supply
  // or wrong target chip
  if (!pox.begin(PULSEOXIMETER_DEBUGGINGMODE_PULSEDETECT)) {
    Serial.println("ERROR: Failed to initialize pulse oximeter");
    for (;;);
  }

  xTaskCreatePinnedToCore(
    BLETask, /* Function to implement the task */
    "BLE", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &Task1,  /* Task handle. */
    0); /* Core where the task should run */

  pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop()
{
  M5.update();
  pox.update(); 
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    spO2 = pox.getSpO2();
    heartRate = int(pox.getHeartRate());
    tsLastReport = millis();
  }
  if (millis() - pwrLastReport > 1000) {
    M5.Lcd.setCursor(0, 80);
    M5.Lcd.printf("%3d  ", heartRate);
    M5.Lcd.setCursor(0, 110);
    M5.Lcd.printf("%3d  ", spO2);
     
    float batVoltage = M5.Axp.GetBatVoltage();
    float batPercentage = ( batVoltage < 3.2 ) ? 0 : (( batVoltage - 3.2 ) * 100) + 1;
    M5.Lcd.setCursor(0, 20);
    M5.Lcd.print(batVoltage);
    M5.Lcd.setCursor(0, 50);
    M5.Lcd.printf("%.0f %", batPercentage);
    pwrLastReport = millis();
  }


     
}

void BLETask( void * pvParameters ) {
  while (true) {
    if (deviceConnected) {
      pCharacteristic->setValue(heartRate);
      pCharacteristic->notify();
      spO2Characteristic->setValue(spO2);
      spO2Characteristic->notify();
      delay(1000);
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
      delay(500);
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
    }
  }

}

void eventDisplay(TouchEvent& e) {
  //if (e.type != TE_TOUCH && e.type != TE_TAP && e.type != TE_DBLTAP) {
  //  Serial.printf("--> (%3d, %3d)  %5d ms", e.to.x, e.to.y, e.duration);
  //}
  if (M5.BtnA.wasPressed()) {

    M5.Lcd.setBrightness(0);
    M5.Axp.SetDCDC3(false);
    pox.shutdown();
  }
  if (M5.BtnB.wasPressed()) {
    M5.Axp.SetDCDC3(true);
    pox.resume();
  }
  if (M5.BtnC.wasPressed()) {
    M5.Axp.SetDCDC3(false);
  }
}
