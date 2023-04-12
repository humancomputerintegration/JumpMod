/*
    JumpMod_BLE -- firmaware for JumpMod haptic backpack
    built for Seeeduino XIAO nRF52840 Sense with Arduino
*/

#include "JumpController.h"
#include "LSM6DS3.h"
#include "Wire.h"
#include <ArduinoBLE.h>
#include <SimpleFOC.h> // https://docs.simplefoc.com/low_pass_filter

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define GET_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SEND_CHARACTERISTIC_UUID "8628FE7C-A4E9-4056-91BD-FD6AA7817E39"
#define TRIGGER_CHARACTERISTIC_UUID "1fee8942-0f64-11ed-861d-0242ac120002"
#define VOLTAGE_CHARACTERISTIC_UUID "9c5a211f-d400-490b-9465-52f7a417d19c"
#define POSITION_CHARACTERISITC_UUID "3deefe33-d8f5-45a8-a24c-8fec01486b03"

JumpController myJC;

//Create a instance of class LSM6DS3
LSM6DS3 xiaoIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

/* XIAO BLE Sense Params */
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

unsigned long prevdt = 0;
bool isUpsideDown = false;

/* Communication Params */
boolean newData = false;
const byte numChars = 32;//11;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing


BLEService jumpService(SERVICE_UUID); // Bluetooth® Low Energy LED Service
// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLECharacteristic jumpCharacteristic(GET_CHARACTERISTIC_UUID, BLERead | BLEWrite, "           "); // Sets max size to 11 with a 11 length string
BLECharacteristic sendCharacteristic(SEND_CHARACTERISTIC_UUID, BLERead | BLEWrite | BLENotify, " "); // Sets max size to 1 with a 1 length string
BLEByteCharacteristic triggerCharacteristic(TRIGGER_CHARACTERISTIC_UUID, BLERead | BLEWrite | BLENotify); // Should only contain either 0 or 1
BLECharacteristic voltageCharacteristic(VOLTAGE_CHARACTERISTIC_UUID, BLERead | BLENotify, "     "); // Battery voltage, 5 characters, ie "22.50"
BLECharacteristic positionCharacteristic(POSITION_CHARACTERISITC_UUID, BLERead | BLENotify, "   "); // Encoder position, 3 ~char

bool isConnected = false;

// Blinks the LED every 100 interations to show the code is running
int blinkCounter = 0;

int oscEvalTimer = 0;
bool alreadyLogged = false;
int logVal = 0;

LowPassFilter filter = LowPassFilter(0.01);

unsigned long lastVoltageRead;
int prevSentPosition = -100; // Throwaway value so it sends on startup

void setup() {
  xiaoIMU.begin();

  myJC.onlyPrediction = false;
  myJC.setup();

  // Setup BLE
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Jumper"); 
  BLE.setAdvertisedService(jumpService);

  // add the characteristic to the service
  jumpService.addCharacteristic(jumpCharacteristic);
  jumpService.addCharacteristic(sendCharacteristic);
  jumpService.addCharacteristic(triggerCharacteristic);
  jumpService.addCharacteristic(voltageCharacteristic);
  jumpService.addCharacteristic(positionCharacteristic);

  // add service
  BLE.addService(jumpService);

  // start advertising
  BLE.advertise();

  //Setup LED
  pinMode(LED_BUILTIN, OUTPUT);
}

void readCommand() {
  const unsigned char *value = jumpCharacteristic.value();
  strcpy(tempChars, (const char*) value);
  if (tempChars[0] == 'l') {
    digitalWrite(LED_BUILTIN, LOW);
    myJC.lightCommandOn = true;
  }
  myJC.parseData(tempChars);
  if (myJC.debug && !myJC.debugPlot) {
    Serial.print("Message received: ");
    Serial.println((const char*) value);
  }
}

void readTrigger() {
  byte incoming = 0;
  triggerCharacteristic.readValue(incoming);
  if (incoming == '1') {
    triggerCharacteristic.writeValue('0');
    myJC.trigger();
    myJC.lightCommandOn;
  }
  else if (incoming == '0') {
    myJC.resetQueue();
  }
}

void loop() {
  // get position from encoder
  myJC.getPosition();

  if (!myJC.predictJump){
    myJC.driveController();

    myJC.currentTime = millis();
    if (myJC.driveStatus == "initial") {
      myJC.drivePosition();
    }
    myJC.previousTime = myJC.currentTime; 
  }
 
  // /* Handle commands */
  if (myJC.drive) {
    recvWithStartEndMarkers();
    if (newData == true) {
      strcpy(tempChars, receivedChars);
      // this temporary copy is necessary to protect the original data
      //   because strtok() used in parseData() replaces the commas with \0
      if (tempChars[0] == 'l') {
        digitalWrite(LED_BUILTIN, LOW);
        myJC.lightCommandOn = true;
      }
      else if (tempChars[0] == 'o') {
        oscEvalTimer = 200;
        Serial.println("Sending \"0\" to OSC/command");
        digitalWrite(LED_BUILTIN, LOW);
        myJC.lightCommandOn = true;
      }
      myJC.parseData(tempChars);
      newData = false;
    }
  }

  if(myJC.queuedMovements.empty()) {
    if(myJC.vibrationMode == 1) {
      myJC.addQueue(myJC.vibrateDC, 30);
      myJC.addQueue(myJC.vibrateDC, 70);
    }
    else if(myJC.vibrationMode == 4) {
      myJC.addQueue(myJC.vibrateSoftDC, myJC.vibrationOffset);
      myJC.addQueue(myJC.vibrateSoftDC, myJC.vibrationPosition);
      Serial.print("Queuing: " + String(myJC.vibrateSoftDC) + " - " + String(myJC.vibrationOffset));
      Serial.println(" and: " + String(myJC.vibrationPosition) + " current: " + String(myJC.encoderPosition));
    }
    else {
      myJC.vibrationMode = 0;
    }
  }
  // vibrate regular
  if (myJC.vibrationMode != 0 && (myJC.driveStatus == "brake" || myJC.driveStatus == "none")
      && !myJC.queuedMovements.empty() ) {
    
    myJC.trigger();
  }

  

  /* BLE */
  bool connected = BLE.connected();
  if (connected && !isConnected) {
    isConnected = true;
    if (myJC.debug && !myJC.debugPlot) {
      Serial.println("Bluetooth device connected!");
    }
  }
  else if (!connected && isConnected) {
    isConnected = false;
    BLE.advertise();
    if (myJC.debug && !myJC.debugPlot) {
      Serial.println("Bluetooth device disconnected! Restarting advertising...");
    }
  }

  /* Read bluetooth messages */
  if (jumpCharacteristic.written()) {
    blinkCounter = 0;
    digitalWrite(LED_BUILTIN, LOW);
    readCommand();
  }
  /* Blink the LED */
  controlLED();

  if (triggerCharacteristic.written()) {
    readTrigger();
  }

  /* Call jump prediction functions when enabled */
  if (myJC.predictJump) {
    predictJump();
  }

  if (millis() - lastVoltageRead > 2000 && myJC.UART.getVescValues()) {
    lastVoltageRead = millis();
    myJC.batteryVoltage = myJC.UART.data.inpVoltage;
    voltageCharacteristic.writeValue(String(myJC.batteryVoltage).c_str());
  }

  if (prevSentPosition != myJC.encoderPosition ) {
    positionCharacteristic.setValue(String((myJC.encoderPosition * 100) / myJC.encoderTopPosition).c_str());
    prevSentPosition = myJC.encoderPosition;
    if(myJC.debug && !myJC.debugPlot && myJC.debugBLE) {
      Serial.println("Sent via BLE: Position = " + String((myJC.encoderPosition * 100) / myJC.encoderTopPosition));
    }
  }
}

void controlLED() {
  if (myJC.lightCommandOn) {
    blinkCounter = 0;
    myJC.lightCommandOn = false;
  }
  if (blinkCounter == 1000) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  if (blinkCounter == 0) {
    digitalWrite(LED_BUILTIN, LOW);
  }
  blinkCounter = (blinkCounter + 1) % 2000;
  if (blinkCounter >= 1001 && myJC.useBlinkCommands) {
    blinkCounter = 1001;
  }
}

void predictJump() {
  /* Jump prediction setup */
  // Read acceleration values
  accX = xiaoIMU.readFloatAccelX();
  accY = xiaoIMU.readFloatAccelY();
  accZ = xiaoIMU.readFloatAccelZ();
  accX *= 9.8 * 9.8 / 9.93;
  accY *= 9.8 * 9.8 / 9.93;
  accZ *= 9.8 * 9.8 / 9.93;

  // Apply a low-pass filter on the accelation data to reduce noise
  float YZ = filter(sqrt(sq(accY) + sq(accZ))) - 9.8;
  if (isUpsideDown) {
    YZ *= -1;
  }
  myJC.loop(YZ);

  // Print plotting info
  if (myJC.debugPlot && !alreadyLogged && logVal == 0) {
    unsigned long dt = millis();
    unsigned long deltaTime = dt - prevdt;
    prevdt = dt;
    Serial.print(
      "YZ: " + String(YZ) +
      ", Velocity: " + String(myJC.velocity) + ", Displacement: " + String(myJC.displacement * 5) +
      ", Jump Status: " + String((float)(myJC.jumpStatus - 1) / (float) 4));
    if (myJC.jumpStatus == 9 && !alreadyLogged) {
      alreadyLogged = true;
    }
    if (myJC.maxAccel != -9999) {
      Serial.print(", Max accel: " + String(myJC.maxAccel));
    }
    Serial.println(myJC.printDT ? (" Delta Time: " + String(deltaTime)) : "");
  }
  logVal = (logVal + 1) % 5;

  int sendCharValue = atoi((const char *) sendCharacteristic.value());

  if (oscEvalTimer > 0) {
    if (isConnected && oscEvalTimer == 200) {
      sendCharacteristic.writeValue("0");
    }
    oscEvalTimer--;
    return;
  }
  // Ensure the user is not tilted on the Z-axis
  else if (abs(myJC.displacement) > 4) {
    if (myJC.debug && !myJC.debugPlot) {
      Serial.println("Error: You are not rotated correctly!");
    }
    if (isConnected && sendCharValue > 0) {
      sendCharacteristic.writeValue("0");
    }
  }
  else if (isConnected && sendCharValue <= 0) {
    sendCharacteristic.writeValue((char *) String((int) myJC.jumpStatus).c_str());
    if (myJC.debug && !myJC.debugPlot) {
      Serial.println("Sent via BLE: " + String(myJC.jumpStatus));
    }
  }
  sendCharValue = atoi((const char *) sendCharacteristic.value());
  /* Update bluetooth jump status */
  if (isConnected && sendCharValue != (int) myJC.jumpStatus && sendCharValue > 0) {
    sendCharacteristic.writeValue((char *) String((int) myJC.jumpStatus).c_str());
    if (myJC.debug && !myJC.debugPlot) {
      Serial.println("Sent via BLE: " + String(myJC.jumpStatus));
    }
  }
}

// Read data from the Serial monitor
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}
