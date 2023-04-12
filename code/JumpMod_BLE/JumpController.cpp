/*
    JumpController.h -- firmaware for JumpMod haptic backpack
    built for Seeeduino XIAO nRF52840 Sense with Arduino
*/

#include "JumpController.h"
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VescUart.h>
#include <math.h>


#define SWITCH_BOT 0 // limit switch bottom
#define SWITCH_TOP 1 // limit switch top
#define ENCODER_A 2
#define ENCODER_B 3

#define readA digitalRead(ENCODER_A)  //bitRead(PIND,2) // faster than digitalRead() -- doesnt work
#define readB digitalRead(ENCODER_B)  //bitRead(PIND,3) // faster than digitalRead()

int count = 0; // used by interrupts to count encoder

void JumpController::setup() {
  /** Setup serial port for computer com **/
  Serial.begin(115200);
  Serial.println("JumpAR program begins...");

  if (!onlyPrediction) {
    /** Setup UART port for VESC (Serial1 on Atmega32u4) */
    Serial1.begin(115200);

    /** Define which ports to use as UART */
    UART.setSerialPort(&Serial1);

    Serial.println("UART initialization successful!");

    pinMode(SWITCH_BOT, INPUT);
    pinMode(SWITCH_TOP, INPUT);

    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), isrB, CHANGE);    
    
  }
}

void JumpController::loop(float YZ) { // used with prediction
  // Predict user's jump status every 10ms
  predictJumpStatus(YZ);

  if (!onlyPrediction) {
    // Perform powered jump if we are in powered jump mode and the user jumps
    if (jumpStatus == predStartDrive && powerJumpPhase == "none" && powerJump) { // condition to start drive
      startJump(powerJumpDutyCycle, powerJumpDriveDuration);
      powerJumpPhase = "up";
    }
    else if (jumpStatus == predStopDrive && powerJumpPhase == "up" && driveStatus == "initial" && powerJump) // condition to stop drive
    {
      driveStatus = "brake";
      if (debug && !debugPlot) {
        Serial.println("Apex reached, stopping drive early");
      }
    }
    else if (jumpStatus == 1 && powerJumpPhase == "up") { // condition to reset weight
      if (powerJumpDutyCycle < 0)
        startJump(.05, 4000);
      else if (powerJumpDutyCycle > 0)
        startJump(-.05, 4000);
      powerJumpPhase = "down";
    }
    else if (powerJumpPhase == "down" && driveStatus == "none")
    {
      powerJumpPhase = "none";
    }

    if (drive) {
      driveMotor();
    }
  }

  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    // pinging the ESC
    if (!onlyPrediction)
      if (driveStatus == "none")
        UART.setDuty(0);
  }

  // debug limit switches
  if (debugSW && !debugPlot) {
    Serial.print("SW_B = ");
    Serial.print(digitalRead(SWITCH_BOT));
    Serial.print(" -- ");
    Serial.print("SW_T = ");
    Serial.println(digitalRead(SWITCH_TOP));

  }
}

void JumpController::driveController(){ // used with without prediction
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    // pinging the ESC
    if (!onlyPrediction)
      if (driveStatus == "none")
        UART.setDuty(0);
  }

//  // brake if switch hit and driving
//  if ( (digitalRead(SWITCH_TOP) == 1 || digitalRead(SWITCH_BOT) == 1) 
//    && (driveStatus != "brake" || driveStatus != "none" ) ) {
//    driveStatus = "brake"; 
//    UART.setDuty(0);
//  }
  
  // debug limit switches
  if (debugSW && !debugPlot) {
    Serial.print("SW_B = ");
    Serial.print(digitalRead(SWITCH_BOT));
    Serial.print(" -- ");
    Serial.print("SW_T = ");
    Serial.println(digitalRead(SWITCH_TOP));
  }
}

void JumpController::isrA() {
  if(readB != readA) {
    count ++;
  } else {
    count --;
  }
}
void JumpController::isrB() {
  if (readA == readB) {
    count ++;
  } else {
    count --;
  }
}

void JumpController::getPosition() {
  noInterrupts();
  encoderPosition = count;
  interrupts();
  
  if (debug && debugPosition) {
    if (encoderPrevPosition != encoderPosition) {
      Serial.println("Position: " + String(encoderPosition) + " - " + String((encoderPosition*100)/encoderTopPosition));    
    }
  }
  encoderPrevPosition = encoderPosition;
}

void JumpController::encoderCalibration() {
  // Make sure the weight is at the bottome
  while (digitalRead(SWITCH_BOT) != 1)  UART.setDuty(0.05);
  count = 0;
  
  while (digitalRead(SWITCH_TOP) != 1)  {
    UART.setDuty(-0.05);
    getPosition();
  }
  encoderTopPosition = count;
  while (digitalRead(SWITCH_BOT) != 1)  UART.setDuty(0.1);
  calibRoutine = false;  
  Serial.println("Calibration done, new value: " + String(encoderTopPosition));
}

void JumpController::drivePosition() {
  if (encoderPosition > targetPosition && dutyCycle > 0 && driveStatus != "brake") { // driving downwards, positive dutyCycle
  // limit switch when driving down
    if (digitalRead(SWITCH_BOT) == 1) {
        driveStatus = "brake"; 
        count = 0;
    }
  }
  else if (encoderPosition < targetPosition && dutyCycle < 0 && driveStatus != "brake") { // driving upwards, positive dutyCycle
    // limit switch when driving up
    if (digitalRead(SWITCH_TOP) == 1) {        
        driveStatus = "brake"; 
        if (!calibRoutine) count = encoderTopPosition;
     }
  }
  else {
    driveStatus = "brake";
  }

  dutyCycle_PID = - computePID(encoderPosition); // flip PID sign
  if (debug && !debugPlot && debugPID) {
    Serial.print("PID output " + String(dutyCycle_PID));
    Serial.print(" kp " + String(kp * errorToTarget));
    Serial.print(" ki " + String(ki * cumError));
    Serial.print(" kd " + String(kd * rateError));
  }
  // clamping duty cycle or is vibrating
  if (abs(dutyCycle_PID) > abs(dutyCycle) || vibrationMode > 0) {
    dutyCycle_PID = dutyCycle;
  }
  
  if (debug && !debugPlot && debugPID) Serial.println(" PID output clamped " + String(dutyCycle_PID));

  // send message every 900ms so it doesn't fill up ESC buffer
  if (driveStatus != "brake") {
    if (millis() - timePrev >= 900 || firstDrive == 0 || dutyCycle_PID != prevDutyCycle_PID) { 
      if (drive) UART.setDuty(dutyCycle_PID);
      firstDrive = 1;
      timePrev = millis();
    }
  }
  else {
    driveStatus = "brake";
  }

  prevDutyCycle_PID = dutyCycle_PID;
  
  if (driveStatus == "brake") {
    firstDrive == 0;
    if (drive) {
      UART.setDuty(0);
      driveStatus = "none";
    }
  }
}

void JumpController::driveMotor() {
  if (driveStatus == "initial") {
    if (timeAtTarget > millis() - timeZero) {
      // limit switch when driving down
      if (dutyCycle > 0) {
        if (digitalRead(SWITCH_BOT) == 1) {
            driveStatus = "brake"; 
            count = 0;
//            encoderPosition = count;
            Serial.println("Reset to 0 - " + String(encoderPosition));
          }
      }
      // limit switch when driving up
      if (dutyCycle < 0) {
        if (digitalRead(SWITCH_TOP) == 1) {        
            driveStatus = "brake"; 
            Serial.println("Was" + String(encoderPosition));
            if (!calibRoutine) count = encoderTopPosition;
//            encoderPosition = count;
            Serial.println("Reset to 256 - " + String(encoderPosition));
          }
      }
      if (driveStatus != "brake") {
        if (millis() - timePrev >= 900 || firstDrive == 0) { // send message every 900ms so it doesn't fill up ESC buffer
          if (drive) UART.setDuty(dutyCycle);
          timePrev = millis();
          firstDrive = 1;
        }
      }
    }
    else {
      driveStatus = "brake";
    }
  }
  if (driveStatus == "brake") {
    if (drive) {
      UART.setDuty(0);
      driveStatus = "none";
    }
  }
}

void JumpController::printJumpStatus()
{
  const char *jumpStatusStrs[] = {"Stand", "Crouch", "Launch", "Liftoff", "Ascent", "Apex", "Descent", "Landing", "Rebound"};
  if (debug && !debugPlot) {
    Serial.print("Jump Status = ");
    //Serial.print(jumpStatusStrs[(int) jumpStatus - 1]);
    Serial.print((int) jumpStatus);
    Serial.println(" at " + String(millis()));
  }
  statusStart = millis();
}

void JumpController::predictJumpStatus(float YZ)
{
  // Calculate time since last check
  unsigned long curLoopTime = millis();
  deltaTime = curLoopTime - prevLoopTime;

  // Integrate acceleration to calculate velocity and displacement
  velocity += YZ * ((float) deltaTime) / 1000;
  displacement += velocity * ((float) deltaTime) / 1000;

  // Save the current time for next iteration
  prevLoopTime = curLoopTime;

  // Recallibrate when the user is standning still
  if (abs(YZ) <= 0.1) {
    sameCount = sameCount > sameCountMax ? sameCount : sameCount + 1;
  }
  else {
    sameCount = 0;
  }
  if (sameCount >= sameCountMax) {
    velocity = 0;
    displacement = 0;
    resetJumpStatus();
  }

  // Store velocity extrema values
  if (velocity > maxVelocity)
  {
    maxVelocity = velocity;
    timeSinceMax = -1;
  }
  timeSinceMax = timeSinceMax < extremaDelta ? timeSinceMax + 1 : timeSinceMax;
  if (velocity < minVelocity)
  {
    minVelocity = velocity;
    timeSinceMin = -1;
  }
  timeSinceMin = timeSinceMin < extremaDelta ? timeSinceMin + 1 : timeSinceMin;

  if (displacement < minDisplacement)
  {
    minDisplacement = displacement;
    timeSinceMinDisplacement = -1;
  }
  timeSinceMinDisplacement = timeSinceMinDisplacement < extremaDelta ? timeSinceMinDisplacement + 1 : timeSinceMinDisplacement;

  // Store velocity extrema values
  if (YZ > maxAccel)
  {
    maxAccel = YZ;
    timeSinceMaxAccel = -1;
  }
  timeSinceMaxAccel = timeSinceMaxAccel < extremaDelta ? timeSinceMaxAccel + 1 : timeSinceMaxAccel;

  // Determine Jump Status
  switch (jumpStatus) {
    case STAND:
      if (velocity < crouchBound && timeSinceMin >= extremaDelta)
      {
        jumpStatus = CROUCH;
        printJumpStatus();
      }
      break;
    case CROUCH:
      if (velocity > launchDetectionConstant)
      {
        jumpStatus = LAUNCH;
        belowAccelTime = 0;
        maxVelocity = -9999;
        printJumpStatus();
      }
      break;
    case LAUNCH:
      if (YZ < maxAccel * liftoffDetectionFactor && belowAccelTime >= maxBelowAccelTime)
      {
        jumpStatus = LIFTOFF;
        printJumpStatus();
      }
      else if (YZ < maxAccel * liftoffDetectionFactor) {
        belowAccelTime++;
      }
      else {
        belowAccelTime = 0;
      }
      break;
    case LIFTOFF:
      if (!maxVelocitySet && velocity < maxVelocity && timeSinceMax >= extremaDelta) {
        if (displacement < -0.07) {
          minVelocity = 9999;
          maxVelocity = -9999;
          jumpStatus = STAND;
          if (debug && !debugPlot) {
            Serial.println("Crouch detected!");
          }
          printJumpStatus();
        }
        else {
          maxVelocitySet = true;
          velocity += liftoffErrorConstant;
          velocity += maxVelocity * liftoffErrorFactor;
          maxVelocity *= liftoffMaxErrorFactor;
        }
      }
      if (maxVelocitySet && velocity < maxVelocity / 2)
      {
        jumpStatus = ASCENT;
        printJumpStatus();
      }
      break;
    case ASCENT:
      if (velocity < 0)
      {
        jumpStatus = APEX;
        minVelocity = 9999;
        printJumpStatus();
      }
      break;
    case APEX:
      if (velocity < -maxVelocity / 2)
      {
        jumpStatus = DESCENT;
        printJumpStatus();
      }
      break;
    case DESCENT:
      if (velocity <= -maxVelocity * landingDetectionFactor)
      {
        jumpStatus = LANDING;
        velocity += landingErrorConstant;
        minVelocity += landingErrorConstant;
        velocity += abs(minVelocity) * landingErrorFactor;
        printJumpStatus();
      }
      break;
    case LANDING:
      if (velocity >= recoverDetectionConstant)
      {
        jumpStatus = REBOUND;
        printJumpStatus();
      }
      break;
    case REBOUND:
      // Nothing, just wait to reset
      if (velocity == 0) {
        resetJumpStatus();
      }
      break;
  }

  // Reset if nothing changes for a while in case an error occurs
  if (jumpStatus != STAND && millis() - statusStart > resetTime)
  {
    jumpStatus = REBOUND;
    printJumpStatus();
    resetJumpStatus();
  }
}

void JumpController::startJump(float newDutyCycle, int newTimeInMS)
{
  dutyCycle = newDutyCycle;
  timeinms = newTimeInMS;

  // Init params for timer
  firstDrive = 0;
  timeZero = millis();
  timePrev = timeZero;
  timeAtTarget = timeinms;// + timeZero;
  driveStatus = "initial";
  if (debug && !debugPlot) {
    Serial.print("Duty cycle: ");
    Serial.print(dutyCycle);
    Serial.print(" timing: ");
    Serial.println(timeinms);
  }
}

void JumpController::resetJumpStatus() {
  minVelocity = 9999;
  maxVelocity = -9999;
  maxAccel = -9999;
  maxVelocitySet = false;
  if (jumpStatus != STAND) {
    jumpStatus = STAND;
    printJumpStatus();
  }
}


void JumpController::parseData(char* tempChars) {
  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ","); // arming the device: <p,0> = disarm, <p,1> = arm
  if (strtokIndx[0] == 'p')
  {
    strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation
    if (atoi(strtokIndx) == 1) { // convert this part to an integer
      powerJump = true;
      predictJump = true;
    }
    else {
      powerJump = false;
      predictJump = false;
    }
    if (debug && !debugPlot) {
      Serial.println("Set powerJump to " + String(powerJump));
    }
  }
  else if (strtokIndx[0] == 't') // setting different timing modes
  {
    strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation
    predStartDrive = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation
    predStopDrive = atoi(strtokIndx);
    if (debug && !debugPlot) {
      Serial.print("Start driving at mode ");
      Serial.print(predStartDrive);
      Serial.print(" and stop driving at mode ");
      Serial.println(predStopDrive);
    }
  }
  else if (strtokIndx[0] == 's') // setting the jump DC and timing
  {
    strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation
    powerJumpDutyCycle = atof(strtokIndx);     // convert this part to an integer
    if (debug && !debugPlot) {
      Serial.print("Set power jump duty cycle to ");
      Serial.println(powerJumpDutyCycle);
    }
    strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation
    powerJumpDriveDuration = atof(strtokIndx);     // convert this part to an integer
    if (debug && !debugPlot) {
      Serial.print("Set power jump drive duration to ");
      Serial.println(powerJumpDriveDuration);
    }
  }
  else if (strtokIndx[0] == 'g')
  {
    debugPlot = !debugPlot;
  }
  else if (strtokIndx[0] == 'd')
  {
    debug = !debug;
    Serial.println("Debug: " + String(debug));
  }
  else if (strtokIndx[0] == 'q') {
    strtokIndx = strtok(NULL, ","); // Get the motor power
    float motorPower = atof(strtokIndx);
    strtokIndx = strtok(NULL, ","); // Get the target position
    int targetPos = atoi(strtokIndx);
    queuedMovements.push(std::make_pair(motorPower, targetPos));
    if (debug && !debugPlot) {
      Serial.println("Set duty cycle to " + String(motorPower) + " and target position " + String(targetPos));
    }
  }
  else if (strtokIndx[0] == 'c') // Calibration routine
  {
    calibRoutine = true;
    encoderCalibration();
  }
  else if (strtokIndx[0] == 'v') // Calibration routine
  {
    Serial.println("Battery voltage: " + String(batteryVoltage) + "V");
  }
  else if (strtokIndx[0] == 'f') // Calibration routine
  {
    trigger();
  }
  else if (strtokIndx[0] == 'e') // Calibration routine
  {
    driveStatus = "brake";
    dutyCycle = 0.0;
    UART.setDuty(0);
    resetQueue();    
  }
  else if (strtokIndx[0] == 'k') { // tunning PID controller
    strtokIndx = strtok(NULL, ","); // Tune kp
    kp = atof(strtokIndx);
    strtokIndx = strtok(NULL, ","); // Tune ki
    ki = atof(strtokIndx);
    strtokIndx = strtok(NULL, ","); // Tune kd
    kd = atof(strtokIndx);
    if (debug && !debugPlot) {
      Serial.println("Set PID: P " + String(kp, DEC) + " I " + String(ki, DEC) + " D " + String(kd, DEC) );
    }
  }
  else if (strtokIndx[0] == 'j') { // tunning PID controller
    strtokIndx = strtok(NULL, ","); // Get the motor power
    vibrationMode = atoi(strtokIndx);
    
    resetQueue();
    
    if (vibrationMode == 1) { // regular
        addQueue(vibrateDC+0.15f, 30);
        addQueue(vibrateDC+0.15f, 70);
    }
    else if (vibrationMode == 2) { // up
      if (countToPercentage(encoderPosition) > 10) addQueue(vibrateDC, 0);
//      addQueue(vibrateDC, 40);
//      addQueue(vibrateDC, 20);
      addQueue(vibrateDC, 60);
      addQueue(vibrateDC, 40);
//      addQueue(vibrateDC, 80);
//      addQueue(vibrateDC, 60);
      addQueue(vibrateDC, 100);
    }
    else if (vibrationMode == 3) { // down
      if (countToPercentage(encoderPosition) < 90) addQueue(vibrateDC, 100);
//      addQueue(vibrateDC, 60);
//      addQueue(vibrateDC, 80);
      addQueue(vibrateDC, 40);
      addQueue(vibrateDC, 60);
//      addQueue(vibrateDC, 20);
//      addQueue(vibrateDC, 40);
      addQueue(vibrateDC, 0);
    }
    else if (vibrationMode == 4) { // soft vibration
      vibrationPosition = countToPercentage(encoderPosition); 
      int initialVibrationOffset = 2;
      int vibrationRange = 1;
      // Vibrate weight around current position
      if (encoderPosition >= encoderTopPosition - 10) { // if at the top, vibrate down
        vibrationPosition = encoderPosition   - initialVibrationOffset; // this is percentage
        vibrationOffset   = vibrationPosition - vibrationRange;
      }
      else{
        vibrationPosition = encoderPosition   + initialVibrationOffset;
        vibrationOffset   = vibrationPosition + vibrationRange;
      }

      // need to convert counts to percentage
      vibrationPosition = countToPercentage(vibrationPosition);
      vibrationOffset   = countToPercentage(vibrationOffset);
      
      addQueue(vibrateSoftDC, vibrationOffset);
      addQueue(vibrateSoftDC, vibrationPosition);
      Serial.print("Queuing: " + String(vibrateSoftDC) + " - " + String(vibrationOffset));
      Serial.println(" and: " + String(vibrationPosition));
    }
    else {
      resetQueue();
    }
    
    if (debug && !debugPlot) {
      Serial.println("Set vibration to: " + String(vibrationMode));
    }
  }
  else
  { // manual mode
    dutyCycle = atof(strtokIndx);  // copy it to messageFromPC
    strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation
    timeinms = atoi(strtokIndx);     // convert this part to an integer
    startJump(dutyCycle, timeinms);
  }

}

void JumpController::trigger() {
  if (queuedMovements.empty()) {
    if (debug && !debugPlot) Serial.println("Queue empty!");
    return;
  }
  
  std::pair<float, int> motorInstructions = queuedMovements.front();
  float motorPower = motorInstructions.first;
  int targetPos = motorInstructions.second;
  queuedMovements.pop();

  // clamping target position
  if (targetPosition < 0) targetPosition = 0;
  else if (targetPosition > 100) targetPosition = encoderTopPosition;
  
  targetPosition = int((targetPos * encoderTopPosition) / 100); // converts % to actual position
  if (abs(encoderPosition - targetPosition) < 5 && vibrationMode == 0) return;        // do nothing if within +/- 5 and not vibrating
  
  // making duty cycle positive value
  dutyCycle = abs(motorPower);
  if (dutyCycle > 1) dutyCycle = 0;
  if (encoderPosition < targetPosition) dutyCycle = -dutyCycle; 

  // Resetting variables for drive
  driveStatus = "initial";
//  previousTime = millis() - 50;
  lastError = 0;
  cumError = 0;
  if (debug && !debugPlot) Serial.println("Fire! " + String(dutyCycle) + " " + String(targetPosition) + " " + millis());
}

void JumpController::addQueue(float dutyCycle, int targetPos) {
  queuedMovements.push(std::make_pair(dutyCycle, targetPos));
}

void JumpController::resetQueue() {
  std::queue<std::pair<float, int>> emptyQueue;
  std::swap(queuedMovements, emptyQueue);
}
double JumpController::computePID(int input) {     
//  currentTime = millis();                               //get current time
  elapsedTime = (double)(currentTime - previousTime);   //compute time elapsed from previous computation
  
  if (elapsedTime == 0) return -prevDutyCycle_PID; // this returns NAN since function is too fast (sub millis)
  
  errorToTarget = (double) (targetPosition - input);            // determine error
  cumError += errorToTarget * elapsedTime;                      // compute integral
  rateError = (errorToTarget - lastError)/elapsedTime;          // compute derivative

  double out = kp * errorToTarget + ki * cumError + kd * rateError;   //PID output               

  lastError = errorToTarget;                                    //remember current error
//  previousTime = currentTime;                           //remember current time

  return out;                                           //have function return the PID output
}

int JumpController::countToPercentage(int positionCount){
  return int((positionCount * 100) / encoderTopPosition);
}

int JumpController::percentageToCount(int positionPercentage){
  return int((positionPercentage * encoderTopPosition) / 100);
}
