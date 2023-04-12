/*
    JumpController.h -- firmaware for JumpMod haptic backpack
    built for Seeeduino XIAO nRF52840 Sense with Arduino
*/

#ifndef JumpController_h
#define JumpController_h
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VescUart.h>
#include <Wire.h>
#include <math.h>
#include <Queue.h>
#include <utility>

class JumpController
{
  public:
    // Functions
    void loop(float YZ);
    void driveController();
    void parseData(char* tempChars);
    void getPosition();
    void encoderCalibration();
    void driveMotor();
    void drivePosition();
    double computePID(int input);
    void startJump(float newDutyCycle, int newTimeInMS);
    void setup();
    void predictJumpStatus(float YZ);
    void printJumpStatus();
    void trigger();
    void addQueue(float dutyCycle, int targetPos);
    void resetQueue();
    int countToPercentage(int positionCount);
    int percentageToCount(int positionPercentage);

    // Settings
    bool debug = false; // Frequently prints data
    bool useBlinkCommands = true; // Disables regular light blinking so BLE can manually trigger blinking
    bool predictJump = false; // Runs the jump prediction algorithm and uses it to control the motor

    bool debugPlot = false; // Prints only acceleration, displacement, and velocity in serial-plotter-readable format
    bool printDT = false;
    bool debugSW = false;
    bool debugPredict = false;
    bool debugPosition = true;
    bool debugBLE = false;
    bool debugPID = false;

    bool drive = true;
    bool onlyPrediction = false;
    bool calibRoutine = false;

    // Trigger queing controls
    std::queue<std::pair<float, int>> queuedMovements;


    // driving variables
    float dutyCycle = 0;
    float dutyCycle_PID = 0;
    float prevDutyCycle_PID = 0;
    int targetPosition = 0;
    int timeinms = 0;
    int predStartDrive = 3;
    int predStopDrive = 4;
    float batteryVoltage = 0;
    int vibrationMode = 0; // 0 = none, 1 = vibrate regular, 2 = vibrate up, 3 = vibrate down
    float vibrateDC = 0.15f;
    float vibrateSoftDC = 0.05f;
    int vibrationPosition = 0;
    int vibrationOffset = 0;
    
    // encoder position
    int encoderPosition = 0;
    int encoderPrevPosition;
    int encoderTopPosition = 101; // tune this accordingly or run calibRouting()  
    bool encoderReset = false;  // avoid race condition when reseting encoder value at each end

    // PID variables
    double kp = 0.007;
    double ki = 0.000001;
    double kd = 0.1;
    
    unsigned long currentTime, previousTime;
    double elapsedTime;
    double errorToTarget;
    double lastError;
    double input, output;
    double cumError, rateError;
    
    // Timer
    unsigned long timeZero, timePrev;
    unsigned long timeAtTarget;
    unsigned long prevLoopTime = 0;

    // Jump Status
    enum JumpStatus {
      STAND = 1,
      CROUCH,
      LAUNCH,
      LIFTOFF,
      ASCENT,
      APEX,
      DESCENT,
      LANDING,
      REBOUND
    };
    JumpStatus jumpStatus = STAND;
    int statusStart = 0;

    // Callibration
    int sameCount = 0;
    int sameCountMax = 3; // Reset to 0 velocity when sameCount reaches this value
    float launchDetectionConstant = -0.3;
    float crouchBound = -0.3;
    float liftoffDetectionFactor = 0.8; // When acceleration goes below this factor of maxAcceleration, liftoff is achieved
    int belowAccelTime = 0;
    int maxBelowAccelTime = 6;
    float liftoffErrorFactor = -0.74;//-0.76
    float liftoffMaxErrorFactor = 1.05;//0.9;//1.1;
    float liftoffErrorConstant = 0;
    float landingDetectionFactor = 0.8;
    float landingErrorFactor = 0;//-0.3;
    float landingErrorConstant = 0;
    float recoverDetectionConstant = -0.5;
    unsigned long resetTime = 1100;

    // Physics parameters
    float velocity = 0;
    float displacement = 0;

    // Extrema values
    float maxVelocity = -999;
    float minVelocity = 999;
    float maxAccel = -999;
    float minDisplacement = 999;
    int timeSinceMax = 0;
    int timeSinceMin = 0;
    int timeSinceMaxAccel = 0;
    int timeSinceMinDisplacement = 0;
    int extremaDelta = 1;

    bool maxVelocitySet = false;

    float powerJumpDutyCycle = -.5;
    float powerJumpDriveDuration = 100;
    float lastPredUpdate = 0;

    unsigned long deltaTime = 0;

    /** Initiate VescUart class */
    VescUart UART;

    float current = 1.0; /** The current in amps */
    float currentBrake = 20.0;

    int firstDrive = 0;

    unsigned long lastPrint = 0;

    String driveStatus = "none";

    boolean powerJump = false;
    String powerJumpPhase = "none";

    // Light command
    bool lightCommandOn = false;

  private:
    void resetJumpStatus();
    static void isrA();
    static void isrB();
};

#endif
