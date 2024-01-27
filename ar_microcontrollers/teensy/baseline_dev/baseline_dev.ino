#include <Encoder.h>
#include <avr/pgmspace.h>

#include <AccelStepper.h>

// Firmware version
const char* VERSION = "0.0.1";

const int NUM_JOINTS = 6;

/*
  MOTOR DIRECTION - motor directions can be changed on the caibration page in the software but can also
  be changed here: example using DM542T driver(CW) set to 1 - if using ST6600 or DM320T driver(CCW) set to 0
  DEFAULT = 111011   */

const int J1rotdir = 1;
const int J2rotdir = 1;
const int J3rotdir = 1;
const int J4rotdir = 0;
const int J5rotdir = 1;
const int J6rotdir = 1;
const int ROT_DIRS[] = { J1rotdir, J2rotdir, J3rotdir, J4rotdir, J5rotdir, J6rotdir };

// approx encoder counts at rest position, 0 degree joint angle
// DEFAULTS 38681, 11264, 0, 36652, 5839, 15671
// J2 and J6 joint enc { 38681, 902, 0, 36652, 2198, 15671 }
const int REST_ENC_POSITIONS[] = { 38681, 11264, 0, 36652, 5839, 15671 };

const int J1stepPin = 0;
const int J1dirPin = 1;
const int J2stepPin = 2;
const int J2dirPin = 3;
const int J3stepPin = 4;
const int J3dirPin = 5;
const int J4stepPin = 6;
const int J4dirPin = 7;
const int J5stepPin = 8;
const int J5dirPin = 9;
const int J6stepPin = 10;
const int J6dirPin = 11;
const int TRstepPin = 12;
const int TRdirPin = 13;
const int STEP_PINS[] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin };
const int DIR_PINS[] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin };

// set encoder pins
Encoder J1encPos(14, 15);
Encoder J2encPos(16, 17);
Encoder J3encPos(18, 19);
Encoder J4encPos(20, 21);
Encoder J5encPos(22, 23);
Encoder J6encPos(24, 25);
Encoder JOINT_ENCODERS[] = { J1encPos, J2encPos, J3encPos, J4encPos, J5encPos, J6encPos };
// DEFAULTS { 1, 1, 1, 1, 1, 1 }
// J2 joint enc  { 1, -1, 1, 1, 1, 1 }
int ENC_DIR[] = { 1, 1, 1, 1, 1, 1 }; // +1 if encoder direction matches motor direction

// set calibration limit switch pins
const int J1calPin = 26;
const int J2calPin = 27;
const int J3calPin = 28;
const int J4calPin = 29;
const int J5calPin = 30;
const int J6calPin = 31;
const int CAL_PINS[] = { J1calPin, J2calPin, J3calPin, J4calPin, J5calPin, J6calPin };
const int LIMIT_SWITCH_HIGH[] = { 1, 1, 1, 1, 1, 1 }; // to account for both NC and NO limit switches
const int CAL_DIR[] = { -1, -1, 1, -1, -1, 1 }; // joint rotation direction to limit switch
const int CAL_SPEED = 600; // motor steps per second
// DEFAULT { 1, 1, 1, 2, 1, 1 }
// J2 joint enc { 1, 10, 1, 2, 1, 1 }
const int CAL_SPEED_MULT[] = { 1, 1, 1, 2, 1, 1 }; // multiplier to account for transmission ratios

// motor and encoder steps per revolution
// DEFAULTS { 400, 400, 400, 400, 800, 400 }
// J2 motor 4000 steps per rev { 400, 4000, 400, 400, 800, 400 }
const int MOTOR_STEPS_PER_REV[] = { 400, 400, 400, 400, 800, 400 };

// num of steps in range of motion of joint, may vary depending on your limit switch
// DEFAULT { 77363, 36854, 40878, 71967, 23347, 32358 }
// J2 joint enc { 77363, 2949, 40878, 71967, 4740, 32358 }
const int ENC_RANGE_STEPS[] = { 77363, 36854, 40878, 71967, 23347, 32358 };

//set encoder multiplier, encoder steps per motor step
// DEFAULT { 5.12, 5.12, 5.12, 5.12, 2.56, 5.12 }
// J2 and J5 joint enc { 5.12, 0.04096, 5.12, 5.12, 0.5197, 5.12 }
const float ENC_MULT[] = { 5.12, 5.12, 5.12, 5.12, 2.56, 5.12 };
// DEFAULT { 227.5555555555556, 284.4444444444444, 284.4444444444444, 223.0044444444444, 56.04224675948152, 108.0888888888889 }
// J2 and J5 joint enc { 227.5555555555556, 22.75555555555556, 284.4444444444444, 223.0044444444444, 22.75555555555556, 108.0888888888889 }
const float ENC_STEPS_PER_DEG[] = { 227.5555555555556, 284.4444444444444, 284.4444444444444, 223.0044444444444, 56.04224675948152, 108.0888888888889 };

// speed and acceleration settings
float JOINT_MAX_SPEED[] = { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 }; // deg/s
float JOINT_MAX_ACCEL[] = { 5.0, 5.0, 5.0, 5.0, 5.0, 5.0 }; // deg/s^2
// DEFAULT { 1500, 1500, 1500, 2000, 1500, 1500 }
// J2 4000 steps/rev { 1500, 15000, 1500, 2000, 1500, 1500 }
int MOTOR_MAX_SPEED[] = { 1500, 1500, 1500, 2000, 1500, 1500 }; // motor steps per sec
// DEFAULT { 250, 250, 250, 250, 250, 250 }
// J2 4000 steps/rev { 250, 2500, 250, 250, 250, 250 }
int MOTOR_MAX_ACCEL[] = { 250, 250, 250, 250, 250, 250 }; // motor steps per sec^2

// played around with this a little but I don't see a need to use this at the moment
float MOTOR_SPEED_MULT[] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 }; // for tuning position control
float MOTOR_ACCEL_MULT[] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 }; // for tuning position control

AccelStepper stepperJoints[NUM_JOINTS];

enum SM { STATE_INIT, STATE_TRAJ, STATE_ERR };
SM STATE = STATE_INIT;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // run once:
  Serial.begin(115200);

  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    JOINT_ENCODERS[i].write(REST_ENC_POSITIONS[i]);
  }

  pinMode(TRstepPin, OUTPUT);
  pinMode(TRdirPin, OUTPUT);
  pinMode(J1stepPin, OUTPUT);
  pinMode(J1dirPin, OUTPUT);
  pinMode(J2stepPin, OUTPUT);
  pinMode(J2dirPin, OUTPUT);
  pinMode(J3stepPin, OUTPUT);
  pinMode(J3dirPin, OUTPUT);
  pinMode(J4stepPin, OUTPUT);
  pinMode(J4dirPin, OUTPUT);
  pinMode(J5stepPin, OUTPUT);
  pinMode(J5dirPin, OUTPUT);
  pinMode(J6stepPin, OUTPUT);
  pinMode(J6dirPin, OUTPUT);

  pinMode(J1calPin, INPUT_PULLUP);
  pinMode(J2calPin, INPUT_PULLUP);
  pinMode(J3calPin, INPUT_PULLUP);
  pinMode(J4calPin, INPUT_PULLUP);
  pinMode(J5calPin, INPUT_PULLUP);
  pinMode(J6calPin, INPUT_PULLUP);

  digitalWrite(TRstepPin, HIGH);
  digitalWrite(J1stepPin, HIGH);
  digitalWrite(J2stepPin, HIGH);
  digitalWrite(J3stepPin, HIGH);
  digitalWrite(J4stepPin, HIGH);
  digitalWrite(J5stepPin, HIGH);
  digitalWrite(J6stepPin, HIGH);

}

bool initStateTraj(String inData)
{
  // parse initialisation message
  int idxVersion = inData.indexOf('A');
  String softwareVersion = inData.substring(idxVersion + 1, inData.length() - 1);
  int versionMatches = (softwareVersion == VERSION);

  // return acknowledgement with result
  String msg = String("ST") + String("A") + String(versionMatches) + String("B") + String(VERSION) + String("\n");
  Serial.print(msg);

  return versionMatches ? true : false;
}

void readEncPos(int* encPos)
{
  encPos[0] = J1encPos.read() * ENC_DIR[0];
  encPos[1] = J2encPos.read() * ENC_DIR[1];
  encPos[2] = J3encPos.read() * ENC_DIR[2];
  encPos[3] = J4encPos.read() * ENC_DIR[3];
  encPos[4] = J5encPos.read() * ENC_DIR[4];
  encPos[5] = J6encPos.read() * ENC_DIR[5];
}

void updateStepperSpeed(String inData)
{
  int idxSpeedJ1 = inData.indexOf('A');
  int idxAccelJ1 = inData.indexOf('B');
  int idxSpeedJ2 = inData.indexOf('C');
  int idxAccelJ2 = inData.indexOf('D');
  int idxSpeedJ3 = inData.indexOf('E');
  int idxAccelJ3 = inData.indexOf('F');
  int idxSpeedJ4 = inData.indexOf('G');
  int idxAccelJ4 = inData.indexOf('H');
  int idxSpeedJ5 = inData.indexOf('I');
  int idxAccelJ5 = inData.indexOf('J');
  int idxSpeedJ6 = inData.indexOf('K');
  int idxAccelJ6 = inData.indexOf('L');

  JOINT_MAX_SPEED[0] = inData.substring(idxSpeedJ1 + 1, idxAccelJ1).toFloat();
  JOINT_MAX_ACCEL[0] = inData.substring(idxAccelJ1 + 1, idxSpeedJ2).toFloat();
  JOINT_MAX_SPEED[1] = inData.substring(idxSpeedJ2 + 1, idxAccelJ2).toFloat();
  JOINT_MAX_ACCEL[1] = inData.substring(idxAccelJ2 + 1, idxSpeedJ3).toFloat();
  JOINT_MAX_SPEED[2] = inData.substring(idxSpeedJ3 + 1, idxAccelJ3).toFloat();
  JOINT_MAX_ACCEL[2] = inData.substring(idxAccelJ3 + 1, idxSpeedJ4).toFloat();
  JOINT_MAX_SPEED[3] = inData.substring(idxSpeedJ4 + 1, idxAccelJ4).toFloat();
  JOINT_MAX_ACCEL[3] = inData.substring(idxAccelJ4 + 1, idxSpeedJ5).toFloat();
  JOINT_MAX_SPEED[4] = inData.substring(idxSpeedJ5 + 1, idxAccelJ5).toFloat();
  JOINT_MAX_ACCEL[4] = inData.substring(idxAccelJ5 + 1, idxSpeedJ6).toFloat();
  JOINT_MAX_SPEED[5] = inData.substring(idxSpeedJ6 + 1, idxAccelJ6).toFloat();
  JOINT_MAX_ACCEL[5] = inData.substring(idxAccelJ6 + 1).toFloat();
}

void calibrateJoints(int* calJoints)
{
  // check which joints to calibrate
  bool calAllDone = false;
  bool calJointsDone[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    calJointsDone[i] = !calJoints[i];
  }
  
  // first pass of calibration, fast speed
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i]);
  }
  while (!calAllDone)
  {
    calAllDone = true;
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
      // if joint is not calibrated yet
      if (!calJointsDone[i])
      {
        // check limit switches
        if (!reachedLimitSwitch(i))
        {
          // limit switch not reached, continue moving
          stepperJoints[i].runSpeed();
          calAllDone = false;
        }
        else
        {
          // limit switch reached
          stepperJoints[i].setSpeed(0); // redundancy
          calJointsDone[i] = true;
        }   
      }   
    } 
  }
  delay(2000);

  return;
}

bool reachedLimitSwitch(int joint)
{
  int pin = CAL_PINS[joint];
  // check multiple times to deal with noise
  // possibly EMI from motor cables?
  if (digitalRead(pin) == LIMIT_SWITCH_HIGH[joint])
  {
    if (digitalRead(pin) == LIMIT_SWITCH_HIGH[joint])
    {
      if (digitalRead(pin) == LIMIT_SWITCH_HIGH[joint])
      {
        if (digitalRead(pin) == LIMIT_SWITCH_HIGH[joint])
        {
          if (digitalRead(pin) == LIMIT_SWITCH_HIGH[joint])
          {
            return true;
          }
        }
      }
    }
  }
  return false;
}

void stateTRAJ()
{
  // initialise joint steps
  int curEncSteps[NUM_JOINTS];
  readEncPos(curEncSteps);

  int cmdEncSteps[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    cmdEncSteps[i] = curEncSteps[i];
  }

  // initialise AccelStepper instance
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    stepperJoints[i].setPinsInverted(true, false, false); // DM542T CW
    stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i]);
    stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
  }
  stepperJoints[3].setPinsInverted(false, false, false); // J4 DM320T CCW

  // message
  String inData = "";

  // start loop
  while (STATE == STATE_TRAJ)
  {
    char received = "";
    // check for message from host
    if (Serial.available())
    {
      received = Serial.read();
      inData += received;
    }

    // process message when new line character is received
    if (received == '\n')
    {
      String function = inData.substring(0, 2);
      // update trajectory information
      if (function == "MT")
      {
        // read current joint positions
        readEncPos(curEncSteps);

        // update host with joint positions
        String msg = String("JP") + String("A") + String(curEncSteps[0]) + String("B") + String(curEncSteps[1]) + String("C") + String(curEncSteps[2])
                   + String("D") + String(curEncSteps[3]) + String("E") + String(curEncSteps[4]) + String("F") + String(curEncSteps[5]) + String("\n");
        Serial.print(msg);

        // get new position commands
        int msgIdxJ1 = inData.indexOf('A');
        int msgIdxJ2 = inData.indexOf('B');
        int msgIdxJ3 = inData.indexOf('C');
        int msgIdxJ4 = inData.indexOf('D');
        int msgIdxJ5 = inData.indexOf('E');
        int msgIdxJ6 = inData.indexOf('F');
        cmdEncSteps[0] = inData.substring(msgIdxJ1 + 1, msgIdxJ2).toInt();
        cmdEncSteps[1] = inData.substring(msgIdxJ2 + 1, msgIdxJ3).toInt();
        cmdEncSteps[2] = inData.substring(msgIdxJ3 + 1, msgIdxJ4).toInt();
        cmdEncSteps[3] = inData.substring(msgIdxJ4 + 1, msgIdxJ5).toInt();
        cmdEncSteps[4] = inData.substring(msgIdxJ5 + 1, msgIdxJ6).toInt();
        cmdEncSteps[5] = inData.substring(msgIdxJ6 + 1).toInt();

        // update target joint positions
        readEncPos(curEncSteps);
        for (int i = 0; i < NUM_JOINTS; ++i)
        { 
          curEncSteps[1] = J2encPos.read() * ENC_DIR[1];
          int diffEncSteps = cmdEncSteps[i] - curEncSteps[i];
          if (abs(diffEncSteps) > 2)
          {
            int diffMotSteps = diffEncSteps / ENC_MULT[i];
            if (diffMotSteps < MOTOR_STEPS_PER_REV[i])
            {
              // for the last rev of motor, introduce artificial decceleration
              // to help prevent overshoot
              diffMotSteps = diffMotSteps / 2;
            }
            stepperJoints[i].move(diffMotSteps);
            stepperJoints[i].run();
          }
        }
      }
      else if (function == "JC")
      {
        // calibrate joint 6
        int calJoint6[] = { 0, 0, 0, 0, 0, 1 }; // 000001
        calibrateJoints(calJoint6);

        // record encoder steps
        int calStepJ6 = J6encPos.read() * ENC_DIR[5];

        // calibrate joints 1 to 5
        int calJoints[] = { 1, 1, 1, 1, 1, 0 }; // 111110
        calibrateJoints(calJoints);

        // record encoder steps
        int calStepJ1 = J1encPos.read() * ENC_DIR[0];
        int calStepJ2 = J2encPos.read() * ENC_DIR[1];
        int calStepJ3 = J3encPos.read() * ENC_DIR[2];
        int calStepJ4 = J4encPos.read() * ENC_DIR[3];
        int calStepJ5 = J5encPos.read() * ENC_DIR[4];

        // if limit switch at lower end, set encoder to 0
        // otherwise set to encoder upper limit
        J1encPos.write(0);
        J2encPos.write(0);
        J3encPos.write(ENC_RANGE_STEPS[2]);
        J4encPos.write(0);
        J5encPos.write(0);
        J6encPos.write(ENC_RANGE_STEPS[5]);

        // read current joint positions
        readEncPos(curEncSteps);

        // return to original position
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i]);
            stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
            cmdEncSteps[i] = REST_ENC_POSITIONS[i];
            stepperJoints[i].move((REST_ENC_POSITIONS[i] - curEncSteps[i]) / ENC_MULT[i]);
        }

        bool restPosReached = false;
        while (!restPosReached)
        {
          restPosReached = true;
          // read current joint positions
          readEncPos(curEncSteps);
          for (int i = 0; i < NUM_JOINTS; ++i)
          {
            if (abs(REST_ENC_POSITIONS[i] - curEncSteps[i]) > 5)
            {
              restPosReached = false;
              stepperJoints[i].move((REST_ENC_POSITIONS[i] - curEncSteps[i]) / ENC_MULT[i]);
              stepperJoints[i].run();
            }
          }
        }
        
        // calibration done, send calibration values
        String msg = String("JC") + String("A") + String(calStepJ1) + String("B") + String(calStepJ2) + String("C") + String(calStepJ3)
                  + String("D") + String(calStepJ4) + String("E") + String(calStepJ5) + String("F") + String(calStepJ6) + String("\n");
        Serial.print(msg);

        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i]);
            stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
        }
      }
      else if (function == "JP")
      {
        // read current joint positions
        readEncPos(curEncSteps);

        // update host with joint positions
        String msg = String("JP") + String("A") + String(curEncSteps[0]) + String("B") + String(curEncSteps[1]) + String("C") + String(curEncSteps[2])
                   + String("D") + String(curEncSteps[3]) + String("E") + String(curEncSteps[4]) + String("F") + String(curEncSteps[5]) + String("\n");
        Serial.print(msg);
      }
      else if (function == "SS")
      {
        updateStepperSpeed(inData);
        // set motor speed and acceleration
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
          MOTOR_MAX_SPEED[i] = JOINT_MAX_SPEED[i] * ENC_STEPS_PER_DEG[i] / ENC_MULT[i];
          MOTOR_MAX_ACCEL[i] = JOINT_MAX_ACCEL[i] * ENC_STEPS_PER_DEG[i] / ENC_MULT[i];
          stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i] * MOTOR_ACCEL_MULT[i]);
          stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i] * MOTOR_SPEED_MULT[i]);
        }
        // read current joint positions
        readEncPos(curEncSteps);

        // update host with joint positions
        String msg = String("JP") + String("A") + String(curEncSteps[0]) + String("B") + String(curEncSteps[1]) + String("C") + String(curEncSteps[2])
                   + String("D") + String(curEncSteps[3]) + String("E") + String(curEncSteps[4]) + String("F") + String(curEncSteps[5]) + String("\n");
        Serial.print(msg);
      }
      else if (function == "ST")
      {
        if (!initStateTraj(inData))
        {
          STATE = STATE_INIT;
          return;
        }
      }
      
      // clear message
      inData = "";
    }
    // execute motor commands
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
      // target joint positions are already updated, just call run()
      stepperJoints[i].run();
    }
  }
}

void stateINIT()
{
  // message
  String inData = "";

  // start loop
  while (STATE == STATE_INIT)
  {
    char received = "";
    // check for message from host
    if (Serial.available())
    {
      received = Serial.read();
      inData += received;
    }

    // process message when new line character is received
    if (received == '\n')
    {
      String function = inData.substring(0, 2);
      // update trajectory information
      if (function == "ST")
      {
        if (initStateTraj(inData))
        {
          STATE = STATE_TRAJ;
          return;
        }
      }
      inData = "";
    }
  }
}

void stateERR()
{
  // enter holding state
  digitalWrite(J1stepPin, LOW);
  digitalWrite(J2stepPin, LOW);
  digitalWrite(J3stepPin, LOW);
  digitalWrite(J4stepPin, LOW);
  digitalWrite(J5stepPin, LOW);
  digitalWrite(J6stepPin, LOW);

  // do recovery @TODO
  while (STATE == STATE_ERR) {}
}

void loop() 
{  
  // state control
  switch (STATE)
  {
    case STATE_TRAJ:
      stateTRAJ();
      break;
    case STATE_ERR:
      stateERR();
      break;
    default:
      stateINIT();
      break;
  }
}
