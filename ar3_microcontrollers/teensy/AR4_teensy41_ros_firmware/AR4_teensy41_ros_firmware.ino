//VERSION 3.0

/*  AR4 - Stepper motor robot control software
    Copyright (c) 2023, Chris Annin
    All rights reserved.

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

          Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
          Redistribution of this software in source or binary forms shall be free
          of all charges or fees to the recipient of this software.
          Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
          you must give appropriate credit and indicate if changes were made. You may do
          so in any reasonable manner, but not in any way that suggests the
          licensor endorses you or your use.
          Selling Annin Robotics software, robots, robot parts, or any versions of robots or software based on this
          work is strictly prohibited.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL CHRIS ANNIN BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    chris.annin@gmail.com

*/


// VERSION LOG
// 1.0 - 2/6/21 - initial release
// 1.1 - 2/20/21 - bug fix, calibration offset on negative axis calibration direction axis 2,4,5
// 2.0 - 10/1/22 - added lookahead and spline functionality
// 2.2 - 11/6/22 - added Move V for open cv integrated vision
// 3.0 - 2/3/23 - open loop bypass moved to teensy board / add external axis 8 & 9 / bug fix live jog drift


#include <math.h>
#include <avr/pgmspace.h>
#include <Encoder.h>
#include <AccelStepper.h>

// Firmware version
const char* VERSION = "0.0.1";

// approx encoder counts at rest position, 0 degree joint angle
const int REST_ENC_POSITIONS[] = { 75507, 20000, 49234, 70489, 11470, 34311 };

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
const int STEP_PINS[] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin };
const int DIR_PINS[] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin };

const int J1calPin = 26;
const int J2calPin = 27;
const int J3calPin = 28;
const int J4calPin = 29;
const int J5calPin = 30;
const int J6calPin = 31;
const int CAL_PINS[] = { J1calPin, J2calPin, J3calPin, J4calPin, J5calPin, J6calPin };

//set encoder multiplier
const float ENC_MULT[] = { 10, 10, 10, 10, 5, 10 };

//set encoder pins
Encoder J1encPos(14, 15);
Encoder J2encPos(17, 16);
Encoder J3encPos(19, 18);
Encoder J4encPos(20, 21);
Encoder J5encPos(23, 22);
Encoder J6encPos(24, 25);
Encoder encPos[6] = {
    Encoder(14, 15),
    Encoder(17, 16),
    Encoder(19, 18),
    Encoder(20, 21),
    Encoder(23, 22),
    Encoder(24, 25)
};
int ENC_DIR[] = { -1, 1, 1, 1, 1, 1 }; // +1 if encoder direction matches motor direction

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ROS Driver Params
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int NUM_JOINTS = 6;

// calibration settings
const int LIMIT_SWITCH_HIGH[] = { 1, 1, 1, 1, 1, 1 }; // to account for both NC and NO limit switches
const int CAL_DIR[] = { -1, -1, 1, -1, -1, 1 }; // joint rotation direction to limit switch
const int CAL_SPEED = 600; // motor steps per second
const int CAL_SPEED_MULT[] = { 1, 1, 1, 2, 1, 1 }; // multiplier to account for motor steps/rev

// speed and acceleration settings
float JOINT_MAX_SPEED[] = { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 }; // deg/s
float JOINT_MAX_ACCEL[] = { 5.0, 5.0, 5.0, 5.0, 5.0, 5.0 }; // deg/s^2
int MOTOR_MAX_SPEED[] = { 1500, 1500, 1500, 2000, 1500, 1500 }; // motor steps per sec
int MOTOR_MAX_ACCEL[] = { 250, 250, 250, 250, 250, 250 }; // motor steps per sec^2
float MOTOR_ACCEL_MULT[] = { 1.0, 1.0, 2.0, 1.0, 1.0, 1.0 }; // for tuning position control

AccelStepper stepperJoints[NUM_JOINTS];

enum SM { STATE_TRAJ, STATE_ERR };
SM STATE = STATE_TRAJ;

//define axis limits in degrees, for calibration
float J1axisLimPos = 170;
float J1axisLimNeg = 170;
float J2axisLimPos = 90;
float J2axisLimNeg = 42;
float J3axisLimPos = 52;
float J3axisLimNeg = 89;
float J4axisLimPos = 165;
float J4axisLimNeg = 165;
float J5axisLimPos = 105;
float J5axisLimNeg = 105;
float J6axisLimPos = 155;
float J6axisLimNeg = 155;

//define total axis travel
float J1axisLim = J1axisLimPos + J1axisLimNeg;
float J2axisLim = J2axisLimPos + J2axisLimNeg;
float J3axisLim = J3axisLimPos + J3axisLimNeg;
float J4axisLim = J4axisLimPos + J4axisLimNeg;
float J5axisLim = J5axisLimPos + J5axisLimNeg;
float J6axisLim = J6axisLimPos + J6axisLimNeg;

const float MOTOR_STEPS_PER_DEG[] = {
    44.44444444, 55.55555556, 55.55555556, 42.72664356, 21.86024888, 22.22222222 };
const int MOTOR_STEPS_PER_REV[] = { 400, 400, 400, 400, 800, 400 };

// num of steps in range of motion of joint
const int ENC_RANGE_STEPS[] = { 
    static_cast<int>(MOTOR_STEPS_PER_DEG[0] * J1axisLim * ENC_MULT[0]),
    static_cast<int>(MOTOR_STEPS_PER_DEG[1] * J2axisLim * ENC_MULT[1]),
    static_cast<int>(MOTOR_STEPS_PER_DEG[2] * J3axisLim * ENC_MULT[2]),
    static_cast<int>(MOTOR_STEPS_PER_DEG[3] * J4axisLim * ENC_MULT[3]),
    static_cast<int>(MOTOR_STEPS_PER_DEG[4] * J5axisLim * ENC_MULT[4]),
    static_cast<int>(MOTOR_STEPS_PER_DEG[5] * J6axisLim * ENC_MULT[5]),
};

String inData;

void setup() {
  // run once:
  Serial.begin(9600);

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

  pinMode(J1calPin, INPUT);
  pinMode(J2calPin, INPUT);
  pinMode(J3calPin, INPUT);
  pinMode(J4calPin, INPUT);
  pinMode(J5calPin, INPUT);
  pinMode(J6calPin, INPUT);

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

void readMotorSteps(int* motorSteps)
{
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    motorSteps[i] = encPos[i].read() / ENC_MULT[i];
  }
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

void moveOppositeABit()
{
  // first pass of calibration, fast speed
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i] * -1);
  }
  for (int j = 0; j < 10000000; j++)
  {
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
      stepperJoints[i].runSpeed();
    } 
  }
  // redundancy
  for (int i = 0; i < NUM_JOINTS; i++) {
    stepperJoints[i].setSpeed(0);
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
  // clear message
  inData = "";

  // initialise joint steps
  int curMotorSteps[NUM_JOINTS];
  readMotorSteps(curMotorSteps);

  int cmdEncSteps[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    cmdEncSteps[i] = curMotorSteps[i];
  }

  // initialise AccelStepper instance
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    stepperJoints[i].setPinsInverted(true, false, false); // DM542T CW
    stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i]);
    stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
    stepperJoints[i].setMinPulseWidth(10);
  }
  stepperJoints[3].setPinsInverted(false, false, false); // J4 DM320T CCW

  // start loop
  while (STATE == STATE_TRAJ)
  {
    char received = '\0';
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
        readMotorSteps(curMotorSteps);

        // update host with joint positions
        String msg = String("JP") + String("A") + String(curMotorSteps[0]) + String("B") + String(curMotorSteps[1]) + String("C") + String(curMotorSteps[2])
                   + String("D") + String(curMotorSteps[3]) + String("E") + String(curMotorSteps[4]) + String("F") + String(curMotorSteps[5]);
        Serial.println(msg);

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
        readMotorSteps(curMotorSteps);
        for (int i = 0; i < NUM_JOINTS; ++i)
        { 
          int diffEncSteps = cmdEncSteps[i] - curMotorSteps[i];
          if (abs(diffEncSteps) > 2)
          {
            int diffMotSteps = diffEncSteps * ENC_DIR[i];
            // if (diffMotSteps < MOTOR_STEPS_PER_REV[i])
            // {
            //   // for the last rev of motor, introduce artificial decceleration
            //   // to help prevent overshoot
            //   diffMotSteps = diffMotSteps / 2;
            // }
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


        // calibrate joints 1 to 5
        int calJoints[] = { 1, 1, 1, 1, 1, 0 }; // 111110
        calibrateJoints(calJoints);

        // record encoder steps
        int calSteps[6];
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
          calSteps[i] = encPos[i].read();
        }

        J1encPos.write(ENC_RANGE_STEPS[0]);
        J2encPos.write(0);
        J3encPos.write(ENC_RANGE_STEPS[2]);
        J4encPos.write(0);
        J5encPos.write(0);
        J6encPos.write(ENC_RANGE_STEPS[5]);

        moveOppositeABit();

        // return to original position
        for (int i = 0; i < NUM_JOINTS; ++i) {
          stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i]);
          stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
        }

        bool restPosReached = false;
        while (!restPosReached) {
          restPosReached = true;
          readMotorSteps(curMotorSteps);

          for (int i = 0; i < NUM_JOINTS; ++i) {
            if (abs(REST_ENC_POSITIONS[i] / ENC_MULT[i] - curMotorSteps[i]) > 5) {
              restPosReached = false;
              float target_pos = (REST_ENC_POSITIONS[i] / ENC_MULT[i]  - curMotorSteps[i]) * ENC_DIR[i];
              stepperJoints[i].move(target_pos);
              stepperJoints[i].run();
            }
          }
        }

        // calibration done, send calibration values
        String msg = String("JC") + "A" + String(calSteps[0]) + "B" + String(calSteps[1]) + "C" + String(calSteps[2])
                   + "D" + String(calSteps[3]) + "E" + String(calSteps[4]) + "F" + String(calSteps[5]);
        Serial.println(msg);

        for (int i = 0; i < NUM_JOINTS; ++i) {
            stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i]);
            stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
        }
      }
      else if (function == "JP")
      {
        // read current joint positions
        readMotorSteps(curMotorSteps);

        // update host with joint positions
        String msg = String("JP") + "A" + String(curMotorSteps[0]) + "B" + String(curMotorSteps[1]) + "C" + String(curMotorSteps[2])
                   + "D" + String(curMotorSteps[3]) + "E" + String(curMotorSteps[4]) + "F" + String(curMotorSteps[5]);
        Serial.println(msg);
      }
      else if (function == "SS")
      {
        updateStepperSpeed(inData);
        // set motor speed and acceleration
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
          MOTOR_MAX_SPEED[i] = JOINT_MAX_SPEED[i] * MOTOR_STEPS_PER_DEG[i];
          MOTOR_MAX_ACCEL[i] = JOINT_MAX_ACCEL[i] * MOTOR_STEPS_PER_DEG[i];
          stepperJoints[i].setAcceleration(MOTOR_MAX_ACCEL[i] * MOTOR_ACCEL_MULT[i]);
          stepperJoints[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
        }
        // read current joint positions
        readMotorSteps(curMotorSteps);

        // update host with joint positions
        String msg = String("JP") + "A" + String(curMotorSteps[0]) + "B" + String(curMotorSteps[1]) + "C" + String(curMotorSteps[2])
                   + "D" + String(curMotorSteps[3]) + "E" + String(curMotorSteps[4]) + "F" + String(curMotorSteps[5]);
        Serial.println(msg);
      }
      else if (function == "ST")
      {
        if (!initStateTraj(inData))
        {
          STATE = STATE_ERR;
          return;
        }

      }
      
      // clear message
      inData = "";
    }
    // // execute motor commands
    // for (int i = 0; i < NUM_JOINTS; ++i)
    // {
    //   // target joint positions are already updated, just call run()
    //   stepperJoints[i].run();
    // }
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

  // do recovery
  while (STATE == STATE_ERR) {}
}

void loop() 
{  
  //test traj state
  STATE = STATE_TRAJ;

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
      stateTRAJ();
      break;
  }
}


