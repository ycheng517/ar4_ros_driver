/*  ARCS - Stepper motor robot control software Ver 1.0
    Copyright (c) 2019, Chris Annin
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
          Selling AR2 software, robots, robot parts, or any versions of robots or software based on this
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

    Log:

*/

// SPEED // millisecond multiplier // raise value to slow robot speeds // DEFAULT = 220
const int SpeedMult = 220;

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

const int TRACKrotdir = 0;

/* start positions - these are the joint step values at power up, default is in the rest position using
  the following values: J1=7600, J2=2322, J3=0, J4=7600, J5=2287, J6=3312 */

int J1startSteps = 7600;
int J2startSteps = 2322;
int J3startSteps = 0;
int J4startSteps = 7600;
int J5startSteps = 2287;
int J6startSteps = 3312;


#include <Encoder.h>
#include <avr/pgmspace.h>

String inData;
String function;
char WayPt[200][70];
int WayPtDel;

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

//set encoder pins
Encoder J1encPos(14, 15);
Encoder J2encPos(16, 17);
Encoder J3encPos(18, 19);
Encoder J4encPos(20, 21);
Encoder J5encPos(22, 23);
Encoder J6encPos(24, 25);

//set calibration limit switch pins
const int J1calPin = 26;
const int J2calPin = 27;
const int J3calPin = 28;
const int J4calPin = 29;
const int J5calPin = 30;
const int J6calPin = 31;

//set encoder multiplier
const float J1encMult = 5.12;
const float J2encMult = 5.12;
const float J3encMult = 5.12;
const float J4encMult = 5.12;
const float J5encMult = 2.56;
const float J6encMult = 5.12;
const float EncDiv = .1;






/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // run once:
  Serial.begin(115200);


  J1encPos.write(J1startSteps * J1encMult);
  J2encPos.write(J2startSteps * J2encMult);
  J3encPos.write(J3startSteps * J3encMult);
  J4encPos.write(J4startSteps * J4encMult);
  J5encPos.write(J5startSteps * J5encMult);
  J6encPos.write(J6startSteps * J6encMult);

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


void loop() {

  //start loop
  WayPtDel = 0;
  while (Serial.available() > 0 or WayPtDel == 1)
  {
    char recieved = Serial.read();
    inData += recieved;
    // Process message when new line character is recieved
    if (recieved == '\n')
    {
      String function = inData.substring(0, 2);


      //-----COMMAND TO WAIT TIME---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "WT")
      {
        int WTstart = inData.indexOf('S');
        float WaitTime = inData.substring(WTstart + 1).toFloat();
        int WaitTimeMS = WaitTime * 1000;
        delay(WaitTimeMS);
        Serial.print("Done");
      }


      //-----COMMAND GET ROBOT POSITION---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "GP")
      {
        int J1Tstart = inData.indexOf('U');
        int J2Tstart = inData.indexOf('V');
        int J3Tstart = inData.indexOf('W');
        int J4Tstart = inData.indexOf('X');
        int J5Tstart = inData.indexOf('Y');
        int J6Tstart = inData.indexOf('Z');

        int J1tarStep = inData.substring(J1Tstart + 1, J2Tstart).toInt();
        int J2tarStep = inData.substring(J2Tstart + 1, J3Tstart).toInt();
        int J3tarStep = inData.substring(J3Tstart + 1, J4Tstart).toInt();
        int J4tarStep = inData.substring(J4Tstart + 1, J5Tstart).toInt();
        int J5tarStep = inData.substring(J5Tstart + 1, J6Tstart).toInt();
        int J6tarStep = inData.substring(J6Tstart + 1).toInt();

        int ErrorTrue = 0;
        String ErrorCode = "00";
        String J1error = "0";
        String J2error = "0";
        String J3error = "0";
        String J4error = "0";
        String J5error = "0";
        String J6error = "0";

        int J1curStep = J1encPos.read() / J1encMult;
        int J2curStep = J2encPos.read() / J2encMult;
        int J3curStep = J3encPos.read() / J3encMult;
        int J4curStep = J4encPos.read() / J4encMult;
        int J5curStep = J5encPos.read() / J5encMult;
        int J6curStep = J6encPos.read() / J6encMult;

        if (abs(J1curStep - J1tarStep) >  J1encMult / EncDiv)
        {
          J1error = "1";
          ErrorTrue = 1;
        }
        if (abs(J2curStep - J2tarStep) >  J2encMult / EncDiv)
        {
          J2error = "1";
          ErrorTrue = 1;
        }
        if (abs(J3curStep - J3tarStep) >  J3encMult / EncDiv)
        {
          J3error = "1";
          ErrorTrue = 1;
        }
        if (abs(J4curStep - J4tarStep) >  J4encMult / EncDiv)
        {
          J4error = "1";
          ErrorTrue = 1;
        }
        if (abs(J5curStep - J5tarStep) >  J5encMult / EncDiv)
        {
          J5error = "1";
          ErrorTrue = 1;
        }
        if (abs(J6curStep - J6tarStep) >  J6encMult / EncDiv)
        {
          J6error = "1";
          ErrorTrue = 1;
        }

        if (ErrorTrue == 1)
        {
          ErrorCode = "01" + J1error + J2error + J3error + J4error + J5error + J6error + "A" + String(J1curStep) + "B" + String(J2curStep) + "C" + String(J3curStep) + "D" + String(J4curStep) + "E" + String(J5curStep) + "F" + String(J6curStep);
        }
        else if (ErrorTrue == 0)
        {
          ErrorCode = "00" + J1error + J2error + J3error + J4error + J5error + J6error + "A" + String(J1curStep) + "B" + String(J2curStep) + "C" + String(J3curStep) + "D" + String(J4curStep) + "E" + String(J5curStep) + "F" + String(J6curStep);
          J1encPos.write(J1tarStep * J1encMult);
          J2encPos.write(J2tarStep * J2encMult);
          J3encPos.write(J3tarStep * J3encMult);
          J4encPos.write(J4tarStep * J4encMult);
          J5encPos.write(J5tarStep * J5encMult);
          J6encPos.write(J6tarStep * J6encMult);
        }
        //Serial.print(ErrorCode);
        Serial.print(ErrorCode);
        Serial.println();
      }


      //-----COMMAND CALIBRATE ENCODERS-----------------------------------------
      //-----------------------------------------------------------------------
      if (function == "LM")
      {
        int J1start = inData.indexOf('A');
        int J2start = inData.indexOf('B');
        int J3start = inData.indexOf('C');
        int J4start = inData.indexOf('D');
        int J5start = inData.indexOf('E');
        int J6start = inData.indexOf('F');
        ///
        int J1step = (inData.substring(J1start + 1, J2start).toInt());
        int J2step = (inData.substring(J2start + 1, J3start).toInt());
        int J3step = (inData.substring(J3start + 1, J4start).toInt());
        int J4step = (inData.substring(J4start + 1, J5start).toInt());
        int J5step = (inData.substring(J5start + 1, J6start).toInt());
        int J6step = (inData.substring(J6start + 1).toInt());
        ///
        int J1ePos = J1step * J1encMult;
        int J2ePos = J2step * J2encMult;
        int J3ePos = J3step * J3encMult;
        int J4ePos = J4step * J4encMult;
        int J5ePos = J5step * J5encMult;
        int J6ePos = J6step * J6encMult;
        //
        J1encPos.write(J1ePos);
        J2encPos.write(J2ePos);
        J3encPos.write(J3ePos);
        J4encPos.write(J4ePos);
        J5encPos.write(J5ePos);
        J6encPos.write(J6ePos);
        ///
        Serial.print("Done");
      }



      ///////////////////////////////////////////////////////////////////////////////////////////
      //-----COMMAND DRIVE TO LIMIT SWITCHES-----------------------------------
      //-----------------------------------------------------------------------
      ///////////////////////////////////////////////////////////////////////////////////////////
      if (function == "LL")
      {
        int J1start = inData.indexOf('A');
        int J2start = inData.indexOf('B');
        int J3start = inData.indexOf('C');
        int J4start = inData.indexOf('D');
        int J5start = inData.indexOf('E');
        int J6start = inData.indexOf('F');
        int SPstart = inData.indexOf('S');
        ///
        int J1caldir = inData.substring(J1start + 1, J1start + 2).toInt();
        int J2caldir = inData.substring(J2start + 1, J2start + 2).toInt();
        int J3caldir = inData.substring(J3start + 1, J3start + 2).toInt();
        int J4caldir = inData.substring(J4start + 1, J4start + 2).toInt();
        int J5caldir = inData.substring(J5start + 1, J5start + 2).toInt();
        int J6caldir = inData.substring(J6start + 1, J6start + 2).toInt();
        ///
        int J1step = (inData.substring(J1start + 2, J2start).toInt());
        int J2step = (inData.substring(J2start + 2, J3start).toInt());
        int J3step = (inData.substring(J3start + 2, J4start).toInt());
        int J4step = (inData.substring(J4start + 2, J5start).toInt());
        int J5step = (inData.substring(J5start + 2, J6start).toInt());
        int J6step = (inData.substring(J6start + 2).toInt());
        ///
        float SpeedIn = inData.substring(SPstart + 1).toFloat();


        //RESET COUNTERS
        int J1done = 0;
        int J2done = 0;
        int J3done = 0;
        int J4done = 0;
        int J5done = 0;
        int J6done = 0;

        String J1calStat = "0";

        //SET DIRECTIONS
        // J1 //
        if (J1rotdir == 1 && J1caldir == 1) {
          digitalWrite(J1dirPin, LOW);
        }
        else if (J1rotdir == 0 && J1caldir == 1) {
          digitalWrite(J1dirPin, HIGH);
        }
        else if (J1rotdir == 1 && J1caldir == 0) {
          digitalWrite(J1dirPin, HIGH);
        }
        else if (J1rotdir == 0 && J1caldir == 0) {
          digitalWrite(J1dirPin, LOW);
        }

        // J2 //
        if (J2rotdir == 1 && J2caldir == 1) {
          digitalWrite(J2dirPin, LOW);
        }
        else if (J2rotdir == 0 && J2caldir == 1) {
          digitalWrite(J2dirPin, HIGH);
        }
        else if (J2rotdir == 1 && J2caldir == 0) {
          digitalWrite(J2dirPin, HIGH);
        }
        else if (J2rotdir == 0 && J2caldir == 0) {
          digitalWrite(J2dirPin, LOW);
        }

        // J3 //
        if (J3rotdir == 1 && J3caldir == 1) {
          digitalWrite(J3dirPin, LOW);
        }
        else if (J3rotdir == 0 && J3caldir == 1) {
          digitalWrite(J3dirPin, HIGH);
        }
        else if (J3rotdir == 1 && J3caldir == 0) {
          digitalWrite(J3dirPin, HIGH);
        }
        else if (J3rotdir == 0 && J3caldir == 0) {
          digitalWrite(J3dirPin, LOW);
        }

        // J4 //
        if (J4rotdir == 1 && J4caldir == 1) {
          digitalWrite(J4dirPin, LOW);
        }
        else if (J4rotdir == 0 && J4caldir == 1) {
          digitalWrite(J4dirPin, HIGH);
        }
        else if (J4rotdir == 1 && J4caldir == 0) {
          digitalWrite(J4dirPin, HIGH);
        }
        else if (J4rotdir == 0 && J4caldir == 0) {
          digitalWrite(J4dirPin, LOW);
        }

        // J5 //
        if (J5rotdir == 1 && J5caldir == 1) {
          digitalWrite(J5dirPin, LOW);
        }
        else if (J5rotdir == 0 && J5caldir == 1) {
          digitalWrite(J5dirPin, HIGH);
        }
        else if (J5rotdir == 1 && J5caldir == 0) {
          digitalWrite(J5dirPin, HIGH);
        }
        else if (J5rotdir == 0 && J5caldir == 0) {
          digitalWrite(J5dirPin, LOW);
        }

        // J6 //
        if (J6rotdir == 1 && J6caldir == 1) {
          digitalWrite(J6dirPin, LOW);
        }
        else if (J6rotdir == 0 && J6caldir == 1) {
          digitalWrite(J6dirPin, HIGH);
        }
        else if (J6rotdir == 1 && J6caldir == 0) {
          digitalWrite(J6dirPin, HIGH);
        }
        else if (J6rotdir == 0 && J6caldir == 0) {
          digitalWrite(J6dirPin, LOW);
        }

        float AdjSpeed = (SpeedIn / 100);
        float CalcRegSpeed = ((SpeedMult * 2) / AdjSpeed);
        int Speed = int(CalcRegSpeed);

        //DRIVE MOTORS FOR CALIBRATION
        for (int i = 0; i <= 6; i++) {
          while (digitalRead(J1calPin) == LOW && J1done < J1step || digitalRead(J2calPin) == LOW && J2done < J2step || digitalRead(J3calPin) == LOW && J3done < J3step || digitalRead(J4calPin) == LOW && J4done < J4step || digitalRead(J5calPin) == LOW && J5done < J5step || digitalRead(J6calPin) == LOW && J6done < J6step)
          {
            if (J1done < J1step && (digitalRead(J1calPin) == LOW))
            {
              digitalWrite(J1stepPin, LOW);
            }
            delayMicroseconds(5);
            if (J1done < J1step && (digitalRead(J1calPin) == LOW))
            {
              digitalWrite(J1stepPin, HIGH);
              J1done = ++J1done;
            }
            delayMicroseconds(5);
            if (J2done < J2step && (digitalRead(J2calPin) == LOW))
            {
              digitalWrite(J2stepPin, LOW);
            }
            delayMicroseconds(5);
            if (J2done < J2step && (digitalRead(J2calPin) == LOW))
            {
              digitalWrite(J2stepPin, HIGH);
              J2done = ++J2done;
            }
            delayMicroseconds(5);
            if (J3done < J3step && (digitalRead(J3calPin) == LOW))
            {
              digitalWrite(J3stepPin, LOW);
            }
            delayMicroseconds(5);
            if (J3done < J3step && (digitalRead(J3calPin) == LOW))
            {
              digitalWrite(J3stepPin, HIGH);
              J3done = ++J3done;
            }
            delayMicroseconds(5);
            if (J4done < J4step && (digitalRead(J4calPin) == LOW))
            {
              digitalWrite(J4stepPin, LOW);
            }
            delayMicroseconds(5);
            if (J4done < J4step && (digitalRead(J4calPin) == LOW))
            {
              digitalWrite(J4stepPin, HIGH);
              J4done = ++J4done;
            }
            delayMicroseconds(5);
            if (J5done < J5step && (digitalRead(J5calPin) == LOW))
            {
              digitalWrite(J5stepPin, LOW);
            }
            delayMicroseconds(5);
            if (J5done < J5step && (digitalRead(J5calPin) == LOW))
            {
              digitalWrite(J5stepPin, HIGH);
              J5done = ++J5done;;
            }
            delayMicroseconds(5);
            if (J6done < J6step && (digitalRead(J6calPin) == LOW))
            {
              digitalWrite(J6stepPin, LOW);
            }
            delayMicroseconds(5);
            if (J6done < J6step && (digitalRead(J6calPin) == LOW))
            {
              digitalWrite(J6stepPin, HIGH);
              J6done = ++J6done;
            }
            ///////////////DELAY BEFORE RESTARTING LOOP
            delayMicroseconds(Speed);
          }
          delayMicroseconds(10);
        }
        //OVERDRIVE
        int OvrDrv = 0;
        while (OvrDrv <= 25)
        {
          if (J1step > 0)
          {
            digitalWrite(J1stepPin, LOW);
          }
          if (J2step > 0)
          {
            digitalWrite(J2stepPin, LOW);
          }
          if (J3step > 0)
          {
            digitalWrite(J3stepPin, LOW);
          }
          if (J4step > 0)
          {
            digitalWrite(J4stepPin, LOW);
          }
          if (J5step > 0)
          {
            digitalWrite(J5stepPin, LOW);
          }
          if (J6step > 0)
          {
            digitalWrite(J6stepPin, LOW);
          }
          ///////////////DELAY AND SET HIGH
          delayMicroseconds(Speed);
          if (J1step > 0)
          {
            digitalWrite(J1stepPin, HIGH);
          }
          if (J2step > 0)
          {
            digitalWrite(J2stepPin, HIGH);
          }
          if (J3step > 0)
          {
            digitalWrite(J3stepPin, HIGH);
          }
          if (J4step > 0)
          {
            digitalWrite(J4stepPin, HIGH);
          }
          if (J5step > 0)
          {
            digitalWrite(J5stepPin, HIGH);
          }
          if (J6step > 0)
          {
            digitalWrite(J6stepPin, HIGH);
          }
          OvrDrv = ++OvrDrv;
          ///////////////DELAY BEFORE RESTARTING LOOP AND SETTING LOW AGAIN
          delayMicroseconds(Speed);
        }
        //SEE IF ANY SWITCHES NOT MADE
        delay(500);
        ///
        int J1pass = 1;
        int J2pass = 1;
        int J3pass = 1;
        int J4pass = 1;
        int J5pass = 1;
        int J6pass = 1;
        ///
        if (J1step > 0) {
          if (digitalRead(J1calPin) == LOW) {
            J1pass = 0;
          }
        }
        if (J2step > 0) {
          if (digitalRead(J2calPin) == LOW) {
            J2pass = 0;
          }
        }
        if (J3step > 0) {
          if (digitalRead(J3calPin) == LOW) {
            J3pass = 0;
          }
        }
        if (J4step > 0) {
          if (digitalRead(J4calPin) == LOW) {
            J4pass = 0;
          }
        }
        if (J5step > 0)
        { if (digitalRead(J5calPin) == LOW) {
            J5pass = 0;
          }
        }
        if (J6step > 0)
        { if (digitalRead(J6calPin) == LOW) {
            J6pass = 0;
          }
        }
        if ((J1pass + J2pass + J3pass + J4pass + J5pass + J6pass) == 6)
        {
          Serial.print("P\r");
        }
        else
        {
          Serial.print("F\r");
        }
        inData = ""; // Clear recieved buffer
      }





      ///////////////////////////////////////////////////////////////////////////////////////////
      //----- MOVE J ---------------------------------------------------
      //-----------------------------------------------------------------------
      ///////////////////////////////////////////////////////////////////////////////////////////
      if (function == "MJ")
      {
        int J1curStep = J1encPos.read() / J1encMult;
        int J2curStep = J2encPos.read() / J2encMult;
        int J3curStep = J3encPos.read() / J3encMult;
        int J4curStep = J4encPos.read() / J4encMult;
        int J5curStep = J5encPos.read() / J5encMult;
        int J6curStep = J6encPos.read() / J6encMult;
        int J1start = inData.indexOf('A');
        int J2start = inData.indexOf('B');
        int J3start = inData.indexOf('C');
        int J4start = inData.indexOf('D');
        int J5start = inData.indexOf('E');
        int J6start = inData.indexOf('F');
        int TRstart = inData.indexOf('T');
        int Adstart = inData.indexOf('G');
        int Asstart = inData.indexOf('H');
        int Ddstart = inData.indexOf('I');
        int Dsstart = inData.indexOf('K');
        int SPstart = inData.indexOf('S');
        int J1Tstart = inData.indexOf('U');
        int J2Tstart = inData.indexOf('V');
        int J3Tstart = inData.indexOf('W');
        int J4Tstart = inData.indexOf('X');
        int J5Tstart = inData.indexOf('Y');
        int J6Tstart = inData.indexOf('Z');
        int J1dir = inData.substring(J1start + 1, J1start + 2).toInt();
        int J2dir = inData.substring(J2start + 1, J2start + 2).toInt();
        int J3dir = inData.substring(J3start + 1, J3start + 2).toInt();
        int J4dir = inData.substring(J4start + 1, J4start + 2).toInt();
        int J5dir = inData.substring(J5start + 1, J5start + 2).toInt();
        int J6dir = inData.substring(J6start + 1, J6start + 2).toInt();
        int TRdir = inData.substring(TRstart + 1, TRstart + 2).toInt();
        int J1step = inData.substring(J1start + 2, J2start).toInt();
        int J2step = inData.substring(J2start + 2, J3start).toInt();
        int J3step = inData.substring(J3start + 2, J4start).toInt();
        int J4step = inData.substring(J4start + 2, J5start).toInt();
        int J5step = inData.substring(J5start + 2, J6start).toInt();
        int J6step = inData.substring(J6start + 2, TRstart).toInt();
        int TRstep = inData.substring(TRstart + 2, SPstart).toInt();
        float SpeedIn = inData.substring(SPstart + 1, Adstart).toFloat();
        float ACCdur = inData.substring(Adstart + 1, Asstart).toInt();
        float ACCspd = inData.substring(Asstart + 1, Ddstart).toInt();
        float DCCdur = inData.substring(Ddstart + 1, Dsstart).toInt();
        float DCCspd = inData.substring(Dsstart + 1, J1Tstart).toInt();
        int J1tarStep = inData.substring(J1Tstart + 1, J2Tstart).toInt();
        int J2tarStep = inData.substring(J2Tstart + 1, J3Tstart).toInt();
        int J3tarStep = inData.substring(J3Tstart + 1, J4Tstart).toInt();
        int J4tarStep = inData.substring(J4Tstart + 1, J5Tstart).toInt();
        int J5tarStep = inData.substring(J5Tstart + 1, J6Tstart).toInt();
        int J6tarStep = inData.substring(J6Tstart + 1).toInt();



        //FIND HIGHEST STEP
        int HighStep = J1step;
        if (J2step > HighStep)
        {
          HighStep = J2step;
        }
        if (J3step > HighStep)
        {
          HighStep = J3step;
        }
        if (J4step > HighStep)
        {
          HighStep = J4step;
        }
        if (J5step > HighStep)
        {
          HighStep = J5step;
        }
        if (J6step > HighStep)
        {
          HighStep = J6step;
        }
        if (TRstep > HighStep)
        {
          HighStep = TRstep;
        }

        //FIND ACTIVE JOINTS
        int J1active = 0;
        int J2active = 0;
        int J3active = 0;
        int J4active = 0;
        int J5active = 0;
        int J6active = 0;
        int TRactive = 0;
        int Jactive = 0;

        if (J1step >= 1)
        {
          J1active = 1;
        }
        if (J2step >= 1)
        {
          J2active = 1;
        }
        if (J3step >= 1)
        {
          J3active = 1;
        }
        if (J4step >= 1)
        {
          J4active = 1;
        }
        if (J5step >= 1)
        {
          J5active = 1;
        }
        if (J6step >= 1)
        {
          J6active = 1;
        }
        if (TRstep >= 1)
        {
          TRactive = 1;
        }
        Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + TRactive);

        int J1_PE = 0;
        int J2_PE = 0;
        int J3_PE = 0;
        int J4_PE = 0;
        int J5_PE = 0;
        int J6_PE = 0;
        int TR_PE = 0;

        int J1_SE_1 = 0;
        int J2_SE_1 = 0;
        int J3_SE_1 = 0;
        int J4_SE_1 = 0;
        int J5_SE_1 = 0;
        int J6_SE_1 = 0;
        int TR_SE_1 = 0;

        int J1_SE_2 = 0;
        int J2_SE_2 = 0;
        int J3_SE_2 = 0;
        int J4_SE_2 = 0;
        int J5_SE_2 = 0;
        int J6_SE_2 = 0;
        int TR_SE_2 = 0;

        int J1_LO_1 = 0;
        int J2_LO_1 = 0;
        int J3_LO_1 = 0;
        int J4_LO_1 = 0;
        int J5_LO_1 = 0;
        int J6_LO_1 = 0;
        int TR_LO_1 = 0;

        int J1_LO_2 = 0;
        int J2_LO_2 = 0;
        int J3_LO_2 = 0;
        int J4_LO_2 = 0;
        int J5_LO_2 = 0;
        int J6_LO_2 = 0;
        int TR_LO_2 = 0;

        //reset
        int J1cur = 0;
        int J2cur = 0;
        int J3cur = 0;
        int J4cur = 0;
        int J5cur = 0;
        int J6cur = 0;
        int TRcur = 0;

        int J1_PEcur = 0;
        int J2_PEcur = 0;
        int J3_PEcur = 0;
        int J4_PEcur = 0;
        int J5_PEcur = 0;
        int J6_PEcur = 0;
        int TR_PEcur = 0;

        int J1_SE_1cur = 0;
        int J2_SE_1cur = 0;
        int J3_SE_1cur = 0;
        int J4_SE_1cur = 0;
        int J5_SE_1cur = 0;
        int J6_SE_1cur = 0;
        int TR_SE_1cur = 0;

        int J1_SE_2cur = 0;
        int J2_SE_2cur = 0;
        int J3_SE_2cur = 0;
        int J4_SE_2cur = 0;
        int J5_SE_2cur = 0;
        int J6_SE_2cur = 0;
        int TR_SE_2cur = 0;

        int highStepCur = 0;
        float curDelay = 0;


        //SET DIRECTIONS

        /////// J1 /////////
        if (J1dir == 1 && J1rotdir == 1)
        {
          digitalWrite(J1dirPin, LOW);
        }
        else if (J1dir == 1 && J1rotdir == 0)
        {
          digitalWrite(J1dirPin, HIGH);
        }
        else if (J1dir == 0 && J1rotdir == 1)
        {
          digitalWrite(J1dirPin, HIGH);
        }
        else if (J1dir == 0 && J1rotdir == 0)
        {
          digitalWrite(J1dirPin, LOW);
        }

        /////// J2 /////////
        if (J2dir == 1 && J2rotdir == 1)
        {
          digitalWrite(J2dirPin, LOW);
        }
        else if (J2dir == 1 && J2rotdir == 0)
        {
          digitalWrite(J2dirPin, HIGH);
        }
        else if (J2dir == 0 && J2rotdir == 1)
        {
          digitalWrite(J2dirPin, HIGH);
        }
        else if (J2dir == 0 && J2rotdir == 0)
        {
          digitalWrite(J2dirPin, LOW);
        }

        /////// J3 /////////
        if (J3dir == 1 && J3rotdir == 1)
        {
          digitalWrite(J3dirPin, LOW);
        }
        else if (J3dir == 1 && J3rotdir == 0)
        {
          digitalWrite(J3dirPin, HIGH);
        }
        else if (J3dir == 0 && J3rotdir == 1)
        {
          digitalWrite(J3dirPin, HIGH);
        }
        else if (J3dir == 0 && J3rotdir == 0)
        {
          digitalWrite(J3dirPin, LOW);
        }

        /////// J4 /////////
        if (J4dir == 1 && J4rotdir == 1)
        {
          digitalWrite(J4dirPin, LOW);
        }
        else if (J4dir == 1 && J4rotdir == 0)
        {
          digitalWrite(J4dirPin, HIGH);
        }
        else if (J4dir == 0 && J4rotdir == 1)
        {
          digitalWrite(J4dirPin, HIGH);
        }
        else if (J4dir == 0 && J4rotdir == 0)
        {
          digitalWrite(J4dirPin, LOW);
        }

        /////// J5 /////////
        if (J5dir == 1 && J5rotdir == 1)
        {
          digitalWrite(J5dirPin, LOW);
        }
        else if (J5dir == 1 && J5rotdir == 0)
        {
          digitalWrite(J5dirPin, HIGH);
        }
        else if (J5dir == 0 && J5rotdir == 1)
        {
          digitalWrite(J5dirPin, HIGH);
        }
        else if (J5dir == 0 && J5rotdir == 0)
        {
          digitalWrite(J5dirPin, LOW);
        }

        /////// J6 /////////
        if (J6dir == 1 && J6rotdir == 1)
        {
          digitalWrite(J6dirPin, LOW);
        }
        else if (J6dir == 1 && J6rotdir == 0)
        {
          digitalWrite(J6dirPin, HIGH);
        }
        else if (J6dir == 0 && J6rotdir == 1)
        {
          digitalWrite(J6dirPin, HIGH);
        }
        else if (J6dir == 0 && J6rotdir == 0)
        {
          digitalWrite(J6dirPin, LOW);
        }

        /////// TRACK /////////
        if (TRdir == 1 && TRACKrotdir == 1)
        {
          digitalWrite(TRdirPin, LOW);
        }
        else if (TRdir == 1 && TRACKrotdir == 0)
        {
          digitalWrite(TRdirPin, HIGH);
        }
        else if (TRdir == 0 && TRACKrotdir == 1)
        {
          digitalWrite(TRdirPin, HIGH);
        }
        else if (TRdir == 0 && TRACKrotdir == 0)
        {
          digitalWrite(TRdirPin, LOW);
        }



        /////CALC SPEEDS//////
        float ACCStep = (HighStep * (ACCdur / 100));
        float DCCStep = HighStep - (HighStep * (DCCdur / 100));
        float AdjSpeed = (SpeedIn / 100);
        //REG SPEED
        float CalcRegSpeed = (SpeedMult / AdjSpeed);
        int REGSpeed = int(CalcRegSpeed);

        //ACC SPEED
        float ACCspdT = (ACCspd / 100);
        float CalcACCSpeed = ((SpeedMult + (SpeedMult / ACCspdT)) / AdjSpeed);
        float ACCSpeed = (CalcACCSpeed);
        float ACCinc = (REGSpeed - ACCSpeed) / ACCStep;

        //DCC SPEED
        float DCCspdT = (DCCspd / 100);
        float CalcDCCSpeed = ((SpeedMult + (SpeedMult / DCCspdT)) / AdjSpeed);
        float DCCSpeed = (CalcDCCSpeed);
        float DCCinc = (REGSpeed + DCCSpeed) / DCCStep;
        DCCSpeed = REGSpeed;




        ///// DRIVE MOTORS /////
        while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step || TRcur < TRstep)
          //while (J1curStep < J1tarStep || J1curStep != J1tarStep)
        {

          ////DELAY CALC/////
          if (highStepCur <= ACCStep)
          {
            curDelay = (ACCSpeed / Jactive);
            ACCSpeed = ACCSpeed + ACCinc;
          }
          else if (highStepCur >= DCCStep)
          {
            curDelay = (DCCSpeed / Jactive);
            DCCSpeed = DCCSpeed + DCCinc;
          }
          else
          {
            curDelay = (REGSpeed / Jactive);
          }

          /////// J1 ////////////////////////////////
          ///find pulse every
          if (J1cur < J1step)
          {
            J1_PE = (HighStep / J1step);
            ///find left over 1
            J1_LO_1 = (HighStep - (J1step * J1_PE));
            ///find skip 1
            if (J1_LO_1 > 0)
            {
              J1_SE_1 = (HighStep / J1_LO_1);
            }
            else
            {
              J1_SE_1 = 0;
            }
            ///find left over 2
            if (J1_SE_1 > 0)
            {
              J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
            }
            else
            {
              J1_LO_2 = 0;
            }
            ///find skip 2
            if (J1_LO_2 > 0)
            {
              J1_SE_2 = (HighStep / J1_LO_2);
            }
            else
            {
              J1_SE_2 = 0;
            }
            /////////  J1  ///////////////
            if (J1_SE_2 == 0)
            {
              J1_SE_2cur = (J1_SE_2 + 1);
            }
            if (J1_SE_2cur != J1_SE_2)
            {
              J1_SE_2cur = ++J1_SE_2cur;
              if (J1_SE_1 == 0)
              {
                J1_SE_1cur = (J1_SE_1 + 1);
              }
              if (J1_SE_1cur != J1_SE_1)
              {
                J1_SE_1cur = ++J1_SE_1cur;
                J1_PEcur = ++J1_PEcur;
                if (J1_PEcur == J1_PE)
                {
                  J1cur = ++J1cur;
                  J1_PEcur = 0;
                  digitalWrite(J1stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J1stepPin, HIGH);
                }
              }
              else
              {
                J1_SE_1cur = 0;
              }
            }
            else
            {
              J1_SE_2cur = 0;
            }
          }

          /////// J2 ////////////////////////////////
          ///find pulse every
          if (J2cur < J2step)
          {
            J2_PE = (HighStep / J2step);
            ///find left over 1
            J2_LO_1 = (HighStep - (J2step * J2_PE));
            ///find skip 1
            if (J2_LO_1 > 0)
            {
              J2_SE_1 = (HighStep / J2_LO_1);
            }
            else
            {
              J2_SE_1 = 0;
            }
            ///find left over 2
            if (J2_SE_1 > 0)
            {
              J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
            }
            else
            {
              J2_LO_2 = 0;
            }
            ///find skip 2
            if (J2_LO_2 > 0)
            {
              J2_SE_2 = (HighStep / J2_LO_2);
            }
            else
            {
              J2_SE_2 = 0;
            }
            /////////  J2  ///////////////
            if (J2_SE_2 == 0)
            {
              J2_SE_2cur = (J2_SE_2 + 1);
            }
            if (J2_SE_2cur != J2_SE_2)
            {
              J2_SE_2cur = ++J2_SE_2cur;
              if (J2_SE_1 == 0)
              {
                J2_SE_1cur = (J2_SE_1 + 1);
              }
              if (J2_SE_1cur != J2_SE_1)
              {
                J2_SE_1cur = ++J2_SE_1cur;
                J2_PEcur = ++J2_PEcur;
                if (J2_PEcur == J2_PE)
                {
                  J2cur = ++J2cur;
                  J2_PEcur = 0;
                  digitalWrite(J2stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J2stepPin, HIGH);
                }
              }
              else
              {
                J2_SE_1cur = 0;
              }
            }
            else
            {
              J2_SE_2cur = 0;
            }
          }

          /////// J3 ////////////////////////////////
          ///find pulse every
          if (J3cur < J3step)
          {
            J3_PE = (HighStep / J3step);
            ///find left over 1
            J3_LO_1 = (HighStep - (J3step * J3_PE));
            ///find skip 1
            if (J3_LO_1 > 0)
            {
              J3_SE_1 = (HighStep / J3_LO_1);
            }
            else
            {
              J3_SE_1 = 0;
            }
            ///find left over 2
            if (J3_SE_1 > 0)
            {
              J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
            }
            else
            {
              J3_LO_2 = 0;
            }
            ///find skip 2
            if (J3_LO_2 > 0)
            {
              J3_SE_2 = (HighStep / J3_LO_2);
            }
            else
            {
              J3_SE_2 = 0;
            }
            /////////  J3  ///////////////
            if (J3_SE_2 == 0)
            {
              J3_SE_2cur = (J3_SE_2 + 1);
            }
            if (J3_SE_2cur != J3_SE_2)
            {
              J3_SE_2cur = ++J3_SE_2cur;
              if (J3_SE_1 == 0)
              {
                J3_SE_1cur = (J3_SE_1 + 1);
              }
              if (J3_SE_1cur != J3_SE_1)
              {
                J3_SE_1cur = ++J3_SE_1cur;
                J3_PEcur = ++J3_PEcur;
                if (J3_PEcur == J3_PE)
                {
                  J3cur = ++J3cur;
                  J3_PEcur = 0;
                  digitalWrite(J3stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J3stepPin, HIGH);
                }
              }
              else
              {
                J3_SE_1cur = 0;
              }
            }
            else
            {
              J3_SE_2cur = 0;
            }
          }

          /////// J4 ////////////////////////////////
          ///find pulse every
          if (J4cur < J4step)
          {
            J4_PE = (HighStep / J4step);
            ///find left over 1
            J4_LO_1 = (HighStep - (J4step * J4_PE));
            ///find skip 1
            if (J4_LO_1 > 0)
            {
              J4_SE_1 = (HighStep / J4_LO_1);
            }
            else
            {
              J4_SE_1 = 0;
            }
            ///find left over 2
            if (J4_SE_1 > 0)
            {
              J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
            }
            else
            {
              J4_LO_2 = 0;
            }
            ///find skip 2
            if (J4_LO_2 > 0)
            {
              J4_SE_2 = (HighStep / J4_LO_2);
            }
            else
            {
              J4_SE_2 = 0;
            }
            /////////  J4  ///////////////
            if (J4_SE_2 == 0)
            {
              J4_SE_2cur = (J4_SE_2 + 1);
            }
            if (J4_SE_2cur != J4_SE_2)
            {
              J4_SE_2cur = ++J4_SE_2cur;
              if (J4_SE_1 == 0)
              {
                J4_SE_1cur = (J4_SE_1 + 1);
              }
              if (J4_SE_1cur != J4_SE_1)
              {
                J4_SE_1cur = ++J4_SE_1cur;
                J4_PEcur = ++J4_PEcur;
                if (J4_PEcur == J4_PE)
                {
                  J4cur = ++J4cur;
                  J4_PEcur = 0;
                  digitalWrite(J4stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J4stepPin, HIGH);
                }
              }
              else
              {
                J4_SE_1cur = 0;
              }
            }
            else
            {
              J4_SE_2cur = 0;
            }
          }

          /////// J5 ////////////////////////////////
          ///find pulse every
          if (J5cur < J5step)
          {
            J5_PE = (HighStep / J5step);
            ///find left over 1
            J5_LO_1 = (HighStep - (J5step * J5_PE));
            ///find skip 1
            if (J5_LO_1 > 0)
            {
              J5_SE_1 = (HighStep / J5_LO_1);
            }
            else
            {
              J5_SE_1 = 0;
            }
            ///find left over 2
            if (J5_SE_1 > 0)
            {
              J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
            }
            else
            {
              J5_LO_2 = 0;
            }
            ///find skip 2
            if (J5_LO_2 > 0)
            {
              J5_SE_2 = (HighStep / J5_LO_2);
            }
            else
            {
              J5_SE_2 = 0;
            }
            /////////  J5  ///////////////
            if (J5_SE_2 == 0)
            {
              J5_SE_2cur = (J5_SE_2 + 1);
            }
            if (J5_SE_2cur != J5_SE_2)
            {
              J5_SE_2cur = ++J5_SE_2cur;
              if (J5_SE_1 == 0)
              {
                J5_SE_1cur = (J5_SE_1 + 1);
              }
              if (J5_SE_1cur != J5_SE_1)
              {
                J5_SE_1cur = ++J5_SE_1cur;
                J5_PEcur = ++J5_PEcur;
                if (J5_PEcur == J5_PE)
                {
                  J5cur = ++J5cur;
                  J5_PEcur = 0;
                  digitalWrite(J5stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J5stepPin, HIGH);
                }
              }
              else
              {
                J5_SE_1cur = 0;
              }
            }
            else
            {
              J5_SE_2cur = 0;
            }
          }

          /////// J6 ////////////////////////////////
          ///find pulse every
          if (J6cur < J6step)
          {
            J6_PE = (HighStep / J6step);
            ///find left over 1
            J6_LO_1 = (HighStep - (J6step * J6_PE));
            ///find skip 1
            if (J6_LO_1 > 0)
            {
              J6_SE_1 = (HighStep / J6_LO_1);
            }
            else
            {
              J6_SE_1 = 0;
            }
            ///find left over 2
            if (J6_SE_1 > 0)
            {
              J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
            }
            else
            {
              J6_LO_2 = 0;
            }
            ///find skip 2
            if (J6_LO_2 > 0)
            {
              J6_SE_2 = (HighStep / J6_LO_2);
            }
            else
            {
              J6_SE_2 = 0;
            }
            /////////  J6  ///////////////
            if (J6_SE_2 == 0)
            {
              J6_SE_2cur = (J6_SE_2 + 1);
            }
            if (J6_SE_2cur != J6_SE_2)
            {
              J6_SE_2cur = ++J6_SE_2cur;
              if (J6_SE_1 == 0)
              {
                J6_SE_1cur = (J6_SE_1 + 1);
              }
              if (J6_SE_1cur != J6_SE_1)
              {
                J6_SE_1cur = ++J6_SE_1cur;
                J6_PEcur = ++J6_PEcur;
                if (J6_PEcur == J6_PE)
                {
                  J6cur = ++J6cur;
                  J6_PEcur = 0;
                  digitalWrite(J6stepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(J6stepPin, HIGH);
                }
              }
              else
              {
                J6_SE_1cur = 0;
              }
            }
            else
            {
              J6_SE_2cur = 0;
            }
          }

          /////// TR ////////////////////////////////
          ///find pulse every
          if (TRcur < TRstep)
          {
            TR_PE = (HighStep / TRstep);
            ///find left over 1
            TR_LO_1 = (HighStep - (TRstep * TR_PE));
            ///find skip 1
            if (TR_LO_1 > 0)
            {
              TR_SE_1 = (HighStep / TR_LO_1);
            }
            else
            {
              TR_SE_1 = 0;
            }
            ///find left over 2
            if (TR_SE_1 > 0)
            {
              TR_LO_2 = HighStep - ((TRstep * TR_PE) + ((TRstep * TR_PE) / TR_SE_1));
            }
            else
            {
              TR_LO_2 = 0;
            }
            ///find skip 2
            if (TR_LO_2 > 0)
            {
              TR_SE_2 = (HighStep / TR_LO_2);
            }
            else
            {
              TR_SE_2 = 0;
            }
            /////////  TR  ///////////////
            if (TR_SE_2 == 0)
            {
              TR_SE_2cur = (TR_SE_2 + 1);
            }
            if (TR_SE_2cur != TR_SE_2)
            {
              TR_SE_2cur = ++TR_SE_2cur;
              if (TR_SE_1 == 0)
              {
                TR_SE_1cur = (TR_SE_1 + 1);
              }
              if (TR_SE_1cur != TR_SE_1)
              {
                TR_SE_1cur = ++TR_SE_1cur;
                TR_PEcur = ++TR_PEcur;
                if (TR_PEcur == TR_PE)
                {
                  TRcur = ++TRcur;
                  TR_PEcur = 0;
                  digitalWrite(TRstepPin, LOW);
                  delayMicroseconds(curDelay);
                  digitalWrite(TRstepPin, HIGH);
                }
              }
              else
              {
                TR_SE_1cur = 0;
              }
            }
            else
            {
              TR_SE_2cur = 0;
            }
          }


          // inc cur step
          highStepCur = ++highStepCur;
          delayMicroseconds(200);


        }

        ////////// check for stalled motor
        int ErrorTrue = 0;
        String ErrorCode = "00";
        String J1error = "0";
        String J2error = "0";
        String J3error = "0";
        String J4error = "0";
        String J5error = "0";
        String J6error = "0";

        J1curStep = J1encPos.read() / J1encMult;
        J2curStep = J2encPos.read() / J2encMult;
        J3curStep = J3encPos.read() / J3encMult;
        J4curStep = J4encPos.read() / J4encMult;
        J5curStep = J5encPos.read() / J5encMult;
        J6curStep = J6encPos.read() / J6encMult;

        if (abs(J1curStep - J1tarStep) >  J1encMult / EncDiv)
        {
          J1error = "1";
          ErrorTrue = 1;
        }
        if (abs(J2curStep - J2tarStep) >  J2encMult / EncDiv)
        {
          J2error = "1";
          ErrorTrue = 1;
        }
        if (abs(J3curStep - J3tarStep) >  J3encMult / EncDiv)
        {
          J3error = "1";
          ErrorTrue = 1;
        }
        if (abs(J4curStep - J4tarStep) >  J4encMult / EncDiv)
        {
          J4error = "1";
          ErrorTrue = 1;
        }
        if (abs(J5curStep - J5tarStep) >  J5encMult / EncDiv)
        {
          J5error = "1";
          ErrorTrue = 1;
        }
        if (abs(J6curStep - J6tarStep) >  J6encMult / EncDiv)
        {
          J6error = "1";
          ErrorTrue = 1;
        }

        if (ErrorTrue == 1)
        {
          ErrorCode = "01" + J1error + J2error + J3error + J4error + J5error + J6error + "A" + String(J1curStep) + "B" + String(J2curStep) + "C" + String(J3curStep) + "D" + String(J4curStep) + "E" + String(J5curStep) + "F" + String(J6curStep);
        }
        else if (ErrorTrue == 0)
        {
          J1encPos.write(J1tarStep * J1encMult);
          J2encPos.write(J2tarStep * J2encMult);
          J3encPos.write(J3tarStep * J3encMult);
          J4encPos.write(J4tarStep * J4encMult);
          J5encPos.write(J5tarStep * J5encMult);
          J6encPos.write(J6tarStep * J6encMult);
        }


        Serial.print(ErrorCode);
        Serial.println();
        ////////MOVE COMPLETE///////////
      }
      
      /// END OF MOVE J ///





      ///////////////////////////////////////////////////////////////////////////////////////////
      //----- MOVE L ---------------------------------------------------
      //-----------------------------------------------------------------------
      ///////////////////////////////////////////////////////////////////////////////////////////
      if (function == "ML")
      {
        WayPtDel = 1;
        int NumPtsStart = inData.indexOf('L');
        int WayPts = inData.substring(NumPtsStart + 1).toInt();
        Serial.println();
        inData = ""; // Clear recieved buffer
        //STORE WAYPOINTS
        int i = 0;
        while (i < WayPts) {
          while (Serial.available() > 0) {
            char recieved = Serial.read();
            inData += recieved;
            if (recieved == '\n') {
              inData.toCharArray(WayPt[i], 70);
              Serial.println();
              ++i;
              inData = ""; // Clear recieved buffer
            }
          }
        }
        Serial.println();
        //EXECUTE WAYPOINTS
        int ErrorTrue = 0;
        String ErrorCode = "00";
        String J1error = "0";
        String J2error = "0";
        String J3error = "0";
        String J4error = "0";
        String J5error = "0";
        String J6error = "0";
        int J1tarStep = 0;
        int J2tarStep = 0;
        int J3tarStep = 0;
        int J4tarStep = 0;
        int J5tarStep = 0;
        int J6tarStep = 0;

        i = 0;
        while (i <= WayPts+1) {
          inData = WayPt[i];
          int J1start = inData.indexOf('A');
          int J2start = inData.indexOf('B');
          int J3start = inData.indexOf('C');
          int J4start = inData.indexOf('D');
          int J5start = inData.indexOf('E');
          int J6start = inData.indexOf('F');
          int TRstart = inData.indexOf('T');
          int Adstart = inData.indexOf('G');
          int Asstart = inData.indexOf('H');
          int Ddstart = inData.indexOf('I');
          int Dsstart = inData.indexOf('K');
          int SPstart = inData.indexOf('S');
          int J1Tstart = inData.indexOf('U');
          int J2Tstart = inData.indexOf('V');
          int J3Tstart = inData.indexOf('W');
          int J4Tstart = inData.indexOf('X');
          int J5Tstart = inData.indexOf('Y');
          int J6Tstart = inData.indexOf('Z');
          int J1dir = inData.substring(J1start + 1, J1start + 2).toInt();
          int J2dir = inData.substring(J2start + 1, J2start + 2).toInt();
          int J3dir = inData.substring(J3start + 1, J3start + 2).toInt();
          int J4dir = inData.substring(J4start + 1, J4start + 2).toInt();
          int J5dir = inData.substring(J5start + 1, J5start + 2).toInt();
          int J6dir = inData.substring(J6start + 1, J6start + 2).toInt();
          int TRdir = inData.substring(TRstart + 1, TRstart + 2).toInt();
          int J1step = inData.substring(J1start + 2, J2start).toInt();
          int J2step = inData.substring(J2start + 2, J3start).toInt();
          int J3step = inData.substring(J3start + 2, J4start).toInt();
          int J4step = inData.substring(J4start + 2, J5start).toInt();
          int J5step = inData.substring(J5start + 2, J6start).toInt();
          int J6step = inData.substring(J6start + 2, TRstart).toInt();
          int TRstep = inData.substring(TRstart + 2, SPstart).toInt();
          float SpeedIn = inData.substring(SPstart + 1, Adstart).toFloat();
          float ACCdur = inData.substring(Adstart + 1, Asstart).toInt();
          float ACCspd = inData.substring(Asstart + 1, Ddstart).toInt();
          float DCCdur = inData.substring(Ddstart + 1, Dsstart).toInt();
          float DCCspd = inData.substring(Dsstart + 1, J1Tstart).toInt();
          int J1tarStep = inData.substring(J1Tstart + 1, J2Tstart).toInt();
          int J2tarStep = inData.substring(J2Tstart + 1, J3Tstart).toInt();
          int J3tarStep = inData.substring(J3Tstart + 1, J4Tstart).toInt();
          int J4tarStep = inData.substring(J4Tstart + 1, J5Tstart).toInt();
          int J5tarStep = inData.substring(J5Tstart + 1, J6Tstart).toInt();
          int J6tarStep = inData.substring(J6Tstart + 1).toInt();


          //FIND HIGHEST STEP
          int HighStep = J1step;
          if (J2step > HighStep)
          {
            HighStep = J2step;
          }
          if (J3step > HighStep)
          {
            HighStep = J3step;
          }
          if (J4step > HighStep)
          {
            HighStep = J4step;
          }
          if (J5step > HighStep)
          {
            HighStep = J5step;
          }
          if (J6step > HighStep)
          {
            HighStep = J6step;
          }
          if (TRstep > HighStep)
          {
            HighStep = TRstep;
          }

          //FIND ACTIVE JOINTS
          int J1active = 0;
          int J2active = 0;
          int J3active = 0;
          int J4active = 0;
          int J5active = 0;
          int J6active = 0;
          int TRactive = 0;
          int Jactive = 0;

          if (J1step >= 1)
          {
            J1active = 1;
          }
          if (J2step >= 1)
          {
            J2active = 1;
          }
          if (J3step >= 1)
          {
            J3active = 1;
          }
          if (J4step >= 1)
          {
            J4active = 1;
          }
          if (J5step >= 1)
          {
            J5active = 1;
          }
          if (J6step >= 1)
          {
            J6active = 1;
          }
          if (TRstep >= 1)
          {
            TRactive = 1;
          }
          Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + TRactive);

          int J1_PE = 0;
          int J2_PE = 0;
          int J3_PE = 0;
          int J4_PE = 0;
          int J5_PE = 0;
          int J6_PE = 0;
          int TR_PE = 0;

          int J1_SE_1 = 0;
          int J2_SE_1 = 0;
          int J3_SE_1 = 0;
          int J4_SE_1 = 0;
          int J5_SE_1 = 0;
          int J6_SE_1 = 0;
          int TR_SE_1 = 0;

          int J1_SE_2 = 0;
          int J2_SE_2 = 0;
          int J3_SE_2 = 0;
          int J4_SE_2 = 0;
          int J5_SE_2 = 0;
          int J6_SE_2 = 0;
          int TR_SE_2 = 0;

          int J1_LO_1 = 0;
          int J2_LO_1 = 0;
          int J3_LO_1 = 0;
          int J4_LO_1 = 0;
          int J5_LO_1 = 0;
          int J6_LO_1 = 0;
          int TR_LO_1 = 0;

          int J1_LO_2 = 0;
          int J2_LO_2 = 0;
          int J3_LO_2 = 0;
          int J4_LO_2 = 0;
          int J5_LO_2 = 0;
          int J6_LO_2 = 0;
          int TR_LO_2 = 0;

          //reset
          int J1cur = 0;
          int J2cur = 0;
          int J3cur = 0;
          int J4cur = 0;
          int J5cur = 0;
          int J6cur = 0;
          int TRcur = 0;

          int J1_PEcur = 0;
          int J2_PEcur = 0;
          int J3_PEcur = 0;
          int J4_PEcur = 0;
          int J5_PEcur = 0;
          int J6_PEcur = 0;
          int TR_PEcur = 0;

          int J1_SE_1cur = 0;
          int J2_SE_1cur = 0;
          int J3_SE_1cur = 0;
          int J4_SE_1cur = 0;
          int J5_SE_1cur = 0;
          int J6_SE_1cur = 0;
          int TR_SE_1cur = 0;

          int J1_SE_2cur = 0;
          int J2_SE_2cur = 0;
          int J3_SE_2cur = 0;
          int J4_SE_2cur = 0;
          int J5_SE_2cur = 0;
          int J6_SE_2cur = 0;
          int TR_SE_2cur = 0;

          int highStepCur = 0;
          float curDelay = 0;


          //SET DIRECTIONS

          /////// J1 /////////
          if (J1dir == 1 && J1rotdir == 1)
          {
            digitalWrite(J1dirPin, LOW);
          }
          else if (J1dir == 1 && J1rotdir == 0)
          {
            digitalWrite(J1dirPin, HIGH);
          }
          else if (J1dir == 0 && J1rotdir == 1)
          {
            digitalWrite(J1dirPin, HIGH);
          }
          else if (J1dir == 0 && J1rotdir == 0)
          {
            digitalWrite(J1dirPin, LOW);
          }

          /////// J2 /////////
          if (J2dir == 1 && J2rotdir == 1)
          {
            digitalWrite(J2dirPin, LOW);
          }
          else if (J2dir == 1 && J2rotdir == 0)
          {
            digitalWrite(J2dirPin, HIGH);
          }
          else if (J2dir == 0 && J2rotdir == 1)
          {
            digitalWrite(J2dirPin, HIGH);
          }
          else if (J2dir == 0 && J2rotdir == 0)
          {
            digitalWrite(J2dirPin, LOW);
          }

          /////// J3 /////////
          if (J3dir == 1 && J3rotdir == 1)
          {
            digitalWrite(J3dirPin, LOW);
          }
          else if (J3dir == 1 && J3rotdir == 0)
          {
            digitalWrite(J3dirPin, HIGH);
          }
          else if (J3dir == 0 && J3rotdir == 1)
          {
            digitalWrite(J3dirPin, HIGH);
          }
          else if (J3dir == 0 && J3rotdir == 0)
          {
            digitalWrite(J3dirPin, LOW);
          }

          /////// J4 /////////
          if (J4dir == 1 && J4rotdir == 1)
          {
            digitalWrite(J4dirPin, LOW);
          }
          else if (J4dir == 1 && J4rotdir == 0)
          {
            digitalWrite(J4dirPin, HIGH);
          }
          else if (J4dir == 0 && J4rotdir == 1)
          {
            digitalWrite(J4dirPin, HIGH);
          }
          else if (J4dir == 0 && J4rotdir == 0)
          {
            digitalWrite(J4dirPin, LOW);
          }

          /////// J5 /////////
          if (J5dir == 1 && J5rotdir == 1)
          {
            digitalWrite(J5dirPin, LOW);
          }
          else if (J5dir == 1 && J5rotdir == 0)
          {
            digitalWrite(J5dirPin, HIGH);
          }
          else if (J5dir == 0 && J5rotdir == 1)
          {
            digitalWrite(J5dirPin, HIGH);
          }
          else if (J5dir == 0 && J5rotdir == 0)
          {
            digitalWrite(J5dirPin, LOW);
          }

          /////// J6 /////////
          if (J6dir == 1 && J6rotdir == 1)
          {
            digitalWrite(J6dirPin, LOW);
          }
          else if (J6dir == 1 && J6rotdir == 0)
          {
            digitalWrite(J6dirPin, HIGH);
          }
          else if (J6dir == 0 && J6rotdir == 1)
          {
            digitalWrite(J6dirPin, HIGH);
          }
          else if (J6dir == 0 && J6rotdir == 0)
          {
            digitalWrite(J6dirPin, LOW);
          }

          /////// TRACK /////////
          if (TRdir == 1 && TRACKrotdir == 1)
          {
            digitalWrite(TRdirPin, LOW);
          }
          else if (TRdir == 1 && TRACKrotdir == 0)
          {
            digitalWrite(TRdirPin, HIGH);
          }
          else if (TRdir == 0 && TRACKrotdir == 1)
          {
            digitalWrite(TRdirPin, HIGH);
          }
          else if (TRdir == 0 && TRACKrotdir == 0)
          {
            digitalWrite(TRdirPin, LOW);
          }


          /////CALC SPEEDS//////
          float AdjSpeed = (SpeedIn / 100);
          //REG SPEED
          float LspeedAdj = 3;
          float CalcRegSpeed = ((SpeedMult * LspeedAdj) / AdjSpeed);
          curDelay = int(CalcRegSpeed) / Jactive;

          //REG SPEED
          //float AdjSpeed = (SpeedIn / 100);
          //float CalcRegSpeed = (SpeedMult / AdjSpeed);
          //int REGSpeed = int(CalcRegSpeed);
          //curDelay = (REGSpeed / Jactive);



          ///// DRIVE MOTORS /////
          while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step || TRcur < TRstep)
            //while (J1curStep < J1tarStep || J1curStep != J1tarStep)
          {

            /////// J1 ////////////////////////////////
            ///find pulse every
            if (J1cur < J1step)
            {
              J1_PE = (HighStep / J1step);
              ///find left over 1
              J1_LO_1 = (HighStep - (J1step * J1_PE));
              ///find skip 1
              if (J1_LO_1 > 0)
              {
                J1_SE_1 = (HighStep / J1_LO_1);
              }
              else
              {
                J1_SE_1 = 0;
              }
              ///find left over 2
              if (J1_SE_1 > 0)
              {
                J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
              }
              else
              {
                J1_LO_2 = 0;
              }
              ///find skip 2
              if (J1_LO_2 > 0)
              {
                J1_SE_2 = (HighStep / J1_LO_2);
              }
              else
              {
                J1_SE_2 = 0;
              }
              /////////  J1  ///////////////
              if (J1_SE_2 == 0)
              {
                J1_SE_2cur = (J1_SE_2 + 1);
              }
              if (J1_SE_2cur != J1_SE_2)
              {
                J1_SE_2cur = ++J1_SE_2cur;
                if (J1_SE_1 == 0)
                {
                  J1_SE_1cur = (J1_SE_1 + 1);
                }
                if (J1_SE_1cur != J1_SE_1)
                {
                  J1_SE_1cur = ++J1_SE_1cur;
                  J1_PEcur = ++J1_PEcur;
                  if (J1_PEcur == J1_PE)
                  {
                    J1cur = ++J1cur;
                    J1_PEcur = 0;
                    digitalWrite(J1stepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(J1stepPin, HIGH);
                  }
                }
                else
                {
                  J1_SE_1cur = 0;
                }
              }
              else
              {
                J1_SE_2cur = 0;
              }
            }

            /////// J2 ////////////////////////////////
            ///find pulse every
            if (J2cur < J2step)
            {
              J2_PE = (HighStep / J2step);
              ///find left over 1
              J2_LO_1 = (HighStep - (J2step * J2_PE));
              ///find skip 1
              if (J2_LO_1 > 0)
              {
                J2_SE_1 = (HighStep / J2_LO_1);
              }
              else
              {
                J2_SE_1 = 0;
              }
              ///find left over 2
              if (J2_SE_1 > 0)
              {
                J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
              }
              else
              {
                J2_LO_2 = 0;
              }
              ///find skip 2
              if (J2_LO_2 > 0)
              {
                J2_SE_2 = (HighStep / J2_LO_2);
              }
              else
              {
                J2_SE_2 = 0;
              }
              /////////  J2  ///////////////
              if (J2_SE_2 == 0)
              {
                J2_SE_2cur = (J2_SE_2 + 1);
              }
              if (J2_SE_2cur != J2_SE_2)
              {
                J2_SE_2cur = ++J2_SE_2cur;
                if (J2_SE_1 == 0)
                {
                  J2_SE_1cur = (J2_SE_1 + 1);
                }
                if (J2_SE_1cur != J2_SE_1)
                {
                  J2_SE_1cur = ++J2_SE_1cur;
                  J2_PEcur = ++J2_PEcur;
                  if (J2_PEcur == J2_PE)
                  {
                    J2cur = ++J2cur;
                    J2_PEcur = 0;
                    digitalWrite(J2stepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(J2stepPin, HIGH);
                  }
                }
                else
                {
                  J2_SE_1cur = 0;
                }
              }
              else
              {
                J2_SE_2cur = 0;
              }
            }

            /////// J3 ////////////////////////////////
            ///find pulse every
            if (J3cur < J3step)
            {
              J3_PE = (HighStep / J3step);
              ///find left over 1
              J3_LO_1 = (HighStep - (J3step * J3_PE));
              ///find skip 1
              if (J3_LO_1 > 0)
              {
                J3_SE_1 = (HighStep / J3_LO_1);
              }
              else
              {
                J3_SE_1 = 0;
              }
              ///find left over 2
              if (J3_SE_1 > 0)
              {
                J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
              }
              else
              {
                J3_LO_2 = 0;
              }
              ///find skip 2
              if (J3_LO_2 > 0)
              {
                J3_SE_2 = (HighStep / J3_LO_2);
              }
              else
              {
                J3_SE_2 = 0;
              }
              /////////  J3  ///////////////
              if (J3_SE_2 == 0)
              {
                J3_SE_2cur = (J3_SE_2 + 1);
              }
              if (J3_SE_2cur != J3_SE_2)
              {
                J3_SE_2cur = ++J3_SE_2cur;
                if (J3_SE_1 == 0)
                {
                  J3_SE_1cur = (J3_SE_1 + 1);
                }
                if (J3_SE_1cur != J3_SE_1)
                {
                  J3_SE_1cur = ++J3_SE_1cur;
                  J3_PEcur = ++J3_PEcur;
                  if (J3_PEcur == J3_PE)
                  {
                    J3cur = ++J3cur;
                    J3_PEcur = 0;
                    digitalWrite(J3stepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(J3stepPin, HIGH);
                  }
                }
                else
                {
                  J3_SE_1cur = 0;
                }
              }
              else
              {
                J3_SE_2cur = 0;
              }
            }

            /////// J4 ////////////////////////////////
            ///find pulse every
            if (J4cur < J4step)
            {
              J4_PE = (HighStep / J4step);
              ///find left over 1
              J4_LO_1 = (HighStep - (J4step * J4_PE));
              ///find skip 1
              if (J4_LO_1 > 0)
              {
                J4_SE_1 = (HighStep / J4_LO_1);
              }
              else
              {
                J4_SE_1 = 0;
              }
              ///find left over 2
              if (J4_SE_1 > 0)
              {
                J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
              }
              else
              {
                J4_LO_2 = 0;
              }
              ///find skip 2
              if (J4_LO_2 > 0)
              {
                J4_SE_2 = (HighStep / J4_LO_2);
              }
              else
              {
                J4_SE_2 = 0;
              }
              /////////  J4  ///////////////
              if (J4_SE_2 == 0)
              {
                J4_SE_2cur = (J4_SE_2 + 1);
              }
              if (J4_SE_2cur != J4_SE_2)
              {
                J4_SE_2cur = ++J4_SE_2cur;
                if (J4_SE_1 == 0)
                {
                  J4_SE_1cur = (J4_SE_1 + 1);
                }
                if (J4_SE_1cur != J4_SE_1)
                {
                  J4_SE_1cur = ++J4_SE_1cur;
                  J4_PEcur = ++J4_PEcur;
                  if (J4_PEcur == J4_PE)
                  {
                    J4cur = ++J4cur;
                    J4_PEcur = 0;
                    digitalWrite(J4stepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(J4stepPin, HIGH);
                  }
                }
                else
                {
                  J4_SE_1cur = 0;
                }
              }
              else
              {
                J4_SE_2cur = 0;
              }
            }

            /////// J5 ////////////////////////////////
            ///find pulse every
            if (J5cur < J5step)
            {
              J5_PE = (HighStep / J5step);
              ///find left over 1
              J5_LO_1 = (HighStep - (J5step * J5_PE));
              ///find skip 1
              if (J5_LO_1 > 0)
              {
                J5_SE_1 = (HighStep / J5_LO_1);
              }
              else
              {
                J5_SE_1 = 0;
              }
              ///find left over 2
              if (J5_SE_1 > 0)
              {
                J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
              }
              else
              {
                J5_LO_2 = 0;
              }
              ///find skip 2
              if (J5_LO_2 > 0)
              {
                J5_SE_2 = (HighStep / J5_LO_2);
              }
              else
              {
                J5_SE_2 = 0;
              }
              /////////  J5  ///////////////
              if (J5_SE_2 == 0)
              {
                J5_SE_2cur = (J5_SE_2 + 1);
              }
              if (J5_SE_2cur != J5_SE_2)
              {
                J5_SE_2cur = ++J5_SE_2cur;
                if (J5_SE_1 == 0)
                {
                  J5_SE_1cur = (J5_SE_1 + 1);
                }
                if (J5_SE_1cur != J5_SE_1)
                {
                  J5_SE_1cur = ++J5_SE_1cur;
                  J5_PEcur = ++J5_PEcur;
                  if (J5_PEcur == J5_PE)
                  {
                    J5cur = ++J5cur;
                    J5_PEcur = 0;
                    digitalWrite(J5stepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(J5stepPin, HIGH);
                  }
                }
                else
                {
                  J5_SE_1cur = 0;
                }
              }
              else
              {
                J5_SE_2cur = 0;
              }
            }

            /////// J6 ////////////////////////////////
            ///find pulse every
            if (J6cur < J6step)
            {
              J6_PE = (HighStep / J6step);
              ///find left over 1
              J6_LO_1 = (HighStep - (J6step * J6_PE));
              ///find skip 1
              if (J6_LO_1 > 0)
              {
                J6_SE_1 = (HighStep / J6_LO_1);
              }
              else
              {
                J6_SE_1 = 0;
              }
              ///find left over 2
              if (J6_SE_1 > 0)
              {
                J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
              }
              else
              {
                J6_LO_2 = 0;
              }
              ///find skip 2
              if (J6_LO_2 > 0)
              {
                J6_SE_2 = (HighStep / J6_LO_2);
              }
              else
              {
                J6_SE_2 = 0;
              }
              /////////  J6  ///////////////
              if (J6_SE_2 == 0)
              {
                J6_SE_2cur = (J6_SE_2 + 1);
              }
              if (J6_SE_2cur != J6_SE_2)
              {
                J6_SE_2cur = ++J6_SE_2cur;
                if (J6_SE_1 == 0)
                {
                  J6_SE_1cur = (J6_SE_1 + 1);
                }
                if (J6_SE_1cur != J6_SE_1)
                {
                  J6_SE_1cur = ++J6_SE_1cur;
                  J6_PEcur = ++J6_PEcur;
                  if (J6_PEcur == J6_PE)
                  {
                    J6cur = ++J6cur;
                    J6_PEcur = 0;
                    digitalWrite(J6stepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(J6stepPin, HIGH);
                  }
                }
                else
                {
                  J6_SE_1cur = 0;
                }
              }
              else
              {
                J6_SE_2cur = 0;
              }
            }

            /////// TR ////////////////////////////////
            ///find pulse every
            if (TRcur < TRstep)
            {
              TR_PE = (HighStep / TRstep);
              ///find left over 1
              TR_LO_1 = (HighStep - (TRstep * TR_PE));
              ///find skip 1
              if (TR_LO_1 > 0)
              {
                TR_SE_1 = (HighStep / TR_LO_1);
              }
              else
              {
                TR_SE_1 = 0;
              }
              ///find left over 2
              if (TR_SE_1 > 0)
              {
                TR_LO_2 = HighStep - ((TRstep * TR_PE) + ((TRstep * TR_PE) / TR_SE_1));
              }
              else
              {
                TR_LO_2 = 0;
              }
              ///find skip 2
              if (TR_LO_2 > 0)
              {
                TR_SE_2 = (HighStep / TR_LO_2);
              }
              else
              {
                TR_SE_2 = 0;
              }
              /////////  TR  ///////////////
              if (TR_SE_2 == 0)
              {
                TR_SE_2cur = (TR_SE_2 + 1);
              }
              if (TR_SE_2cur != TR_SE_2)
              {
                TR_SE_2cur = ++TR_SE_2cur;
                if (TR_SE_1 == 0)
                {
                  TR_SE_1cur = (TR_SE_1 + 1);
                }
                if (TR_SE_1cur != TR_SE_1)
                {
                  TR_SE_1cur = ++TR_SE_1cur;
                  TR_PEcur = ++TR_PEcur;
                  if (TR_PEcur == TR_PE)
                  {
                    TRcur = ++TRcur;
                    TR_PEcur = 0;
                    digitalWrite(TRstepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(TRstepPin, HIGH);
                  }
                }
                else
                {
                  TR_SE_1cur = 0;
                }
              }
              else
              {
                TR_SE_2cur = 0;
              }
            }


            // inc cur step
            highStepCur = ++highStepCur;
            delayMicroseconds(100);


          }

          ++i;
        }


        ////////// check for stalled motor
        ErrorTrue = 0;
        ErrorCode = "00";
        J1error = "0";
        J2error = "0";
        J3error = "0";
        J4error = "0";
        J5error = "0";
        J6error = "0";

        int J1curStep = J1encPos.read() / J1encMult;
        int J2curStep = J2encPos.read() / J2encMult;
        int J3curStep = J3encPos.read() / J3encMult;
        int J4curStep = J4encPos.read() / J4encMult;
        int J5curStep = J5encPos.read() / J5encMult;
        int J6curStep = J6encPos.read() / J6encMult;

        if (abs(J1curStep - J1tarStep) >  J1encMult / EncDiv)
        {
          J1error = "1";
          ErrorTrue = 1;
        }
        if (abs(J2curStep - J2tarStep) >  J2encMult / EncDiv)
        {
          J2error = "1";
          ErrorTrue = 1;
        }
        if (abs(J3curStep - J3tarStep) >  J3encMult / EncDiv)
        {
          J3error = "1";
          ErrorTrue = 1;
        }
        if (abs(J4curStep - J4tarStep) >  J4encMult / EncDiv)
        {
          J4error = "1";
          ErrorTrue = 1;
        }
        if (abs(J5curStep - J5tarStep) >  J5encMult / EncDiv)
        {
          J5error = "1";
          ErrorTrue = 1;
        }
        if (abs(J6curStep - J6tarStep) >  J6encMult / EncDiv)
        {
          J6error = "1";
          ErrorTrue = 1;
        }

        if (ErrorTrue == 1)
        {
          ErrorCode = "01" + J1error + J2error + J3error + J4error + J5error + J6error + "A" + String(J1curStep) + "B" + String(J2curStep) + "C" + String(J3curStep) + "D" + String(J4curStep) + "E" + String(J5curStep) + "F" + String(J6curStep);
        }
        else if (ErrorTrue == 0)
        {
          J1encPos.write(J1tarStep * J1encMult);
          J2encPos.write(J2tarStep * J2encMult);
          J3encPos.write(J3tarStep * J3encMult);
          J4encPos.write(J4tarStep * J4encMult);
          J5encPos.write(J5tarStep * J5encMult);
          J6encPos.write(J6tarStep * J6encMult);
        }

        WayPtDel = 0;
        Serial.println();
      }
      /// END OF MOVE L ///





      ///////////////////////////////////////////////////////////////////////////////////////////
      //----- MOVE C ---------------------------------------------------
      //-----------------------------------------------------------------------
      ///////////////////////////////////////////////////////////////////////////////////////////
      if (function == "MC")
      {
        WayPtDel = 1;
        int NumPtsStart = inData.indexOf('C');
        int WayPts = inData.substring(NumPtsStart + 1).toInt();
        Serial.println();
        inData = ""; // Clear recieved buffer
        //STORE WAYPOINTS
        int i = 0;
        while (i < WayPts) {
          while (Serial.available() > 0) {
            char recieved = Serial.read();
            inData += recieved;
            if (recieved == '\n') {
              inData.toCharArray(WayPt[i], 70);
              Serial.println();
              ++i;
              inData = ""; // Clear recieved buffer
            }
          }
        }
        Serial.println();
        //EXECUTE WAYPOINTS
        int ErrorTrue = 0;
        String ErrorCode = "00";
        String J1error = "0";
        String J2error = "0";
        String J3error = "0";
        String J4error = "0";
        String J5error = "0";
        String J6error = "0";
        int J1tarStep = 0;
        int J2tarStep = 0;
        int J3tarStep = 0;
        int J4tarStep = 0;
        int J5tarStep = 0;
        int J6tarStep = 0;

        i = 0;
        while (i < WayPts+1) {
          inData = WayPt[i];
          int J1start = inData.indexOf('A');
          int J2start = inData.indexOf('B');
          int J3start = inData.indexOf('C');
          int J4start = inData.indexOf('D');
          int J5start = inData.indexOf('E');
          int J6start = inData.indexOf('F');
          int TRstart = inData.indexOf('T');
          int Adstart = inData.indexOf('G');
          int Asstart = inData.indexOf('H');
          int Ddstart = inData.indexOf('I');
          int Dsstart = inData.indexOf('K');
          int SPstart = inData.indexOf('S');
          int J1Tstart = inData.indexOf('U');
          int J2Tstart = inData.indexOf('V');
          int J3Tstart = inData.indexOf('W');
          int J4Tstart = inData.indexOf('X');
          int J5Tstart = inData.indexOf('Y');
          int J6Tstart = inData.indexOf('Z');
          int J1dir = inData.substring(J1start + 1, J1start + 2).toInt();
          int J2dir = inData.substring(J2start + 1, J2start + 2).toInt();
          int J3dir = inData.substring(J3start + 1, J3start + 2).toInt();
          int J4dir = inData.substring(J4start + 1, J4start + 2).toInt();
          int J5dir = inData.substring(J5start + 1, J5start + 2).toInt();
          int J6dir = inData.substring(J6start + 1, J6start + 2).toInt();
          int TRdir = inData.substring(TRstart + 1, TRstart + 2).toInt();
          int J1step = inData.substring(J1start + 2, J2start).toInt();
          int J2step = inData.substring(J2start + 2, J3start).toInt();
          int J3step = inData.substring(J3start + 2, J4start).toInt();
          int J4step = inData.substring(J4start + 2, J5start).toInt();
          int J5step = inData.substring(J5start + 2, J6start).toInt();
          int J6step = inData.substring(J6start + 2, TRstart).toInt();
          int TRstep = inData.substring(TRstart + 2, SPstart).toInt();
          float SpeedIn = inData.substring(SPstart + 1, Adstart).toFloat();
          float ACCdur = inData.substring(Adstart + 1, Asstart).toInt();
          float ACCspd = inData.substring(Asstart + 1, Ddstart).toInt();
          float DCCdur = inData.substring(Ddstart + 1, Dsstart).toInt();
          float DCCspd = inData.substring(Dsstart + 1, J1Tstart).toInt();
          int J1tarStep = inData.substring(J1Tstart + 1, J2Tstart).toInt();
          int J2tarStep = inData.substring(J2Tstart + 1, J3Tstart).toInt();
          int J3tarStep = inData.substring(J3Tstart + 1, J4Tstart).toInt();
          int J4tarStep = inData.substring(J4Tstart + 1, J5Tstart).toInt();
          int J5tarStep = inData.substring(J5Tstart + 1, J6Tstart).toInt();
          int J6tarStep = inData.substring(J6Tstart + 1).toInt();


          //FIND HIGHEST STEP
          int HighStep = J1step;
          if (J2step > HighStep)
          {
            HighStep = J2step;
          }
          if (J3step > HighStep)
          {
            HighStep = J3step;
          }
          if (J4step > HighStep)
          {
            HighStep = J4step;
          }
          if (J5step > HighStep)
          {
            HighStep = J5step;
          }
          if (J6step > HighStep)
          {
            HighStep = J6step;
          }
          if (TRstep > HighStep)
          {
            HighStep = TRstep;
          }

          //FIND ACTIVE JOINTS
          int J1active = 0;
          int J2active = 0;
          int J3active = 0;
          int J4active = 0;
          int J5active = 0;
          int J6active = 0;
          int TRactive = 0;
          int Jactive = 0;

          if (J1step >= 1)
          {
            J1active = 1;
          }
          if (J2step >= 1)
          {
            J2active = 1;
          }
          if (J3step >= 1)
          {
            J3active = 1;
          }
          if (J4step >= 1)
          {
            J4active = 1;
          }
          if (J5step >= 1)
          {
            J5active = 1;
          }
          if (J6step >= 1)
          {
            J6active = 1;
          }
          if (TRstep >= 1)
          {
            TRactive = 1;
          }
          Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + TRactive);

          int J1_PE = 0;
          int J2_PE = 0;
          int J3_PE = 0;
          int J4_PE = 0;
          int J5_PE = 0;
          int J6_PE = 0;
          int TR_PE = 0;

          int J1_SE_1 = 0;
          int J2_SE_1 = 0;
          int J3_SE_1 = 0;
          int J4_SE_1 = 0;
          int J5_SE_1 = 0;
          int J6_SE_1 = 0;
          int TR_SE_1 = 0;

          int J1_SE_2 = 0;
          int J2_SE_2 = 0;
          int J3_SE_2 = 0;
          int J4_SE_2 = 0;
          int J5_SE_2 = 0;
          int J6_SE_2 = 0;
          int TR_SE_2 = 0;

          int J1_LO_1 = 0;
          int J2_LO_1 = 0;
          int J3_LO_1 = 0;
          int J4_LO_1 = 0;
          int J5_LO_1 = 0;
          int J6_LO_1 = 0;
          int TR_LO_1 = 0;

          int J1_LO_2 = 0;
          int J2_LO_2 = 0;
          int J3_LO_2 = 0;
          int J4_LO_2 = 0;
          int J5_LO_2 = 0;
          int J6_LO_2 = 0;
          int TR_LO_2 = 0;

          //reset
          int J1cur = 0;
          int J2cur = 0;
          int J3cur = 0;
          int J4cur = 0;
          int J5cur = 0;
          int J6cur = 0;
          int TRcur = 0;

          int J1_PEcur = 0;
          int J2_PEcur = 0;
          int J3_PEcur = 0;
          int J4_PEcur = 0;
          int J5_PEcur = 0;
          int J6_PEcur = 0;
          int TR_PEcur = 0;

          int J1_SE_1cur = 0;
          int J2_SE_1cur = 0;
          int J3_SE_1cur = 0;
          int J4_SE_1cur = 0;
          int J5_SE_1cur = 0;
          int J6_SE_1cur = 0;
          int TR_SE_1cur = 0;

          int J1_SE_2cur = 0;
          int J2_SE_2cur = 0;
          int J3_SE_2cur = 0;
          int J4_SE_2cur = 0;
          int J5_SE_2cur = 0;
          int J6_SE_2cur = 0;
          int TR_SE_2cur = 0;

          int highStepCur = 0;
          float curDelay = 0;


          //SET DIRECTIONS

          /////// J1 /////////
          if (J1dir == 1 && J1rotdir == 1)
          {
            digitalWrite(J1dirPin, LOW);
          }
          else if (J1dir == 1 && J1rotdir == 0)
          {
            digitalWrite(J1dirPin, HIGH);
          }
          else if (J1dir == 0 && J1rotdir == 1)
          {
            digitalWrite(J1dirPin, HIGH);
          }
          else if (J1dir == 0 && J1rotdir == 0)
          {
            digitalWrite(J1dirPin, LOW);
          }

          /////// J2 /////////
          if (J2dir == 1 && J2rotdir == 1)
          {
            digitalWrite(J2dirPin, LOW);
          }
          else if (J2dir == 1 && J2rotdir == 0)
          {
            digitalWrite(J2dirPin, HIGH);
          }
          else if (J2dir == 0 && J2rotdir == 1)
          {
            digitalWrite(J2dirPin, HIGH);
          }
          else if (J2dir == 0 && J2rotdir == 0)
          {
            digitalWrite(J2dirPin, LOW);
          }

          /////// J3 /////////
          if (J3dir == 1 && J3rotdir == 1)
          {
            digitalWrite(J3dirPin, LOW);
          }
          else if (J3dir == 1 && J3rotdir == 0)
          {
            digitalWrite(J3dirPin, HIGH);
          }
          else if (J3dir == 0 && J3rotdir == 1)
          {
            digitalWrite(J3dirPin, HIGH);
          }
          else if (J3dir == 0 && J3rotdir == 0)
          {
            digitalWrite(J3dirPin, LOW);
          }

          /////// J4 /////////
          if (J4dir == 1 && J4rotdir == 1)
          {
            digitalWrite(J4dirPin, LOW);
          }
          else if (J4dir == 1 && J4rotdir == 0)
          {
            digitalWrite(J4dirPin, HIGH);
          }
          else if (J4dir == 0 && J4rotdir == 1)
          {
            digitalWrite(J4dirPin, HIGH);
          }
          else if (J4dir == 0 && J4rotdir == 0)
          {
            digitalWrite(J4dirPin, LOW);
          }

          /////// J5 /////////
          if (J5dir == 1 && J5rotdir == 1)
          {
            digitalWrite(J5dirPin, LOW);
          }
          else if (J5dir == 1 && J5rotdir == 0)
          {
            digitalWrite(J5dirPin, HIGH);
          }
          else if (J5dir == 0 && J5rotdir == 1)
          {
            digitalWrite(J5dirPin, HIGH);
          }
          else if (J5dir == 0 && J5rotdir == 0)
          {
            digitalWrite(J5dirPin, LOW);
          }

          /////// J6 /////////
          if (J6dir == 1 && J6rotdir == 1)
          {
            digitalWrite(J6dirPin, LOW);
          }
          else if (J6dir == 1 && J6rotdir == 0)
          {
            digitalWrite(J6dirPin, HIGH);
          }
          else if (J6dir == 0 && J6rotdir == 1)
          {
            digitalWrite(J6dirPin, HIGH);
          }
          else if (J6dir == 0 && J6rotdir == 0)
          {
            digitalWrite(J6dirPin, LOW);
          }

          /////// TRACK /////////
          if (TRdir == 1 && TRACKrotdir == 1)
          {
            digitalWrite(TRdirPin, LOW);
          }
          else if (TRdir == 1 && TRACKrotdir == 0)
          {
            digitalWrite(TRdirPin, HIGH);
          }
          else if (TRdir == 0 && TRACKrotdir == 1)
          {
            digitalWrite(TRdirPin, HIGH);
          }
          else if (TRdir == 0 && TRACKrotdir == 0)
          {
            digitalWrite(TRdirPin, LOW);
          }


          /////CALC SPEEDS//////
          float AdjSpeed = (SpeedIn / 100);
          //REG SPEED
          float LspeedAdj = 3;
          float CalcRegSpeed = ((SpeedMult * LspeedAdj) / AdjSpeed);
          curDelay = int(CalcRegSpeed) / Jactive;

          //REG SPEED
          //float AdjSpeed = (SpeedIn / 100);
          //float CalcRegSpeed = (SpeedMult / AdjSpeed);
          //int REGSpeed = int(CalcRegSpeed);
          //curDelay = (REGSpeed / Jactive);



          ///// DRIVE MOTORS /////
          while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step || TRcur < TRstep)
            //while (J1curStep < J1tarStep || J1curStep != J1tarStep)
          {

            /////// J1 ////////////////////////////////
            ///find pulse every
            if (J1cur < J1step)
            {
              J1_PE = (HighStep / J1step);
              ///find left over 1
              J1_LO_1 = (HighStep - (J1step * J1_PE));
              ///find skip 1
              if (J1_LO_1 > 0)
              {
                J1_SE_1 = (HighStep / J1_LO_1);
              }
              else
              {
                J1_SE_1 = 0;
              }
              ///find left over 2
              if (J1_SE_1 > 0)
              {
                J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
              }
              else
              {
                J1_LO_2 = 0;
              }
              ///find skip 2
              if (J1_LO_2 > 0)
              {
                J1_SE_2 = (HighStep / J1_LO_2);
              }
              else
              {
                J1_SE_2 = 0;
              }
              /////////  J1  ///////////////
              if (J1_SE_2 == 0)
              {
                J1_SE_2cur = (J1_SE_2 + 1);
              }
              if (J1_SE_2cur != J1_SE_2)
              {
                J1_SE_2cur = ++J1_SE_2cur;
                if (J1_SE_1 == 0)
                {
                  J1_SE_1cur = (J1_SE_1 + 1);
                }
                if (J1_SE_1cur != J1_SE_1)
                {
                  J1_SE_1cur = ++J1_SE_1cur;
                  J1_PEcur = ++J1_PEcur;
                  if (J1_PEcur == J1_PE)
                  {
                    J1cur = ++J1cur;
                    J1_PEcur = 0;
                    digitalWrite(J1stepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(J1stepPin, HIGH);
                  }
                }
                else
                {
                  J1_SE_1cur = 0;
                }
              }
              else
              {
                J1_SE_2cur = 0;
              }
            }

            /////// J2 ////////////////////////////////
            ///find pulse every
            if (J2cur < J2step)
            {
              J2_PE = (HighStep / J2step);
              ///find left over 1
              J2_LO_1 = (HighStep - (J2step * J2_PE));
              ///find skip 1
              if (J2_LO_1 > 0)
              {
                J2_SE_1 = (HighStep / J2_LO_1);
              }
              else
              {
                J2_SE_1 = 0;
              }
              ///find left over 2
              if (J2_SE_1 > 0)
              {
                J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
              }
              else
              {
                J2_LO_2 = 0;
              }
              ///find skip 2
              if (J2_LO_2 > 0)
              {
                J2_SE_2 = (HighStep / J2_LO_2);
              }
              else
              {
                J2_SE_2 = 0;
              }
              /////////  J2  ///////////////
              if (J2_SE_2 == 0)
              {
                J2_SE_2cur = (J2_SE_2 + 1);
              }
              if (J2_SE_2cur != J2_SE_2)
              {
                J2_SE_2cur = ++J2_SE_2cur;
                if (J2_SE_1 == 0)
                {
                  J2_SE_1cur = (J2_SE_1 + 1);
                }
                if (J2_SE_1cur != J2_SE_1)
                {
                  J2_SE_1cur = ++J2_SE_1cur;
                  J2_PEcur = ++J2_PEcur;
                  if (J2_PEcur == J2_PE)
                  {
                    J2cur = ++J2cur;
                    J2_PEcur = 0;
                    digitalWrite(J2stepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(J2stepPin, HIGH);
                  }
                }
                else
                {
                  J2_SE_1cur = 0;
                }
              }
              else
              {
                J2_SE_2cur = 0;
              }
            }

            /////// J3 ////////////////////////////////
            ///find pulse every
            if (J3cur < J3step)
            {
              J3_PE = (HighStep / J3step);
              ///find left over 1
              J3_LO_1 = (HighStep - (J3step * J3_PE));
              ///find skip 1
              if (J3_LO_1 > 0)
              {
                J3_SE_1 = (HighStep / J3_LO_1);
              }
              else
              {
                J3_SE_1 = 0;
              }
              ///find left over 2
              if (J3_SE_1 > 0)
              {
                J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
              }
              else
              {
                J3_LO_2 = 0;
              }
              ///find skip 2
              if (J3_LO_2 > 0)
              {
                J3_SE_2 = (HighStep / J3_LO_2);
              }
              else
              {
                J3_SE_2 = 0;
              }
              /////////  J3  ///////////////
              if (J3_SE_2 == 0)
              {
                J3_SE_2cur = (J3_SE_2 + 1);
              }
              if (J3_SE_2cur != J3_SE_2)
              {
                J3_SE_2cur = ++J3_SE_2cur;
                if (J3_SE_1 == 0)
                {
                  J3_SE_1cur = (J3_SE_1 + 1);
                }
                if (J3_SE_1cur != J3_SE_1)
                {
                  J3_SE_1cur = ++J3_SE_1cur;
                  J3_PEcur = ++J3_PEcur;
                  if (J3_PEcur == J3_PE)
                  {
                    J3cur = ++J3cur;
                    J3_PEcur = 0;
                    digitalWrite(J3stepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(J3stepPin, HIGH);
                  }
                }
                else
                {
                  J3_SE_1cur = 0;
                }
              }
              else
              {
                J3_SE_2cur = 0;
              }
            }

            /////// J4 ////////////////////////////////
            ///find pulse every
            if (J4cur < J4step)
            {
              J4_PE = (HighStep / J4step);
              ///find left over 1
              J4_LO_1 = (HighStep - (J4step * J4_PE));
              ///find skip 1
              if (J4_LO_1 > 0)
              {
                J4_SE_1 = (HighStep / J4_LO_1);
              }
              else
              {
                J4_SE_1 = 0;
              }
              ///find left over 2
              if (J4_SE_1 > 0)
              {
                J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
              }
              else
              {
                J4_LO_2 = 0;
              }
              ///find skip 2
              if (J4_LO_2 > 0)
              {
                J4_SE_2 = (HighStep / J4_LO_2);
              }
              else
              {
                J4_SE_2 = 0;
              }
              /////////  J4  ///////////////
              if (J4_SE_2 == 0)
              {
                J4_SE_2cur = (J4_SE_2 + 1);
              }
              if (J4_SE_2cur != J4_SE_2)
              {
                J4_SE_2cur = ++J4_SE_2cur;
                if (J4_SE_1 == 0)
                {
                  J4_SE_1cur = (J4_SE_1 + 1);
                }
                if (J4_SE_1cur != J4_SE_1)
                {
                  J4_SE_1cur = ++J4_SE_1cur;
                  J4_PEcur = ++J4_PEcur;
                  if (J4_PEcur == J4_PE)
                  {
                    J4cur = ++J4cur;
                    J4_PEcur = 0;
                    digitalWrite(J4stepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(J4stepPin, HIGH);
                  }
                }
                else
                {
                  J4_SE_1cur = 0;
                }
              }
              else
              {
                J4_SE_2cur = 0;
              }
            }

            /////// J5 ////////////////////////////////
            ///find pulse every
            if (J5cur < J5step)
            {
              J5_PE = (HighStep / J5step);
              ///find left over 1
              J5_LO_1 = (HighStep - (J5step * J5_PE));
              ///find skip 1
              if (J5_LO_1 > 0)
              {
                J5_SE_1 = (HighStep / J5_LO_1);
              }
              else
              {
                J5_SE_1 = 0;
              }
              ///find left over 2
              if (J5_SE_1 > 0)
              {
                J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
              }
              else
              {
                J5_LO_2 = 0;
              }
              ///find skip 2
              if (J5_LO_2 > 0)
              {
                J5_SE_2 = (HighStep / J5_LO_2);
              }
              else
              {
                J5_SE_2 = 0;
              }
              /////////  J5  ///////////////
              if (J5_SE_2 == 0)
              {
                J5_SE_2cur = (J5_SE_2 + 1);
              }
              if (J5_SE_2cur != J5_SE_2)
              {
                J5_SE_2cur = ++J5_SE_2cur;
                if (J5_SE_1 == 0)
                {
                  J5_SE_1cur = (J5_SE_1 + 1);
                }
                if (J5_SE_1cur != J5_SE_1)
                {
                  J5_SE_1cur = ++J5_SE_1cur;
                  J5_PEcur = ++J5_PEcur;
                  if (J5_PEcur == J5_PE)
                  {
                    J5cur = ++J5cur;
                    J5_PEcur = 0;
                    digitalWrite(J5stepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(J5stepPin, HIGH);
                  }
                }
                else
                {
                  J5_SE_1cur = 0;
                }
              }
              else
              {
                J5_SE_2cur = 0;
              }
            }

            /////// J6 ////////////////////////////////
            ///find pulse every
            if (J6cur < J6step)
            {
              J6_PE = (HighStep / J6step);
              ///find left over 1
              J6_LO_1 = (HighStep - (J6step * J6_PE));
              ///find skip 1
              if (J6_LO_1 > 0)
              {
                J6_SE_1 = (HighStep / J6_LO_1);
              }
              else
              {
                J6_SE_1 = 0;
              }
              ///find left over 2
              if (J6_SE_1 > 0)
              {
                J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
              }
              else
              {
                J6_LO_2 = 0;
              }
              ///find skip 2
              if (J6_LO_2 > 0)
              {
                J6_SE_2 = (HighStep / J6_LO_2);
              }
              else
              {
                J6_SE_2 = 0;
              }
              /////////  J6  ///////////////
              if (J6_SE_2 == 0)
              {
                J6_SE_2cur = (J6_SE_2 + 1);
              }
              if (J6_SE_2cur != J6_SE_2)
              {
                J6_SE_2cur = ++J6_SE_2cur;
                if (J6_SE_1 == 0)
                {
                  J6_SE_1cur = (J6_SE_1 + 1);
                }
                if (J6_SE_1cur != J6_SE_1)
                {
                  J6_SE_1cur = ++J6_SE_1cur;
                  J6_PEcur = ++J6_PEcur;
                  if (J6_PEcur == J6_PE)
                  {
                    J6cur = ++J6cur;
                    J6_PEcur = 0;
                    digitalWrite(J6stepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(J6stepPin, HIGH);
                  }
                }
                else
                {
                  J6_SE_1cur = 0;
                }
              }
              else
              {
                J6_SE_2cur = 0;
              }
            }

            /////// TR ////////////////////////////////
            ///find pulse every
            if (TRcur < TRstep)
            {
              TR_PE = (HighStep / TRstep);
              ///find left over 1
              TR_LO_1 = (HighStep - (TRstep * TR_PE));
              ///find skip 1
              if (TR_LO_1 > 0)
              {
                TR_SE_1 = (HighStep / TR_LO_1);
              }
              else
              {
                TR_SE_1 = 0;
              }
              ///find left over 2
              if (TR_SE_1 > 0)
              {
                TR_LO_2 = HighStep - ((TRstep * TR_PE) + ((TRstep * TR_PE) / TR_SE_1));
              }
              else
              {
                TR_LO_2 = 0;
              }
              ///find skip 2
              if (TR_LO_2 > 0)
              {
                TR_SE_2 = (HighStep / TR_LO_2);
              }
              else
              {
                TR_SE_2 = 0;
              }
              /////////  TR  ///////////////
              if (TR_SE_2 == 0)
              {
                TR_SE_2cur = (TR_SE_2 + 1);
              }
              if (TR_SE_2cur != TR_SE_2)
              {
                TR_SE_2cur = ++TR_SE_2cur;
                if (TR_SE_1 == 0)
                {
                  TR_SE_1cur = (TR_SE_1 + 1);
                }
                if (TR_SE_1cur != TR_SE_1)
                {
                  TR_SE_1cur = ++TR_SE_1cur;
                  TR_PEcur = ++TR_PEcur;
                  if (TR_PEcur == TR_PE)
                  {
                    TRcur = ++TRcur;
                    TR_PEcur = 0;
                    digitalWrite(TRstepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(TRstepPin, HIGH);
                  }
                }
                else
                {
                  TR_SE_1cur = 0;
                }
              }
              else
              {
                TR_SE_2cur = 0;
              }
            }


            // inc cur step
            highStepCur = ++highStepCur;
            delayMicroseconds(100);


          }

          ++i;
        }


        ////////// check for stalled motor
        ErrorTrue = 0;
        ErrorCode = "00";
        J1error = "0";
        J2error = "0";
        J3error = "0";
        J4error = "0";
        J5error = "0";
        J6error = "0";

        int J1curStep = J1encPos.read() / J1encMult;
        int J2curStep = J2encPos.read() / J2encMult;
        int J3curStep = J3encPos.read() / J3encMult;
        int J4curStep = J4encPos.read() / J4encMult;
        int J5curStep = J5encPos.read() / J5encMult;
        int J6curStep = J6encPos.read() / J6encMult;

        if (abs(J1curStep - J1tarStep) >  J1encMult / EncDiv)
        {
          J1error = "1";
          ErrorTrue = 1;
        }
        if (abs(J2curStep - J2tarStep) >  J2encMult / EncDiv)
        {
          J2error = "1";
          ErrorTrue = 1;
        }
        if (abs(J3curStep - J3tarStep) >  J3encMult / EncDiv)
        {
          J3error = "1";
          ErrorTrue = 1;
        }
        if (abs(J4curStep - J4tarStep) >  J4encMult / EncDiv)
        {
          J4error = "1";
          ErrorTrue = 1;
        }
        if (abs(J5curStep - J5tarStep) >  J5encMult / EncDiv)
        {
          J5error = "1";
          ErrorTrue = 1;
        }
        if (abs(J6curStep - J6tarStep) >  J6encMult / EncDiv)
        {
          J6error = "1";
          ErrorTrue = 1;
        }

        if (ErrorTrue == 1)
        {
          ErrorCode = "01" + J1error + J2error + J3error + J4error + J5error + J6error + "A" + String(J1curStep) + "B" + String(J2curStep) + "C" + String(J3curStep) + "D" + String(J4curStep) + "E" + String(J5curStep) + "F" + String(J6curStep);
        }
        else if (ErrorTrue == 0)
        {
          J1encPos.write(J1tarStep * J1encMult);
          J2encPos.write(J2tarStep * J2encMult);
          J3encPos.write(J3tarStep * J3encMult);
          J4encPos.write(J4tarStep * J4encMult);
          J5encPos.write(J5tarStep * J5encMult);
          J6encPos.write(J6tarStep * J6encMult);
        }

        WayPtDel = 0;
        Serial.println();
      }
      /// END OF MOVE C ///






      
      else
      {
        inData = ""; // Clear recieved buffer
      }
    }
  }
}
