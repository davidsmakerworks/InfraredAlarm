/*
 * Infrared Alarm for Arduino
 * Copyright (c) 2019 David Rice
 * 
 * This is a simple alarm system using an Arduino, an IR receiver,
 * a motion sensor, an RGB LED, and an active buzzer. The alarm is
 * controlled using any standard infrared remote control.
 * 
 * Global variables are (over)used in this sketch because it is
 * intended to be a simple example that can be easily modified by
 * newer programmers. It would be more stylistically correct to 
 * define many of these as static variables in the loop() function
 * and pass them as parameters to functions that use or modify those
 * values.
 * 
 * Requires IRremote library by shirriff
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <IRremote.h>

/* Pin definitions for common-cathode RGB LED */
const int redPin = 6;
const int greenPin = 7;
const int bluePin = 8;

const int buzzerPin = 9;
const int motionSensorPin = 10;

const int irRecvPin = 11;

/* 
 * Time (in milliseconds) the system will wait after the motion 
 * sensor is triggered before sounding the alarm.
 */
const int triggerWait = 15000;

/*
 * States used in main loop for finite state machine
 */
typedef enum {
  DISARMING, // System in in the process of disarming - this is needed to clear the received codes
  DISARMED, // System is currently disarmed and waiting for the arm code
  ARMED, // System is armed and will be triggered if motion is detected
  TRIGGERED, // Motion has been detected while system is armed
  ALARM // System was not disarmed after being triggered - alarm is sounding
} state_t;

/* Raw data for arm and disarm code */
const unsigned long armCode = { 0x00FF906F };
const unsigned long disarmCodes[4] = { 0x00FF5AA5, 0x00FF38C7, 0x00FF6897, 0x00FF18E7 };

/* Current state of the alarm system as defined above */
state_t state;

/* 
 * Array of the 4 most recent received IR codes. 
 * 
 * The codes will be shifted in from the right by the
 * shiftInCode() function.
 */ 
unsigned long receivedCodes[4] = { 0x00, 0x00, 0x00, 0x00 } ;

/* Time in millis when the motion sensor was triggered */
unsigned long triggerTime;

/* IRrecv object from IRremote library to process received infrared codes */
IRrecv irRecv(irRecvPin);

/* 
 * Check entered code against disarm code 
 * 
 * TODO: Generalize for codes of arbitrary length
 */
bool checkCode() {
  return (receivedCodes[0] == disarmCodes[0]) && (receivedCodes[1] == disarmCodes[1]) &&
         (receivedCodes[2] == disarmCodes[2]) && (receivedCodes[3] == disarmCodes[3]); 
}

/* 
 *  Sound active buzzer for specified number of milliseconds.
 *  
 *  Blocks while buzzer is sounding.
 */
void buzz(int ms) {
  digitalWrite(buzzerPin, HIGH);
  delay(ms);
  digitalWrite(buzzerPin, LOW);
}

/* 
 * Shift last entered code into array of received codes. 
 * This allows the system to always check against the last
 * 4 entered digits.
 * 
 * TODO: Generalize for codes of arbitrary length
 */
void shiftInCode(unsigned long code) {
  Serial.println(code, HEX);
  receivedCodes[0] = receivedCodes[1];
  receivedCodes[1] = receivedCodes[2];
  receivedCodes[2] = receivedCodes[3];
  receivedCodes[3] = code;
}

/* 
 * Check to see if the system should be armed based on comparing with the
 * single key arm code.
 */
bool checkArm(unsigned long code) {
  if (code == armCode) {
    buzz(100);
    delay(100);
    buzz(100);

    return true;
  } else {
    return false;
  } 
}

/*
 * Check to see if the system should be disarmed based on the results of the
 * checkCode() function.
 * 
 * This function does not actually update the state variable, because it would be
 * very confusing to change the state of the state machine outside of the main
 * state machine code.
 */
bool checkDisarm(unsigned long code) {
  if (code != REPEAT) {
    shiftInCode(code);
  }
  
  if (checkCode()) {
    buzz(100);
    delay(100);
    buzz(100);
    return true;
  } else {
    return false;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(buzzerPin, OUTPUT);

  irRecv.enableIRIn();

  state = DISARMED;
}

void loop() {
  decode_results results;
  
  switch (state) {
    case DISARMING:
      /* 
       * Clear last received codes to prevent system from disarming
       * immediately after is is re-armed.
       */
      for (int i = 0; i < 4; i++) {
        receivedCodes[i] = 0;
      }

      state = DISARMED;
      break;
      
    case DISARMED:
      /* Set LED to green (disarmed) */
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, HIGH); 
      digitalWrite(bluePin, LOW);

      /* Turn off buzzer */
      digitalWrite(buzzerPin, LOW);

      /* 
       * Check for pending IR code and compare it with 
       * the arm code. If there is a match, sound
       * a signal and enter ARMED state.
       */
      if (irRecv.decode(&results)) {
        irRecv.resume();

        /*
         * Print received code on serial monitor to allow for easily changing the code
         * without needing to load a separate sketch on the Arduino. Avoids confusing the
         * display by filtering out NEC REPEAT codes.
         */
        if (results.value != REPEAT) {
          Serial.print("Received IR value: ");
          Serial.println(results.value, HEX);
        }
        
        if (checkArm(results.value)) {
          state = ARMED;
        }
      }
      break;

    case ARMED:
      /* Set LED to blue (armed) */
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, HIGH);

      /*
       * If motion sensor has detected motion, sound a tone,
       * record time stamp, and enter TRIGGERED state.
       */
      if (digitalRead(motionSensorPin)) {
        buzz(500);
        triggerTime = millis();
        state = TRIGGERED;
      }

      /*
       * Read any pending IR codes and compare the received codes
       * against the disarm code. If the disarm code has been entered, sound 
       * a signal and enter DISARMING state.
       */
      if (irRecv.decode(&results)) {
        if (checkDisarm(results.value)) {
          state = DISARMING;
        }
        
        irRecv.resume();
      }
      break;

      case TRIGGERED:
        /* Set LED to yellow (triggered) */
        digitalWrite(redPin, HIGH);
        digitalWrite(greenPin, HIGH);
        digitalWrite(bluePin, LOW);

        /* If the trigger wait time has elapsed, enter ALARM state */
        if (millis() - triggerTime > triggerWait) {
          state = ALARM;
        }

        /* 
         * This generates a short beep about once per second. 
         * 
         * 1024 (a power of two) is used instead of 1000 to avoid an
         * unnecessary divide operation.
         */ 
        if (millis() % 1024 == 0) {
          buzz(25);
        }

        /*
         * Read any pending IR codes and compare the received codes
         * against the disarm code. If the disarm code has been entered, sound 
         * a signal and enter DISARMING state.
         */
        if (irRecv.decode(&results)) {
          if (checkDisarm(results.value)) {
            state = DISARMING;
          }
          
          irRecv.resume();
        }
        break;
        
      case ALARM:
        /* Set LED to red (alarm) */
        digitalWrite(redPin, HIGH);
        digitalWrite(greenPin, LOW);
        digitalWrite(bluePin, LOW);

        /* Turn on buzzer */
        digitalWrite(buzzerPin, HIGH);

        /*
         * Read any pending IR codes and compare the received codes
         * against the disarm code. If the disarm code has been entered, sound 
         * a signal and enter DISARMING state.
         */
        if (irRecv.decode(&results)) {
          if (checkDisarm(results.value)) {
            state = DISARMING;
          }
          
          irRecv.resume();
        }
        break;
        
      default:
        /* 
         * We should never get to this point.
         * Set LED to magenta indicating that something has 
         * gone horribly wrong.
         */
         digitalWrite(redPin, HIGH);
         digitalWrite(greenPin, LOW);
         digitalWrite(bluePin, HIGH);
         break;
  }
}
