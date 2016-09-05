#include "RCRPID.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <SD.h>
#include <EEPROM.h>

#define targetAlt 1300

//Special pin read/writes
#define portOfPin(P)\ (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\ (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

//Special pin read/writes cont
#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))

//GLOBAL VARIABLES-----------------------------------------
long padAlt;//The sea level (SL) altitude of the the launchpad. (mm)
long y; //A variable in which to keep the most recent altitude (mm)
float v; //A variable in which to keep the most recent velocity (m/s)
float error; //A variable that is the difference between how fast we want to be going and how fast we are actually going. (m/s) (see proportional control)
const float pGain = 0.45; //The 'kP' coefficient (also see proportional control) (constant)
int control; //The calculated % VDS deployment we need (%)
int output;

const int n = 9; //The number of data points to read velocity from
unsigned long times[n];   //The n most recent times (ms)
long alts[n];    //The n most recent altitudesAGL (mm)
volatile int encPos = 0; //Where the encoder is (encoder units)

//PIN NUMBERS-----------------------------------------------
#define chipSelect  10 //The SD card's chip select pin
#define actuatorPWM 9//PWM: 3, 5, 6, 9, 10, and 11.
#define zeroButton 17
#define encA 3//External Interrupts: 2 and 3.
#define encB 4
#define in1 7
#define in2 8

//More SD card pins
//DI on pin 11
//DO on pin 12
//CLK on pin 13
//Bmp180 i2c pins
//SDA (white) on a4
//SCL (blue) on a5

//create bmp180 objects
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
sensors_event_t event;

//Create SD card object
File dataFile;

//Create PID objects
RCRPID motorPID(&encPos, &output, &control, 10, 0, 0, -255, 255);
//RCRPID brakesPID(&encPos, &output, &deploy, 10, 0, 0, -255, 255);

//SETUP=====================================================
void setup() {
  //initialize the serial port... This is for debugging. Not needed in the field. It slows the code to have it enabled
  Serial.begin(9600);
  //Serial.println("Hello");
  //Wait for the Bmp180 to wake up
  if (!bmp.begin()) {
    while (1);
  }

  //BEGIN SD CONFIGURATION---------------------------------------------------------
  //Serial.print(F("SD card?"));
  if (!SD.begin(chipSelect)) {   // see if the card is present and can be initialized:
    //Serial.println(F("uh-oh"));
    for (int i = 0; i < 10; i++) {  //changed from an infinite loop to a for
      //Serial.println(F("Nope"));
      delay(1000);
    }
  }
  else {
    //Serial.println(F("card yes"));
  }//END SD CONFIGURATION------------------------------------------------------

  //PID configuration


  //pinMode initializations and stuff
  pinMode(zeroButton, INPUT);
  pinMode(actuatorPWM, OUTPUT);
  pinMode(encA, INPUT);
  digitalWrite(encA, HIGH);
  pinMode(encB, INPUT);
  digitalWrite(encB, HIGH);
  //pinAsInputPullUp(encA);
  //pinAsInputPullUp(encB);
  pinMode(in1, INPUT);
  pinMode(in2, INPUT);
  attachInterrupt(1, doEncoder, RISING);



  //TEST SECTION================
  //while (1) {
  //	Serial.print("enc=");
  //	Serial.println(encPos);
  //	Serial.print("output=");
  //	Serial.println(output);
  //	deploy = 420;
  //	motorPID.Compute();
  //	if (output >= 0) {
  //		motorDo(true, output);
  //	}
  //	else if (output < 0) {
  //		motorDo(false, -1 * output);
  //	}
  //}
  //END TEST SECTION===========
}

//LOOP======================================================
void loop()
{
  //Serial.println(encPos);
  //Serial.println(digitalRead(zeroButton));
  //if (isHigh(zeroButton)) { //new
  if (digitalRead(zeroButton)) {
    //while (isHigh(zeroButton)) {
    while (digitalRead(zeroButton)) {
      motorDo(true, 75);
      Serial.print("encPos_ = ");
      Serial.println(encPos);
    }
    motorDo(false, 0);
    delay(1000);
    newFlight();
    
    ///NEW CODE 9/4/2016
    if((y>=350) && (encPos < 400)){ //IF ALTITUDE IS ABOVE 350, ACTUATE ADS TO FULL EXTENSION
      motorDo(true, 255);
    } else if ((y>=350) && (encPos==400)){ //IF ALTITUDE IS ABOVE 350 AND ADS IS FULLY EXTENDED, DO NOTHING
      motorDo(true,0);
    else {
      while(encPos > 0) { //WHEN ALTITUDE IS BELOW 350, CLOSE ADS.
        motorDo(false,255);
      }
    }
   }
   //END NEW CODE 9/4/2016

  Serial.print("encPos = ");
  Serial.println(encPos);
  updateTimesAlts();//13-14 ms low power...15-17ms standard...21-23 ms highres...33-35 ms ultra high res...(with Serial)

  y = alts[0];
  if (y < 0) {
    y = 0;
  }
  //Serial.print("altitude = ");
  //Serial.println(y);
  v = velocity();

  error = v - EEPROM.read(map(y / 1000, 0, targetAlt, 0, 1023));

  control = 100 * pGain * error;
  if ((control > 100) || (y > 1300000L)) {
    control = 100;
  }
  else if (control < 0) {
    control = 0;
  }
  control = control * 4;

  //PID
  //motorPID.Compute();
  //if (output >= 0) {
  //  motorDo(false, output);
  //}
  //else if (output < 0) {
  //  motorDo(true, -1 * output);
  //}

  cardPrint(error, control, v);
}

//FUNCTIONS=================================================

void updateTimesAlts() {
  for (int i = n - 1; i > 0; i--) {
    alts[i] = alts[i - 1];
    times[i] = times[i - 1];
  }
  alts[0] = altitudeSL() - padAlt;
  times[0] = millis();
}

float velocity() {
  long sumTY = 0, sumT = 0, sumY = 0, sumT2 = 0;
  for (int i = 0; i < n; i++) {
    sumTY += times[i] * alts[i];
    sumT += times[i];
    sumY += alts[i];
    sumT2 += times[i] * times[i];
  }
  long herp = n * sumTY - sumT * sumY;
  long derp = n * sumT2 - sumT * sumT;
  return (float)herp / derp;
}

long altitudeSL() {
  bmp.getEvent(&event);
  if (event.pressure) {
    return (long)1000 * (bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, event.pressure));
  }
  else {
    return NULL;
  }
}

long newPadAlt() { //Takes the avg of 5 readings and saves the result the EEPROM
  long returnVal = altitudeSL();
  delay(500);
  returnVal += altitudeSL();
  delay(500);
  returnVal += altitudeSL();
  delay(500);
  returnVal += altitudeSL();
  delay(500);
  returnVal += altitudeSL();
  returnVal = returnVal / 5;
  return returnVal;
}

void newFlight() {
  padAlt = newPadAlt();
  encPos = 0;
  SD.remove("VDS.TXT");
  //File
  File dataFile = SD.open("VDS.TXT", FILE_WRITE);
  if (dataFile) {
    dataFile.println(F("t(ms), Alt(mm), Vel(m/s), Error(m/s), VDS Control, VDS Actual"));
    dataFile.close();
  }
}

void cardPrint(float error, int control, float velocity) {
  File dataFile = SD.open("VDS.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(times[0]);
    dataFile.print(",");
    dataFile.print(alts[0]);
    dataFile.print(",");
    dataFile.print(velocity);
    dataFile.print(",");
    dataFile.print(error);
    dataFile.print(",");
    dataFile.print(control);
    dataFile.print(",");
    dataFile.print(encPos);//mapped back to scale 0-1 in matlab
    dataFile.println(" ");
    dataFile.close();
  }
}

void doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
    forward. If they're different, it's going backward.

    For more information on speeding up this process, see
    [Reference/PortManipulation], specifically the PIND register.
  */
  if (digitalRead(encA) == digitalRead(encB)) {
    encPos--;
  }
  else {
    encPos++;
  }
}

void motorDo(boolean spin, int goRate) {
  if (spin) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(actuatorPWM, goRate);
}



//void EEPROMWriteLong(int address, long value) {
//	//Decomposition from a long to 4 bytes by using bitshift.
//	//One = Most significant -> Four = Least significant byte
//	byte four = (value & 0xFF);
//	byte three = ((value >> 8) & 0xFF);
//	byte two = ((value >> 16) & 0xFF);
//	byte one = ((value >> 24) & 0xFF);
//
//	//Write the 4 bytes into the eeprom memory.
//	EEPROM.write(address, four);
//	EEPROM.write(address + 1, three);
//	EEPROM.write(address + 2, two);
//	EEPROM.write(address + 3, one);
//}
//
//long EEPROMReadLong(long address)
//{
//	//Read the 4 bytes from the eeprom memory.
//	long four = EEPROM.read(address);
//	long three = EEPROM.read(address + 1);
//	long two = EEPROM.read(address + 2);
//	long one = EEPROM.read(address + 3);
//
//	//Return the recomposed long by using bitshift.
//	return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
//}

//
//int free_ram ()
//{
//  extern int __heap_start, *__brkval;
//  int v;
//  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
//}


/*     OLD MOVEMENTS OF MOTOR.
   MOVE THE BLADES TO AN EXTENDED POSITION
    control = 410;
    Serial.println("ZERO!");
    while (encPos < 375) {
      Serial.print("encPos__ = ");
      Serial.println(encPos);
      motorPID.Compute();
      if (output >= 0) {
        motorDo(false, output);
      }
      else if (output < 0) {
        motorDo(true, -1 * output);
      }
    }
    
    
    
    ////RETRACT THE BLADES
    Serial.println("OUT!");
    motorDo(false, 0);
    delay(2000);
    control = 0;
    while (encPos != 0) { //If you can't exit this loop change it to (encPos > 1) or something
      Serial.println(encPos);
      motorPID.Compute();
      if (output >= 0) {
        motorDo(false, output);
      }
      else if (output < 0) {
        motorDo(true, -1 * output);
      }
    }
    motorDo(false, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    Serial.println("IN!");
    delay(1000);
    encPos = 0;
    
*/


