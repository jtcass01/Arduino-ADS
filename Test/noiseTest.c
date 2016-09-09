#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <SD.h>
#include <EEPROM.h>

//SPECIAL PIN READ/WRITES
#define portOfPin(P)\ (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\ (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

//SPECIAL PIN READ/WRITES CONT.
#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))

//PIN NUMBERS
#define chipSelect 10    //The SD card's chip select pin.
#define actuatorPWM 9    //PWM: 3,5,6,9,10, and 11.
#define zeroButton 17
#define encA 3           //External Interupts: 2 and 3.
#define encB 4
#define in1 7
#define in2 8


/********************GLOBAL VARIABLES********************/
long y;                  //Most recent altitude (mm).
float v;                 //Most recent velocity (m/s).
const int n = 9;         //The number of data points to read velocity from.
unsigned long times[n];  //The n most recent times (ms).
long alts[n];            //The n most recent altitudesAGL (mm).
volatile int encPos = 0; //Where the encoder is (encoder units).


//CREATE BMP180 OBJECTS
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
sensors_event_t event;


//CREATE SD CARD OBJECT
File dataFile;


/********************FUNCTION PROTOTYPES********************/
long altitudeSL(void);                             //FINDS ALTITUDE USING BMP SENSOR
void updateTimesAlts(void);                        //UPDATES TIMES AND ALTITUDES
float velocity(void);                              //FIND VELOCITY
void motorDo(boolean,int);                         //GIVES ACTION TO MOTOR
void doEncoder(void);
void newTest(void);                                //DELETES OLD SD DATA AND HEADS FILE
void writeToFile(unsigned long, long, float, int); //WRITES TIME, ALT, VEL, ENCPOS TO SD
/********************END FUNCTION PROTOTYPES********************/


void setup(void) {
  //Wait for the Bmp180 to wake up.
  if(!bmp.begin()){
    while(true);
  }
  
  //OPEN SERIAL COMMUNICATIONS AND WAIT FOR PORT TO OPEN:
  Serial.begin(9600);
  
  
  //SD CARD CONFIGURATION
  if(!SD.begin(chipSelect)){          ///CHECK TO SEE IF SD CARD IS PRESENT AND CAN BE
    for(unsigned int i=0; i<10; i++){ ///INITIALIZED.
      delay(1000);                    ///WAIT FOR SD CARD TO INITIALIZE IF NEEDED.
    }
  } else {
    Serial.println("SD Card initialized.");
  } //END SD CARD CONFIGURATION
  
  
  //pinMode initializations and stuff
  pinMode(zeroButton, INPUT);
  pinMode(actuatorPWM, OUTPUT);
  pinMode(encA, INPUT);
  digitalWrite(encA, HIGH);
  pinMode(encB, INPUT);
  digitalWrite(encB, HIGH);
  pinMode(in1, INPUT);
  pinMode(in2, INPUT);
  attachInterrupt(1, doEncoder, RISING);
}     //////////END SETUP//////////



void loop(void){
  if(digitalRead(zeroButton)){
    newTest();
  }  
  
  if(encPos == 0){ //IF BLADES ARE RETRACTED, EXTEND THEM
    while(encPos < 400){
      motorDo(true,255);
    }
  } else if (encPos == 400) { //IF BLADES ARE EXTENDED, RETRACT THEM
    while(encPos > 0){
      motorDo(false,255);
    }
  }
  
  updateTimesAlts();
  
  y = alts[0];
  if(y<0) {
    y = 0;
  }
  
  v = velocity();
  
  writeToFile(times[0],y,v,encPos);
}     //////////END LOOP//////////




/********************FUNCTION DEFINITIONS********************/
long altitudeSL(){  //FIND CURRENT ALTITUDE USING BMP180 SENSOR
  bmp.getEvent(&event);
  if(event.pressure){
    return ((long)1000 * (bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, event.pressure)));
  } else {
    return NULL;
  }
} // END FUNCTION


void updateTimesAlts(void){  //UPDATE TIME AND ALTITUDE VALUES
  for(unsigned int i=(n-1);i>0;i--){
    alts[i] = alts[(i-1)];
    times[i] = times[(i-1)];
  } 
  alts[0] = altitudeSL();
  times[0] = millis();
} // END FUNCTION


float velocity(void) { //FIND CURRENT VELCOCITY
  long sumTY = 0, sumT = 0, sumY = 0, sumT2;
  
  for(unsigned int i=0;i<0;i++){
    sumTY += (times[i] * alts[i]);
    sumT += times[i];
    sumY += alts[i];
    sumT2 += (times[i] * times[i]);
  }
  
  long herp = (n * sumTY) - (sumT * sumY);
  long derp = (n * sumT2) - (sumT * sumT);
  
  return ((float) herp) / ((float) derp);
} // END FUNCTION


void motorDo(boolean spin, int goRate){    //FUNCTION USED TO COMMUNICATE WITH MOTOR
  if(spin) {                               //TRUE SPINS MOTOR CW
    digitalWrite(in1, HIGH);               //FALSE SPINS MOTOR CCW
    digitalWrite(in2, LOW);                //MAX/MIN RATE: (+-)255
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(actuatorPWM, goRate);
} //END FUNCTION


void doEncoder(void){   //FUNCTION USED TO SET THE POSITION OF THE ENCODER 
  /* If pinA and pinB are both high or both low, it is spinning forward.
    If they are different, it's going backward.
    
    For more information on speeding up this process, see [Reference/PortManipulation],
    specifically the PIND register*/

  if(digitalRead(encA) == digitalRead(encB)){   //FULL EXTENSION OF BLADES HAS EncPos
    encPos--;                                   //VALUE OF 400.  0 IS FULL RETRACTION.
  } else {
    encPos++;
  }
} //END FUNCTION


void newTest(void){   //FUNCTION USED TO ZERO VALUES AND READY FILE
  encPos = 0;
  SD.remove("VDS.txt");
  
  dataFile = SD.open("VDS.txt", FILE_WRITE);
  if(dataFile){
    Serial.println("VDS.txt intialized...");
    dataFile.println("t(ms), Alt(mm), Vel(m/s), encPos(0-400)");
    dataFile.close();
  } else {
    Serial.println("Error opening file.");
  }
}  //END FUNCTION



void writeToFile(unsigned long time, long altitude, float velocity, int encoderPosition){  //FUNCTION USED TO WRITE DATA TO SD CARD
  dataFile = SD.open("VDS.txt", FILE_WRITE);
  if(dataFile){
    Serial.println("Writing data to VDS.txt...");
    dataFile.print(time);
    dataFile.print(",");
    dataFile.print(altitude);
    dataFile.print(",");
    dataFile.print(velocity);
    dataFile.print(",");
    dataFile.print(encoderPosition);
    dataFile.println(" ");
  } else {
    Serial.println("Error opening file.");
  }
}   //END FUNCTION
/********************END FUNCTION DEFINITIONS********************/
