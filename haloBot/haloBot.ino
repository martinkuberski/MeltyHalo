
#include <Adafruit_DotStar.h>
#include <SPI.h>
#include <i2c_t3.h>
#include <PulsePosition.h>
#include <Servo.h>
//#include <Wire.h>

#define enablePin 21
#define PIN_IR 15
#define motor1pin 23
#define motor2pin 22
#define vBatt A0

//animations
unsigned long nextFrameAt = 0;

struct Frame {
  uint16_t duration;//in degrees or milliseconds
  struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
  } led[5];
  struct Frame *next;
} *animationHead, *currentFrame, *idleHead, *tankHead, *spinHead;

void defineAnimations(void);
void resetStaticAnimation(void);
void resetDynamicAnimation(void);

//serial
#define SERIAL_WAIT 0
#define SERIAL_PACKETSTART 1
byte serialState = 0;
unsigned long packetTime = 0;
unsigned long packetTimeout = 200;
int packet[6];
int bytesRead = 0;

unsigned long lastReceived = 0;
uint8_t failsafeCount = 0;

//controls
byte flip = 0;
int16_t thumbX = 0;
int16_t thumbY = 0;
uint16_t throt = 0;
int16_t head = 0;
byte en = 0;

//motors
Servo motor1;
Servo motor2;

//leds
Adafruit_DotStar strip = Adafruit_DotStar(5, DOTSTAR_GBR);

//**********************//
// MELTYBRAIN VARIABLES //
//**********************//
int16_t angle = 0;//LSB is one degree. Our current heading

int16_t meltyAngle = 0;//the commanded bearing angle for meltybrain control
uint16_t meltyThrottle = 0;

#define BEACON_SENSING 0x01//if this is defined, we are angle sensing using only the infrared receiver
#define ACCEL_SENSING 0x02//if this is defined, we are angle sensing using only the accelerometer
#define HYBRID_SENSING 0x03//if this is defined, we are angle sensing using both the beacon and the accelerometer
uint8_t senseMode = ACCEL_SENSING;

//BEACON
boolean beacon = false;//this variable keeps track of the status of the beacon internally. If this variable and the digital read don't match, it's a rising or falling edge

unsigned long beaconEdgeTime[2];//this is the array of rising edge acquisition times. We keep some history for better extrapolation

bool beaconEnvelopeStarted = false;
unsigned long beaconHoldTime;
#define BEACON_DEBOUNCE_TIME 2000//in microseconds
uint8_t beaconEdgesRecorded = 0;//this keeps track of how many beacon pulses we've seen, up to APPROXIMATION_ORDER. It resets when a revolution takes too long
#define REV_TIMEOUT 2000 //this (in ms) is how long a revolution can take before we say the robot is spinning too slowly to control and reset the algorithm

//ACCELEROMETER
void configAccelerometer(void);

//states
uint8_t state = 1;

#define STATE_IDLE 1
#define STATE_TANK 2
#define STATE_SPIN 3

void pollSerial(void);
void receivePacket(void);

void shiftToLEDs(void);
void runStaticAnimation(void);
void runDynamicAnimation(void);

void runMeltyBrain(void);

uint16_t getBatteryVoltage() { //returns voltage in millivolts
  return (analogRead(vBatt)*49)/3;
}


void setMotorSpeed(Servo motorObj, int spd) {
  int motorSpd;
  spd = constrain(spd, -100, 100);//make sure our speed value is valid. This lets us be lazier elsewhere
  
  //apply a deadband
  if(spd < 5 && spd > -5) spd = 0;

  if(&motorObj == &motor1) spd *= -1;

  //map to servo
  motorSpd = map(spd, -100, 100, 0, 180);
  
  //apply min and max throttle
  if(motorSpd > 165) motorSpd = 165;
  else if (motorSpd < 19) motorSpd = 19;

  //send servo data to ESC
  motorObj.write(map(spd, -100, 100, 0, 180));
}

void goIdle() {
  state = STATE_IDLE;

  //digitalWrite(enablePin, HIGH);
  setMotorSpeed(motor1, 0);
  setMotorSpeed(motor2, 0);
  
  animationHead = idleHead;
  resetStaticAnimation();
}

void goTank() {
  state = STATE_TANK;

  digitalWrite(enablePin, LOW);
  
  animationHead = tankHead;
  resetStaticAnimation();
}

void goSpin() {
  state = STATE_SPIN;

  digitalWrite(enablePin, LOW);

  beaconEdgesRecorded = 0;

  animationHead = spinHead;
  currentFrame = animationHead;
  nextFrameAt = currentFrame->duration + angle;
  shiftToLEDs();
}

void feedWatchdog() {
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}

//this runs if the robot code hangs! cut off the motors
void watchdog_isr() {
  digitalWrite(enablePin, HIGH);
}

PulsePositionInput RC;


void setup() {
  Serial.begin(57600);
  RC.begin(5);
  motor1.attach(motor1pin);
  motor2.attach(motor2pin);
  SPI.begin();
  pinMode(enablePin, OUTPUT);
  pinMode(13, OUTPUT); //Teensy LED output
  digitalWrite(13, HIGH);
  digitalWrite(enablePin, HIGH);

  pinMode(PIN_IR, INPUT);

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 1800000, I2C_OP_MODE_IMM);//1.8MHz clock rate
  //Wire.begin();

  //SETUP WATCHDOG
  //settings taken from: https://bigdanzblog.wordpress.com/2017/10/27/watch-dog-timer-wdt-for-teensy-3-1-and-3-2/
  noInterrupts();
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  delayMicroseconds(1);

  WDOG_TOVALH = 0x006d; //1 second timer
  WDOG_TOVALL = 0xdd00;
  WDOG_PRESC = 0x400;
  WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN | 
                  WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN |
                  WDOG_STCTRLH_CLKSRC | WDOG_STCTRLH_IRQRSTEN;
  interrupts();

  NVIC_ENABLE_IRQ(IRQ_WDOG);//enable watchdog interrupt

  //analogWriteFrequency(3, 250);//this changes the frequency of both motor outputs

  configAccelerometer();

  //build the animations
  defineAnimations();

  goIdle();
}

//DEBUG: ACCELEROMETER
int16_t zAccelDBG;
int16_t yAccelDBG;
uint16_t accelAngleDBG = 0;

void loop() {

  //Bark bark
  feedWatchdog();

  //check for incoming messages
  pollSerial();

  //make sure comms haven't timed out - failsafe values on FS-I6 set as Ch1:43%, Ch2: -46%, Ch3: -98%, Ch4: -55%, Ch5: 100%, Ch6: 30%
  if(throt == 1 && thumbX == 43 && thumbY == -46 && head == 53) failsafeCount++;
  else failsafeCount = 0;
  if(failsafeCount >= 10 && state != STATE_IDLE) {
    en = 0x0;
    goIdle();
    Serial.println("Transmitter connection error, engaging failsafe...");
  }
  //DEBUG: ACCELEROMETER
  runAccelerometer();
  Serial.print("YACC: ");
  Serial.print(yAccelDBG);
  Serial.print(" ZACC: ");
  Serial.print(zAccelDBG);
  Serial.print(" ANGLE: ");
  Serial.println(accelAngleDBG);
 
  /*
  //check if battery voltage is below 3.2V/cell cutoff (2.5V/cell under load)
  uint16_t batteryReading  = getBatteryVoltage();
  if((throt == 0 && batteryReading < 3200*4)) {
    //disable motors
    digitalWrite(enablePin, HIGH);
    //blank LEDs
    strip.clear();
    strip.show();
    //sit until power is removed
    while(true) feedWatchdog();
  }
  */

  switch(state) {
    case STATE_IDLE:
      runStaticAnimation();

      if(en == 0xAA) {
        goTank();
      }
      
      break;
    case STATE_TANK:

      setMotorSpeed(motor1, thumbY+thumbX/2);
      setMotorSpeed(motor2, thumbY-thumbX/2);
    
      runStaticAnimation();
      
      if(throt > 2) {
        goSpin();
      }

      if(en != 0xAA) {
        goIdle();
      }
      break;
    case STATE_SPIN:

      runMeltyBrain();//manage all of the sensors and predict our current heading
     
      runDynamicAnimation();
      
      if(throt < 2) {
        goTank();
      }

      if(en != 0xAA) {
        goIdle();
      }
    default:
      break;
  }

  //Teensy LED
  if(micros() % 3000 == 0) digitalWrite(13, HIGH);
  if(micros() % 5000 == 0) digitalWrite(13, LOW);
}
