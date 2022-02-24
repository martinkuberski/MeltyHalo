#include <PulsePosition.h>
#include <Servo.h>

PulsePositionInput RC;
Servo ESC;

//var declarations
int headAngle;
#define SERIAL_WAIT 0
#define SERIAL_PACKETSTART 1
byte serialState = 0;
unsigned long packetTime = 0;
unsigned long packetTimeout = 200;
int packet[6];
int bytesRead = 0;
unsigned long lastReceived = 0;
byte flip = 0;
int16_t thumbX = 0;
int16_t thumbY = 0;
uint16_t throt = 0;
int16_t head = 0;
byte en = 0;
int16_t angle = 0;//LSB is one degree. Our current heading
int16_t meltyAngle = 0;//the commanded bearing angle for meltybrain control
uint16_t meltyThrottle = 0;
uint8_t state = 3;
#define STATE_IDLE 1
#define STATE_TANK 2
#define STATE_SPIN 3

void setup() {
  Serial.begin(9600);
  //if(!ESC.armed) ESC.armMotorESC();
  RC.begin(5);
  ESC.attach(22);
  pinMode(13, OUTPUT);
}

void pollSerial() {
  while(RC.available() == 8) {
    if(serialState == SERIAL_WAIT) {
        serialState = SERIAL_PACKETSTART;
        bytesRead = 0;
        continue;
    }
    packet[bytesRead] = RC.read(bytesRead + 1);
    bytesRead++;

    if(bytesRead == 6) {
      receivePacket();
      bytesRead = 0;
      serialState = SERIAL_WAIT;
      break;
    }    
  }
}
bool tankOverride;
void receivePacket() {
  lastReceived = micros();
  
  tankOverride = map((uint16_t) packet[4], 1000, 2000, false, true);//this set the throttle to 0, forcing the bot into tank mode. Faster than adjusting the throttle pot.
  //thumbstick X
  thumbX = map((uint16_t) packet[0], 1000, 2000, -100, 100);
  //thumbstick Y
  thumbY = map((uint16_t) packet[1], 1000, 2000, -100, 100);
  //throttle
  if(tankOverride) {
    throt = 0;      
  } else {
    throt = map((uint16_t) packet[2], 1000, 2000, 0, 100);
  }
  //heading - 
  head = map((uint16_t) packet[5], 1000, 2000, -180, 180);
  if(head > -15 && head < 15) head = 0;
  //enable
  //en = packet[9];
  en = 1;
  if(state == STATE_SPIN) {
    //calculate the commanded direction and speed
   meltyThrottle = sqrt(thumbX*thumbX + thumbY*thumbY)/2;
   int16_t calcAngle = (int16_t) (atan2((double) thumbY, (double) thumbX)*180.0/PI);
   if(calcAngle < 0) calcAngle += 360;
   meltyAngle = (uint16_t) calcAngle;
  }
}
int motorSpd;
int failsafe;
void loop() {
  Serial.print("Channels available: ");
  Serial.print(RC.available());
  pollSerial();
  headAngle = meltyAngle + head;
  if(headAngle < 0) headAngle += 360;
  if(headAngle > 360) headAngle -= 360;
  Serial.print(" Last received: ");
  Serial.print(lastReceived);
  Serial.print(" Tank override: ");
  Serial.print(tankOverride);
  Serial.print(" Throttle: ");
  Serial.print(throt);
  Serial.print(" X: ");
  Serial.print(thumbX);
  Serial.print(" Y: ");
  Serial.print(thumbY);
  Serial.print(" Angle: ");
  Serial.print(meltyAngle);
  Serial.print(" Heading: ");
  Serial.print(head);
  Serial.print(" Adjusted angle: ");
  Serial.print(headAngle);
  Serial.println();

  //Teensy LEDs
  if(micros() % 3000 == 0) digitalWrite(13, HIGH);
  if(micros() % 5000 == 0) digitalWrite(13, LOW);
  
  //Motor control
  if(tankOverride) {
    motorSpd = map(thumbY,-100,100,0,180);
    if(motorSpd > 165) motorSpd = 165;
    else if(motorSpd < 19) motorSpd = 19;
    ESC.write(motorSpd);
    }
    else ESC.write(90);

  //Failsafe
  if(tankOverride && throt == 1 && thumbX == 43 && thumbY == -46 && head == 53) failsafe++;
  if(failsafe == 10) {
    Serial.println("CONNECTION TERMINATED, FAILSAFE ENGAGED!");
    while(true);
    }
  }
