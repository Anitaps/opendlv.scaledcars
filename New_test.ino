#include <Smartcar.h>
#include <Arduino.h>
#include <SerialControl.h>

#define STEER_PIN 11
#define ESC_PIN 10
#define CH_1 2
#define CH_2 4

//Signal ranges
const int low = 1000;
const int high = 2000;

const int deadLow = 1450;
const int deadHigh = 1520;

const int center = 1500;

const int steerJitterL = 1450;
const int steerJitterH = 1530;

//Servo ranges: 80deg/left , 120deg/center , 160deg/right

int func_is_changed = -1;

int direction_int = 0;

Servo steerServo, motorServo;
int ch1, ch2;
int steerVal, throttleVal = 1500;

int spd;
int ang;

void setup() {
  pinMode(CH_1, INPUT);
  pinMode(CH_2, INPUT);

  pinMode(STEER_PIN,OUTPUT);  
  pinMode(ESC_PIN,OUTPUT);
  
  steerServo.attach(STEER_PIN);
  motorServo.attach(ESC_PIN);

  Serial.begin(9600);
  
  arm_esc(steerServo, motorServo);
}

void arm_esc(Servo steer, Servo motor){
  steer.write(90);
  delay(500);
  motor.writeMicroseconds(center+10);
  delay(500);
  Serial.println("< ESC ARMED >");
}

void loop() {
  
    ch1 = pulseIn(CH_1, HIGH); //Read ch1 input
    if (ch1 == 0) {  
        serial_control();
        cleanUp();
    } else {
        RXTX_control();
    }
}

void RXTX_control() {
    if((func_is_changed == 0) && (throttleVal < steerJitterL || throttleVal > steerJitterH)) brake;
    if(func_is_changed == 0) arm_esc(steerServo, motorServo);
    func_is_changed = 1;
  
    ch1 = pulseIn(CH_1, HIGH);
    ch2 = pulseIn(CH_2, HIGH);

    Serial.print("Channel 1:");
    Serial.println(ch1);
    Serial.print("Channel 2:");
    Serial.println(ch2);
    Serial.println();

    steerVal = ch1;
    steer(steerVal);

    throttleVal = ch2;
    throttle(throttleVal);
}

void steer(int angle) {
    if (angle >= low && angle <= deadLow) {
        angle = map(angle, low, deadLow, 0, 90);
    } else if (angle == center) {
        angle = 90;
    } else if (angle >= deadHigh && angle <= high) {
        angle = map(angle, deadHigh, high, 90, 180);
    }
    steerServo.write(angle);
}

void throttle(int throttle) {
  
    if (throttle >= low && throttle <= deadLow) {
        throttle = map(throttle, low, steerJitterL, 1050, deadLow);
        direction_int = 1;
    } else if (throttle == center) {
        throttle = center;
    } else if (throttle >= deadHigh && throttle <= high) {
        throttle = map(throttle, steerJitterH, high, 1660, 1750);
        direction_int = 0;
    }
    motorServo.writeMicroseconds(center);
    motorServo.writeMicroseconds(throttle);
    Serial.print("THROTTLE ");
    Serial.println(throttle);
}


void serial_control() {
  //TODO: Continue build of manual control
  
    if((func_is_changed == 1) && (throttleVal < steerJitterL || throttleVal > steerJitterH)) brake;
    if(func_is_changed == 1) arm_esc(steerServo, motorServo);
    func_is_changed = 0;
    
    if (Serial.available() > 2 ) {
      String input = Serial.readStringUntil('\n');
//      Serial.println("Input");
//      Serial.println(input);
      SerialControl serialc(input);
      String p1 = serialc.getValue('_', 0);
      String p2 = serialc.getValue('_', input.indexOf('_') + 1);
      spd = p1.toInt();
      ang = p2.toInt();
      if(spd >= 1350 && spd <= 1650){
        throttle(spd);
      }
//      Serial.println("Angle");
//      Serial.println(ang);
      if(ang >= 70 && ang <= 120){
        steerServo.write(ang);
      }
      
    }

}

void cleanUp(){
  while(Serial.available() > 0){
    Serial.read();
  }
}

void brake(){
  //TODO: Improve this
    throttle(1500);
}
