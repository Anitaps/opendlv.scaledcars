//For: DIT168 Group 8
//Author: Axel Verner, 

#include <Smartcar.h>

#define US_C 0x73 //Connector farthest to the north-west if reset-button is pointing south / to the rear of the car.
#define ODOMETER_PIN 3
#define SIDE_FRONT_PIN A0
#define SIDE_BACK_PIN A1
#define BACK_PIN A2
#define STEER_PIN 11
#define ESC_PIN 10
#define CH_1 2
#define CH_2 4

//Hardware components used

SRF08 US_center;
GP2D120 sideFrontIR, sideBackIR, backIR;
Odometer odometer;
Servo steerServo, motorServo;

//Variables used.

unsigned long odo_val = 0;
int us_c, ir_sf, ir_sb, ir_b, odom;
const int center = 1500;
String sensOutput;
bool remote_control = 0;
int ch1, ch2;
int steerVal, throttleVal = 1500;



//Assigns sensor variables to their pins respectively.

void setup() {

  odometer.attach(ODOMETER_PIN);
  odometer.begin();

  US_center.attach(US_C);

  sideFrontIR.attach(SIDE_FRONT_PIN);
  sideBackIR.attach(SIDE_BACK_PIN);
  backIR.attach(BACK_PIN);
  
  pinMode(CH_1, INPUT);
  pinMode(CH_2, INPUT);

  pinMode(STEER_PIN,OUTPUT);  
  pinMode(ESC_PIN,OUTPUT);
  
  steerServo.attach(STEER_PIN);
  motorServo.attach(ESC_PIN);

  Serial.begin(9600);

//Waits for an input (' ' for example) before it sends the defaults to the ESC. This way we don't get accidental armings.

  while(Serial.available() == 0) {

    Serial.println("ESC has not been turned on..");
    delay(1000);
  }
  
  arm_esc(steerServo, motorServo);
}


//Arming sequence that sends default values to the ESC, so that it will know how to interpret data sent later on.

void arm_esc(Servo steer, Servo motor){
  steer.write(100);
  delay(500);
  motor.writeMicroseconds(center+10);
  delay(500);
  Serial.println("< ESC ARMED >");
}


//Main loop that checks if it should read from either RX pin or Serial. 

void loop() {
  
    ch1 = pulseIn(CH_1, HIGH); //Read ch1 input
    if (ch1 == 0) {  
        serial_control();
    } else {
        RXTX_control();
    }

    process_sensors();
}


//Method that processes RX/TX data (from remote control). 
//The method reads data from RX pins and directly and sends it to responsible methods.

void RXTX_control() {
    if(remote_control == false) arm_esc(steerServo, motorServo);
    remote_control = true;
  
    ch1 = pulseIn(CH_1, HIGH);
    ch2 = pulseIn(CH_2, HIGH);

    Serial.print("Channel 1:");
    Serial.println(ch1);
    Serial.print("Channel 2:");
    Serial.println(ch2);
    Serial.println();

    steerVal = ch1;
    steerRXTX(steerVal);

    throttleVal = ch2;
    throttle(throttleVal);
}


//Method that writes steering values to the servo. This is used when sending steering from serial, where we also set min/max values.

void steer(int angle) {

    if(angle <= 150 && angle >= 45){
      steerServo.write(angle);
//      Serial.print("ANGLE: ");
//      Serial.println(angle);
    } else{
//      Serial.println("------------------------------");
//      Serial.print("ROUGUE DATA, ANGLE ");
//      Serial.println(angle);
    }
}


//Method to send raw steering data received from the receiver(AKA, data from remote control).

void steerRXTX(int angle){
    steerServo.write(angle);
}


//Writing 'throttle' to the ESC pin. We also set max/min values the arduino is allowed to send to ESC for safety.
//Commented lines is for debugging, left commented now so the serial won't be flooded.

void throttle(int throttle) {

    if(throttle <= 1650 && throttle >= 1350){
      motorServo.writeMicroseconds(throttle);
//      Serial.print("THROTTLE ");
//      Serial.println(throttle);
    } else{
//      Serial.println("------------------------------");
//      Serial.print("ROUGUE DATA, THROTTLE ");
//      Serial.println(throttle);
    }
}



//Method that reads the serial input sent from a computer connected via USB.

void serial_control() {

  //If it's switching from RXTX input to serial (USB), then redo arming of the ESC).
  
    if(remote_control == true) arm_esc(steerServo, motorServo);
    remote_control = false;

  //Telling the serial to have a time-out so that it won't wait after reading a message.

    Serial.setTimeout(1);

  //Reads data from the serial buffer and looks for delimiters to find the values we want to send to steering/throttle.
    
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
      if (input.startsWith("t")) { //throttle
          throttle(input.substring(input.indexOf('t') + 1, input.indexOf('s')).toInt());
          steer(input.substring(input.indexOf('s') + 1, input.indexOf('\n')).toInt());
      }else if (input.startsWith("s")){
          steer(input.substring(input.indexOf('s') + 1, input.indexOf('\n')).toInt());
      }else if (input.startsWith("b")){
          brake();
      }
    }
   
}



//Pulling sensor data from each sensor, then assigning the values to variables we use when sending to serial.
//Commented segments is for debugging individual sensors.

void process_sensors(){

//    Serial.print(",IR1"); //Side front
    ir_sf = sideFrontIR.getDistance();
//    Serial.println(ir_sf);
//    Serial.println();

//    Serial.print(",IR2"); //Side back
    ir_sb = sideBackIR.getDistance();
//    Serial.println(ir_sb);
//    Serial.println();

//    Serial.print(",IR3"); //Back
    ir_b = backIR.getDistance();
//    Serial.println(ir_b);
//    Serial.println();

//    Serial.print(",US");
    us_c = US_center.getDistance();
//    Serial.println(us_c);
//    Serial.println();

//    Serial.print(",Odo");
    odo_val= odometer.getDistance();
//    Serial.println(odo_val);
//    Serial.println();


//Concatenating the sensor output into a string. 
//We also add delimiters and padding to keep the sent data uniform.

sensOutput='H';
if(ir_sf<10) sensOutput += 0;
sensOutput+=ir_sf;
sensOutput+='a';
if(ir_sb<10) sensOutput += 0;
sensOutput+=ir_sb;
sensOutput+='b';
if(ir_b<10) sensOutput += 0;
sensOutput+=ir_b;
sensOutput+='c';
if(us_c<100) sensOutput += 0;
if(us_c<10) sensOutput += 0;
sensOutput+=us_c;
sensOutput+='d';
if(odo_val<100) sensOutput += 0;
if(odo_val<10) sensOutput += 0;
sensOutput+=odo_val;
sensOutput+='T';

Serial.println(sensOutput);

}


//Just a simple method that send '1500' to the ECS, telling it to stop.

void brake(){
    throttle(1500);
}
