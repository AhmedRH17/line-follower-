#include <Arduino.h>
#include <L298N.h>
#include <BluetoothSerial.h>
#include <PID_v1.h>
#include "ManualControl.h"
#include "pidadjust.h"

// bluetouth var handling 

String bluetooth_setmaxspeed= "SS";
String bluetooth_ki= "KI";
String bluetooth_kd= "KD";
String bluetooth_kp= "KP";


bool man_or_pid ;
BluetoothSerial SerialBT;

int read_and_calc_error();

// Right motor dir
#define MOTOR_A1 19
#define MOTOR_A2 21
 // Left motor dir
#define MOTOR_B1 22 
#define MOTOR_B2 23
// PWM pins for speed control
#define ENA 12         
#define ENB 13


L298N leftmotor(ENA, MOTOR_A1,  MOTOR_A2);
L298N rightmotor(ENB, MOTOR_B1, MOTOR_B2);

int max_speed =200;

// sensor handling 
int sensor_pins[]   =   { 35 , 32 , 33 , 25 , 26};
int sensor_weights[] = { -2 , -1 , 0 , 1 ,2};
 
// PID Variables
double Setpoint = 0;  // Centered 
double Input;         // Current error
double Output;        // Correction to motor speeds

// PID 

float Kp = 2.0, Ki = 0.0, Kd = 1.0;  

PID linepid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

#define STATUS_LED 2  // Built-in LED on most ESP32 boards

void indicateStatus() {
    if (!SerialBT.hasClient()) {
        digitalWrite(STATUS_LED, millis() % 1000 < 500);  // Slow blink when no BT connection
    } else {
        digitalWrite(STATUS_LED, millis() % 200 < 100);   // Fast blink when connected
    }
}

void setup() {

  leftmotor.setSpeed(0);  // Stop motor A
  rightmotor.setSpeed(0);  // Stop motor B

// sensor pins decleration
for (int i= 0; i<5; i++){
  pinMode(sensor_pins[i],INPUT);
}

linepid.SetMode(AUTOMATIC);
linepid.SetOutputLimits(-(max_speed), max_speed); 
  Serial.begin(115200); 

  SerialBT.begin("ESP32_Control");  // Initialize Bluetooth
  Serial.println("Bluetooth Started");
  Serial.println("Device name: ESP32_Control");
  Serial.println("Waiting for connection...");

  }

void loop() {
//  Serial.println("working ");
  // detecting what mode it is 
  String command = "";
  if (SerialBT.available()) {
      while (SerialBT.available()){
     command += (char)SerialBT.read(); // Build the full command
      }
      Serial.print("Received Bluetooth command: ");
      Serial.println(command);
      
      if (command.startsWith("SM")) {
        if (command.length() > 2) {
          char mode = command.charAt(2);
          switch (mode)
          {
          case '0':
          {
           
            man_or_pid =0;
            processCommand(command);
           }
            break;
            case '1':
            man_or_pid =1;
            break;
          }
        }
     }
  


  if (man_or_pid==1){
  int error = read_and_calc_error();
  Input = error;

  // Add debug prints
  Serial.print("Error: ");
  Serial.println(error);

  linepid.Compute();
  
  // Print PID output
  Serial.print("PID Output: ");
  Serial.println(Output);

  int right_speed = max_speed - Output;
  int lift_speed =  max_speed + Output ;

  // Print motor speeds
  Serial.print("Left Speed: ");
  Serial.println(lift_speed);
  Serial.print("Right Speed: ");
  Serial.println(right_speed);

  lift_speed = constrain(lift_speed, 0, max_speed);
  right_speed = constrain(right_speed, 0, max_speed);

  leftmotor.setSpeed(lift_speed);
  rightmotor.setSpeed(right_speed);

  leftmotor.forward();
  rightmotor.forward();
  
}


if (man_or_pid==0){
   move_baby();
}
}

}
int read_and_calc_error()
{
  int total_error = 0;
  int sensors_on_line = 0;  // Counter for sensors detecting the line
  
  Serial.print("Sensors: ");
  for (int i = 0; i < 5; i++) {
    int sensor_value = !digitalRead(sensor_pins[i]);
    Serial.print(sensor_value);
    Serial.print(" ");
    total_error += sensor_value * sensor_weights[i];
    if(sensor_value == 1) {
      sensors_on_line++;
    }
  }
  Serial.println();

  // Handle off-line condition
  if(sensors_on_line == 0) {
    Serial.println("Line lost - stopping");
    leftmotor.setSpeed(0);
    rightmotor.setSpeed(0);
    return 0;  // or you could return a special value
  }

  // Handle sharp turns (when only outermost sensors detect)
  if(sensors_on_line == 1) {
    if(!digitalRead(sensor_pins[0])) {  // Far left sensor
      Serial.println("Sharp left turn detected");
      leftmotor.setSpeed(0);
      rightmotor.setSpeed(max_speed);
      return -4;  // Strong left turn
    }
    if(!digitalRead(sensor_pins[4])) {  // Far right sensor
      Serial.println("Sharp right turn detected");
      leftmotor.setSpeed(max_speed);
      rightmotor.setSpeed(0);
      return 4;   // Strong right turn
    }
  }

  Serial.print("Error: ");
  Serial.println(total_error);
  return total_error;
}