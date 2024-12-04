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


bool man_or_pid = 1;
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
int sensor_pins[]   =   { 5 , 6 , 7 , 8 , 9};
int sensor_weights[] = { -2 , -1 , 0 , 1 ,2};
 
// PID Variables
double Setpoint = 0;  // Centered 
double Input;         // Current error
double Output;        // Correction to motor speeds

// PID 

float Kp = 2.0, Ki = 0.0, Kd = 1.0;  

PID linepid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  leftmotor.setSpeed(0);  // Stop motor A
  rightmotor.setSpeed(0);  // Stop motor B

// sensor pins decleration
for (int i= 0; i<5; i++){
  pinMode(sensor_pins[i],INPUT);
}

linepid.SetMode(AUTOMATIC);
linepid.SetOutputLimits(-(max_speed), max_speed); 

  SerialBT.begin("ESP32_Control"); 
  Serial.begin(115200); 
  }

void loop() {
  
  // detecting what mode it is 
  String command = "";
  if (SerialBT.available()) {
      while (SerialBT.available()){
     command += (char)SerialBT.read(); // Build the full command
      }
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
  }  

  processCommand();

  if (man_or_pid==1){
  int error = read_and_calc_error();
  // Set the PID input to the calculated error
  Input = error;

  // Compute PID control output
  linepid.Compute();
  int right_speed = max_speed - Output;
  int lift_speed =  max_speed + Output ;

  lift_speed = constrain(lift_speed, 0, max_speed);
  right_speed = constrain(right_speed, 0, max_speed);

  leftmotor.setSpeed(lift_speed);
  rightmotor.setSpeed(right_speed);

  leftmotor.forward();
  rightmotor.forward();
  
}
}

if (man_or_pid==0){
   move_baby();
}

int read_and_calc_error(){
  int current_error= 0;
  int total_error= 0;
for (int i= 0; i<5; i++){
  current_error = ( digitalRead(sensor_pins[i]) ) *sensor_weights[i];
  total_error += current_error;
}
return total_error;
}