#include <Arduino.h>
#include <L298N.h>
#include <BluetoothSerial.h>
#include "ManualControl.h"

extern int man_or_pid;

extern BluetoothSerial SerialBT;
extern L298N leftmotor;
extern L298N rightmotor;
extern int max_speed;

void move_baby(){
char command;
 if (SerialBT.available()) {
     command = SerialBT.read();}
      
      switch (command)
      {
      case ('F'):
        leftmotor.setSpeed(max_speed);
        rightmotor.setSpeed(max_speed);
        leftmotor.forward();
        rightmotor.forward();
        break;
      case ('B'):
        leftmotor.setSpeed(max_speed);
        rightmotor.setSpeed(max_speed);
        leftmotor.backward();
        rightmotor.backward();
        break;
      case 'L':  
        leftmotor.setSpeed(max_speed / 2);  
        rightmotor.setSpeed(max_speed);
        leftmotor.backward();
        rightmotor.forward();
        break;
      case 'R':  
        leftmotor.setSpeed(max_speed / 2);  
        rightmotor.setSpeed(max_speed);
        leftmotor.forward();
        rightmotor.backward(); 
        break;

      default:
        leftmotor.setSpeed(0);
        rightmotor.setSpeed(0);
        leftmotor.stop();
        rightmotor.stop();
        break;
      }
  } 