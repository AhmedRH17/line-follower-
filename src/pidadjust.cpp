#include <Arduino.h>
#include <BluetoothSerial.h>
#include <PID_v1.h>

extern int man_or_pid;

extern BluetoothSerial SerialBT;
extern int max_speed;
extern PID linepid; 
extern float Kp, Ki, Kd ; 

void processCommand(String command) {
  command.trim();
  command.toUpperCase(); 
    if (command.startsWith("SS")){
        int newSpeed = command.substring(2).toInt();
        if (newSpeed > 0 && newSpeed <= 255) {  
        {max_speed = newSpeed;
        newSpeed = command.substring(3).toInt();
        linepid.SetOutputLimits(-(max_speed), max_speed); 
    }}}

    else if (command.startsWith("KP")){
        Kp = command.substring(2).toFloat();
        linepid.SetTunings(Kp, Ki, Kd);
    }

        else if (command.startsWith("KD")){
        Kd = command.substring(2).toFloat();
        linepid.SetTunings(Kp, Ki, Kd);
    }
    
        else if (command.startsWith("KI")){
        Ki = command.substring(2).toFloat();
        linepid.SetTunings(Kp, Ki, Kd);
    }
    
}
