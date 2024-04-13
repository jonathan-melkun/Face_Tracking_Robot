#include "SerialTransfer.h"
#include "ServoEasing.hpp"
#include "SerialTransfer.h"
#include "AccelStepper.h"

#define servo1_pin 14
#define servo2_pin 15

#define stepPin 22
#define dirPin 21
bool not_moved_yet = false;
int maxStepperPosition = 9560/2;
int minStepperPosition = -9560/2;
int servoSpeed = 80;

ServoEasing Servo1;
ServoEasing Servo2;
SerialTransfer myTransfer;
AccelStepper stepper(1, stepPin, dirPin);

struct __attribute__((packed)) STRUCT {
  int servo1_move = 0;
  int servo2_move = 0;
  int stepper_move = 0;
} packet;

//used for debugging since I apparently cannot print anything other than a struct
struct __attribute__((packed)) STRUCT2 {
  int servo1_move = 333;
  int servo2_move = 333;
  int stepper_move = 333;
} debugStruct;

void setup()
{
  Servo1.attach(servo1_pin, 90, 1200, 1670);
  Servo2.attach(servo2_pin, 90, 1320, 1680);
  setSpeedForAllServos(servoSpeed);
  Servo1.setEasingType(EASE_LINEAR);
  Servo2.setEasingType(EASE_LINEAR);

  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1000);
  stepper.setCurrentPosition(0);
  Serial.begin(115200);
  myTransfer.begin(Serial);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
}

void loop()
{
  if(myTransfer.available())
  {
    uint16_t recSize = 0;
    recSize = myTransfer.rxObj(packet, recSize);
    not_moved_yet=true;
    /*
    //for debugging, sends received packet back to Python
    uint16_t sendSize = 0;
    sendSize = myTransfer.txObj(packet, sendSize);
    myTransfer.sendData(sendSize);
    */   
  }
  
  if (not_moved_yet) {
    // uses non-blocking for theoretical simultaneous movement of servos and stepper
    if (!Servo1.isMoving()){
      Servo1.setEaseTo(Servo1.getCurrentAngle()+packet.servo1_move);
      Servo2.setEaseTo(Servo2.getCurrentAngle()+packet.servo2_move);
      setEaseToForAllServosSynchronizeAndStartInterrupt(servoSpeed);
    }
    
    if (stepper.currentPosition()+packet.stepper_move < maxStepperPosition && stepper.currentPosition()+packet.stepper_move > minStepperPosition) {
      stepper.move(packet.stepper_move);
    }
    not_moved_yet=false;
  }
  stepper.run();
}