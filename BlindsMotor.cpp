//#include "BlindsMotor.h"
//#include "PinChangeInterrupt.h"
//
//
//BlindsMotor::BlindsMotor () {
//  int pinEncoderPos;
//  int pinEncoderNeg;
//  int pinMotorPos;
//  int pinMotorNeg;
//  PIDController pidController;
//  volatile long currentEncoderPosition;
//  int minDriveValue;
//  long pidResult;
//  long desiredPosition;
//}
//
////void encoderIntteruptCallback (int thisRef)
//
//void BlindsMotor::setupMotor(int PIN_ENCODER_POS, int PIN_ENCODER_NEG, int PIN_MOTOR_POS, int PIN_MOTOR_NEG) {
//  minDriveValue = 10;
//  pinEncoderPos = PIN_ENCODER_POS;
//  pinEncoderNeg = PIN_ENCODER_NEG;
//  pinMotorPos = PIN_MOTOR_POS;
//  pinMotorNeg = PIN_MOTOR_NEG;
//  pinMode(pinEncoderPos, INPUT_PULLUP);
//  pinMode(pinEncoderNeg, INPUT_PULLUP);
//  pinMode(pinMotorPos, OUTPUT);
//  pinMode(pinMotorNeg, OUTPUT);
//  attachPCINT(digitalPinToPCINT(pinEncoderPos), [&]() {
//    Serial.println(pinEncoderPos);
////    Serial.println(this->pinEncoderPos);
////        bool goingClockwise = digitalRead(this->pinEncoderPos) == digitalRead(this->pinEncoderNeg);
//    //    if (goingClockwise) {
//    //      currentEncoderPosition++;
//    //    } else {
//    //      currentEncoderPosition--;
//    //    }
//  }
//  , CHANGE);
//}
//
//
//void BlindsMotor::goToPosition(int DESIRED_POSITION) {
//  desiredPosition = DESIRED_POSITION;
//  pidController.setpoint(desiredPosition);
//  pidResult = pidController.compute(currentEncoderPosition);
//  this->turnMotor(pidResult, pinMotorPos, pinMotorNeg);
//}
//
//
//void BlindsMotor::turnMotor(int power, int posPin, int negPin) {
//  if (abs(power) < minDriveValue) {
//    return;
//  }
//
//  if (power > 0) {
//    analogWrite(posPin, power);
//    digitalWrite(negPin, LOW);
//  } else {
//    analogWrite(negPin, abs(power));
//    digitalWrite(posPin, LOW);
//  }
//}
//
