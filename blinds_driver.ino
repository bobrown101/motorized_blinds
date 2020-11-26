#include "PinChangeInterrupt.h"
#include "PIDController.h"

#define MOTOR_RIGHT 1
#define MOTOR_LEFT -1
#define MOTOR_OFF 0
#define MOTOR_LEEWAY 10

float MOTOR1_SKIP_BASE = 2;
float MOTOR2_SKIP_BASE = 1.5;


#define PIN_MOTOR_1_ENCODER_POS 11
#define PIN_MOTOR_1_ENCODER_NEG 12
#define PIN_MOTOR_1_MOTOR_POS 4
#define PIN_MOTOR_1_MOTOR_NEG 5

#define PIN_MOTOR_2_ENCODER_POS 9
#define PIN_MOTOR_2_ENCODER_NEG 10
#define PIN_MOTOR_2_MOTOR_POS 2
#define PIN_MOTOR_2_MOTOR_NEG 3

#define LOOP_TIME 5
// an increasing encoder value means its turning ccw
// a decreasing encoder value means its turing cw

volatile long motor1_currentEncoderPosition = 0;
long motor1_desiredPosition = 0;
long motor1_skipCount = 0;

volatile long motor2_currentEncoderPosition = 0;
long motor2_desiredPosition = 0;
long motor2_skipCount = 0;



void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);


  pinMode(PIN_MOTOR_1_ENCODER_POS, INPUT_PULLUP);
  pinMode(PIN_MOTOR_1_ENCODER_NEG, INPUT_PULLUP);
  pinMode(PIN_MOTOR_1_MOTOR_POS, OUTPUT);
  pinMode(PIN_MOTOR_1_MOTOR_NEG, OUTPUT);
  attachPCINT(digitalPinToPCINT(PIN_MOTOR_1_ENCODER_POS), motor1_encoderInterrupt, CHANGE);

  pinMode(PIN_MOTOR_2_ENCODER_POS, INPUT_PULLUP);
  pinMode(PIN_MOTOR_2_ENCODER_NEG, INPUT_PULLUP);
  pinMode(PIN_MOTOR_2_MOTOR_POS, OUTPUT);
  pinMode(PIN_MOTOR_2_MOTOR_NEG, OUTPUT);
  attachPCINT(digitalPinToPCINT(PIN_MOTOR_2_ENCODER_POS), motor2_encoderInterrupt, CHANGE);
}

void getDesiredPositions() {
  int charOffset = 48;
  if (Serial.available() > 0) {
    int motorChoice = Serial.read() - charOffset;
    long integerValue = 0;

    char incomingByte;
    while (1) {
      incomingByte = Serial.read();
      if (incomingByte == '\n') break;
      if (incomingByte == -1) continue;
      integerValue *= 10;
      integerValue = ((incomingByte - charOffset) + integerValue);
    }

    Serial.print("motor choice: ");
    Serial.println(motorChoice);
    if (motorChoice == 1) {
      motor1_desiredPosition = integerValue;
    } else if (motorChoice == 2) {
      motor2_desiredPosition = integerValue;
    }
  }
}

int skipCountReducer(int difference, float skipBase) {
  if (difference > 500) {
    return 0;
  } else if (difference > 400) {
    return pow(skipBase, 1);
  } else if (difference > 300) {
    return pow(skipBase, 2);
  } else if (difference > 200) {
    return pow(skipBase, 3);
  } else if (difference > 100) {
    return pow(skipBase, 4);
  } else if (difference > 50) {
    return pow(skipBase, 5);
  } else if (difference > 25) {
    return pow(skipBase, 6);
  } else if (difference > MOTOR_LEEWAY) {
    return pow(skipBase, 7);
  } else {
    return 0;
  }

}

int determineMotorPower(int desiredPosition, int currentPosition, long *skipCount, float skipBase) {
  int difference = desiredPosition - currentPosition;
  int absDiff = abs(difference);

  if (desiredPosition == currentPosition) {
    return MOTOR_OFF;
  } else if (absDiff < MOTOR_LEEWAY) {
    return MOTOR_OFF;
  } else if (*skipCount >= 1) {
    *skipCount = *skipCount - 1;
    return MOTOR_OFF;
  } else {
    *skipCount = skipCountReducer(difference, skipBase);
    if (difference > 0) {
      return MOTOR_RIGHT;
    } else {
      return MOTOR_LEFT;
    }
  }

}


void loop() {


  getDesiredPositions();

  int motor1_power = determineMotorPower(motor1_desiredPosition, motor1_currentEncoderPosition, &motor1_skipCount, MOTOR1_SKIP_BASE);
  turnMotor(motor1_power, PIN_MOTOR_1_MOTOR_POS, PIN_MOTOR_1_MOTOR_NEG);

  int motor2_power = determineMotorPower(motor2_desiredPosition, motor2_currentEncoderPosition, &motor2_skipCount, MOTOR2_SKIP_BASE);
  turnMotor(motor2_power, PIN_MOTOR_2_MOTOR_POS, PIN_MOTOR_2_MOTOR_NEG);



  if (motor1_power != 0 || motor2_power > 0 ) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
//
//  Serial.print(motor1_currentEncoderPosition);
//  Serial.print(", ");
//  Serial.println(motor2_currentEncoderPosition);

  //  Serial.print("motor1_power: ");
  //  Serial.print(motor1_power);
  //  Serial.print(", motor1_currentEncoderPosition: ");
  //  Serial.print(motor1_currentEncoderPosition);
  //  Serial.print(", motor1_desiredPosition: ");
  //  Serial.println(motor1_desiredPosition);

  //  Serial.print("                                                                                 motor2_power: ");
  //  Serial.print(motor2_power);
  //  Serial.print(", motor2_currentEncoderPosition: ");
  //  Serial.print(motor2_currentEncoderPosition);
  //  Serial.print(", motor2_desiredPosition: ");
  //  Serial.println(motor2_desiredPosition);
  delay(LOOP_TIME);
}



void turnMotor(int power, int posPin, int negPin) {
  if (power == MOTOR_OFF) {
    digitalWrite(posPin, LOW);
    digitalWrite(negPin, LOW);
  } else if (power == MOTOR_RIGHT) {
    analogWrite(posPin, 255);
    digitalWrite(negPin, LOW);
  } else {
    analogWrite(negPin, 255);
    digitalWrite(posPin, LOW);
  }

}

void motor1_encoderInterrupt(void) {
  bool goingClockwise = digitalRead(PIN_MOTOR_1_ENCODER_POS) == digitalRead(PIN_MOTOR_1_ENCODER_NEG);
  if (goingClockwise) {
    motor1_currentEncoderPosition++;
  } else {
    motor1_currentEncoderPosition--;
  }
}

void motor2_encoderInterrupt(void) {
  bool goingClockwise = digitalRead(PIN_MOTOR_2_ENCODER_POS) == digitalRead(PIN_MOTOR_2_ENCODER_NEG);
  if (goingClockwise) {
    motor2_currentEncoderPosition++;
  } else {
    motor2_currentEncoderPosition--;
  }
}
