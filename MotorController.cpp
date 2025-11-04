  #include <Arduino.h>
  #include <MotorController.h>
  MotorController::MotorController(){}
  void MotorController::init (int pin1a, int pin1b, int pin2a, int pin2b, int pinen1, int pinen2){
    in1a = pin1a; in1b = pin1b; in2a = pin2a; in2b = pin2b; en1 = pinen1; en2 = pinen2;
    pinMode(in1a, OUTPUT);
    pinMode(in1b, OUTPUT);
    pinMode(in2a, OUTPUT);
    pinMode(in2b, OUTPUT);
    pinMode(en1, OUTPUT);
    pinMode(en2, OUTPUT);

    digitalWrite(in1a, LOW); //logic control
    digitalWrite(in1b, LOW);
    digitalWrite(in2a, LOW);
    digitalWrite(in2b, LOW);

    digitalWrite(en1, HIGH); //pwm control
    digitalWrite(en2, HIGH);
  }
  void MotorController::forward(){
    digitalWrite(in1a, HIGH);
    digitalWrite(in1b, LOW);

    digitalWrite(in2a, HIGH);
    digitalWrite(in2b, LOW);
  }
  void MotorController::backward(){
    digitalWrite(in2b, HIGH);
    digitalWrite(in2a, LOW);

    digitalWrite(in2b, HIGH);
    digitalWrite(in2a, LOW);
  }
  void MotorController::stop(){
    digitalWrite(in1a, LOW);
    digitalWrite(in1b, LOW);

    digitalWrite(in2a, LOW);
    digitalWrite(in2b, LOW);
  }
  void MotorController::right(){}
  void MotorController::left(){}
