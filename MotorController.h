#ifndef MOTORCONTROLLER_H   
#define MOTORCONTROLLER_H
class MotorController {
  private: 
    int in1a;
    int in1b;
    int in2a;
    int in2b;
    int en1;
    int en2;
  public: 
    MotorController();
    void init (int pin1a, int pin1b, int pin2a, int pin2b, int pinen1, int pinen2);
    void forward();
    void backward();
    void right();
    void left();
    void stop();
};

#endif  // MOTORCONTROLLER_H