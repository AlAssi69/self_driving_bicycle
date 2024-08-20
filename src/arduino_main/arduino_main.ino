#include <Wire.h>
#include <AccelStepper.h>

#define SLAVE_ADDRESS 9
#define ANSWER_SIZE 3

String yaw_string = "";
char yaw_char = 'b';

class DCMotor
{
    int en_r, en_l, pwm_r, pwm_l;

  public:
    DCMotor(int en_r = 8, int en_l = 5, int pwm_r = 3, int pwm_l = 6)
    {
      this->en_r = en_r;
      this->en_l = en_l;
      this->pwm_r = pwm_r;
      this->pwm_l = pwm_l;

      pinMode(this->en_r, OUTPUT);
      pinMode(this->en_l, OUTPUT);
      pinMode(this->pwm_r, OUTPUT);
      pinMode(this->pwm_l, OUTPUT);

      digitalWrite(this->en_r, HIGH);
      digitalWrite(this->en_l, HIGH);
    }

    void move(int speed = 35)
    {
      analogWrite(this->pwm_l, speed);
    }
};

class StepperMotor
{
    int motorInterfaceType, directionPin, pulsePin, maxSpeed, acceleration;
    AccelStepper sm;

  public:
    StepperMotor(int motorInterfaceType = 1, int directionPin = 9, int pulsePin = 10, int maxSpeed = 1000, int acceleration = 500)
    {
      this->motorInterfaceType = motorInterfaceType;
      this->directionPin = directionPin;
      this->pulsePin = pulsePin;
      this->maxSpeed = maxSpeed;
      this->acceleration = acceleration;

      this->sm = AccelStepper(this->motorInterfaceType, this->pulsePin, this->directionPin);
      this->sm.setMaxSpeed(this->maxSpeed);
      this->sm.setAcceleration(this->acceleration);
    }

    void move(int angle)
    {
      double factor = 1.8;
      int steps = angle / factor;
      this->sm.moveTo(steps);
      this->sm.runToPosition();
    }
};

volatile int count = 0;
const int sensorPin = 2
volatile unsigned long lastTime = 0;  
const unsigned long debounceDelay = 50; // Set an appropriate debounce time (in milliseconds)  

DCMotor dc;
StepperMotor stepper;

int speed;
int angle;

void countRevolutions() {  
    unsigned long currentTime = millis();  
    if (currentTime - lastTime >= debounceDelay) {  
        count++; // Increase count on each valid detection  
        lastTime = currentTime; // Update lastTime to current time  
    }  
}

void setup()
{
  //attachInterrupt(digitalPinToInterrupt(ir.inputPin), ir.countRevolutions, ir.edge);
  //Wire.begin();

  pinMode(sensorPin, INPUT);  
  attachInterrupt(digitalPinToInterrupt(sensorPin), countRevolutions, RISING);  
  while (!Serial)
  {
  }

  Serial.begin(9600);
  Serial.println("Starting...");
}

String in = "";           // a variable to hold the input from the serial
String* in_split;         // this is used to split the input (e.g., 90 90 90) and hold the 3 values
int qtde;                 // don't mind this variable (it holds thecount)

// split function
String* split(String& v, char delimiter, int& length)
{
  length = 1;
  bool found = false;

  // Figure out how many itens the array should have
  for (int i = 0; i < v.length(); i++) {
    if (v[i] == delimiter) {
      length++;
      found = true;
    }
  }

  // If the delimiter is found than create the array
  // and split the String
  if (found) {

    // Create array
    String* valores = new String[length];

    // Split the string into array
    int i = 0;
    for (int itemIndex = 0; itemIndex < length; itemIndex++) {
      for (; i < v.length(); i++) {

        if (v[i] == delimiter) {
          i++;
          break;
        }
        valores[itemIndex] += v[i];
      }
    }

    // Done, return the values
    return valores;
  }

  // No delimiter found
  return nullptr;
}

void loop()
{
  if(Serial.available()>0)
  {
    in = Serial.readString();
    in.trim();
    Serial.println(in);
    in_split = split(in, ' ', qtde);

    speed = in_split[0].toInt();
    dc.move(speed);

    angle = in_split[1].toInt();
    stepper.move(angle);

    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.print("\tAngle: ");
    Serial.println(angle);

    /*
    Wire.requestFrom(SLAVE_ADDRESS, ANSWER_SIZE);

    while (Wire.available())
    {
      yaw_char = Wire.read();
      yaw_string += yaw_char;
    }

    Serial.print("Yaw: ");
    Serial.print(yaw_string);
    */
    Serial.print("\tCount: ");
    Serial.println(count / 8);
    
  } 
  
    delay(10);
}
