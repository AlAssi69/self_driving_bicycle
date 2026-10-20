#include <PID_v1.h>
#include "DCMotor.h"
#include "IRSensor.h"

double rev = 0;

String in = "";
String *in_split;
int qtde;
int motorSpeed = 0;

int irCount = 0;
void countRev();

String *split(String &v, char delimiter, int &length);
void plot(String label, float value, bool isLastGraph = false);

DCMotor dc;
IRSensor ir(countRev, 2, RISING, 95);

// Define Variables we'll be connecting to
double setPoint, input, output;

// Specify the links and initial tuning parameters
double kp = 2, ki = 0, kd = 0;
PID motorPID(&input, &output, &setPoint, kp, ki, kd, DIRECT);

void setup()
{
  while (!Serial)
  {
    ;
  }

  Serial.begin(57600);
  // Serial.setTimeout(3000);

  input = 0;
  setPoint = rev * 8;
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(0, 128);
}

void loop()
{
#if 0
  if (Serial.available() > 0)
  {
    in = Serial.readString();
    in.trim();

    in_split = split(in, ' ', qtde);

    if (in == "e")
    {
      Serial.print(irCount);
      Serial.print(" ");
      Serial.println("0");
    }
    else if (in_split[0] == "m")
    {
      rev = in_split[2].toInt();
      setPoint = rev * 8;
      input = irCount;
      motorPID.Compute();

      if (output > 50)
        output = 50;

      if (output < 15)
        output = 0;

      motorSpeed = output;
      dc.moveMotor(motorSpeed);
      
      Serial.println(motorSpeed);
    }
  }
#endif

#if 1
  if (Serial.available() > 0)
  {
    in = Serial.readString();
    in.trim();

    in_split = split(in, ' ', qtde);

    if (in == "e")
    {
      Serial.print(irCount);
      Serial.print(" ");
      Serial.println("0");
    }
    else if (in_split[0] == "m")
    {
      motorSpeed = in_split[2].toInt();

      if (motorSpeed > 0)
        motorSpeed = 35;

      dc.moveMotor(motorSpeed);
      Serial.println(motorSpeed);
    }
  }
#endif
}

void countRev()
{
  // Serial.print("INT!!!\t");
  // Serial.println(irCount);

  ir.currentTime = millis();
  if (ir.currentTime - ir.lastTime >= ir.debounceDelay)
  {
    irCount++;
    ir.lastTime = ir.currentTime;
  }
}

String *split(String &v, char delimiter, int &length)
{
  length = 1;
  bool found = false;

  // Figure out how many items the array should have
  for (int i = 0; i < v.length(); i++)
  {
    if (v[i] == delimiter)
    {
      length++;
      found = true;
    }
  }

  // If the delimiter is found than create the array
  // and split the String
  if (found)
  {
    // Create array
    String *valores = new String[length];

    // Split the string into array
    int i = 0;
    for (int itemIndex = 0; itemIndex < length; itemIndex++)
    {
      for (; i < v.length(); i++)
      {

        if (v[i] == delimiter)
        {
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

void plot(String label, float value, bool isLastGraph = false)
{
  Serial.print(label); // May be empty string

  if (label != "")
    Serial.print(": ");

  Serial.print(value);

  if (isLastGraph == false)
    Serial.print(", ");
  else
    Serial.println();
}
