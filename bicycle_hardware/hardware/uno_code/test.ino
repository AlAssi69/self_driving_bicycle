/*
#include "DCMotor.h"
#include "IRSensor.h"

#define LED_PIN1 11
#define LED_PIN2 3

String in = "";
String *in_split;
int intensity1 = 0;
int intensity2 = 0;
int qtde;
int enc = 0;
int pos = 0;

String *split(String &v, char delimiter, int &length);

void setup1()
{
    pinMode(LED_PIN1, OUTPUT);
    pinMode(LED_PIN2, OUTPUT);

    while (!Serial)
        ;

    Serial.begin(57600);
}

void loop1()
{
    if (enc >= 100)
        enc = 0;
    else
        enc = millis() / 500;

    pos = 0;

    if (Serial.available() > 0)
    {
        in = Serial.readString();
        in.trim();

        in_split = split(in, ' ', qtde);

        if (in_split[0] == "e")
        {
            Serial.print(pos);
            Serial.print(" ");
            Serial.println(enc);
        }
        else if (in_split[0] == "m")
        {
            intensity1 = in_split[1].toInt();
            intensity2 = in_split[2].toInt();

            analogWrite(LED_PIN1, intensity1);
            analogWrite(LED_PIN2, intensity2);

            Serial.println("DONE!");
        }
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
*/
