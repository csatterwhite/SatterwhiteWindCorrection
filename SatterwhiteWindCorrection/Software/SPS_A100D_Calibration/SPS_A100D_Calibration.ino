


//-----------------------------INCLUDES--------------------------------------
#include <Arduino.h>

long timer=0;
int iter=0;

// Define resistor values used in voltage divider circuit Vread = Vsense*R2/(R1+R2)
int R1 = 100000;
int R2 = 100000;
int R1inv = 100000;
int R2inv = 100000;

double mastRot;
double rot1;
double rot2;
int pin1 = A0;
int pin2 = A1;

double voltScale;
double voltScaleInv;
int voltRead;
int voltReadInv;
double voltReadSum = 0;
double voltReadInvSum = 0;
double voltSense;
double voltSenseInv;

void setup()
{
  Serial.begin(9600);
  
// Calculate voltage divider scaling
  voltScale = R2/(R1+R2);
  voltScaleInv = R2inv/(R1inv+R2inv);

}

void loop() //Main Loop
{
    iter = iter+1;
    delay(10);
    voltRead = analogRead(pin1);
    delay(10);
    voltReadInv = analogRead(pin2);
    voltReadSum = voltReadSum+voltRead;
    voltReadInvSum = voltReadInvSum+voltReadInv;
    if (iter == 10)
    {
      voltSense = (voltReadSum/100)/voltScale;
      voltSenseInv = (voltReadInvSum/100)/voltScaleInv;
      iter = 0;
      voltReadSum = 0;
      voltReadInvSum = 0;

      Serial.print("  voltRead = ");
      Serial.print(voltRead);
      Serial.print(", Inverted voltRead = ");
      Serial.println(voltReadInv);
    }
    delay(10);

}
