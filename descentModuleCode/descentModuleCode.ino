#include <EYW.h>
  EYW::Altimeter a;
void setup(){
  Serial.begin(9600);
  Serial.println("Initializing.....");
  Serial.print(a.begin());
  a.calibrate(10);
  pinMode(5,OUTPUT);
}
double heights[10];
void loop(){
 Serial.print("Measuring Altitude: ");
  Serial.println((int)a.getHeightAvg(10));
}
