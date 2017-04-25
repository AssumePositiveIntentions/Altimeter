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
 /**for(int i=0;i<10;i++){
  heights[i]=a.getHeight();
  delay(100);
 }
 int sum=0;
 for(int c=0;c<10;c++){
  sum=sum+heights[c];
 }
 
 Serial.println(sum/10);

 //Serial.println(a.getHeight());
 **/
  Serial.println((int)a.getHeightAvg(10));
}
