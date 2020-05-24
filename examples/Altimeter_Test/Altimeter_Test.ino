#include <AltiVario.h>

#define FREQUENCY 10

AltiVario altimeter;


unsigned long lastTime;
unsigned long now;



void setup() {
  // put your setup code here, to run once:

  altimeter.begin();
  Serial.begin(115200);
  
  lastTime = millis();
}

void loop() {
  now = millis();

  if(now - lastTime >= 1000 / FREQUENCY)
  {
    //Serial.print(altiVario.getAltimeter('m'));
    Serial.print("  ");
    Serial.print(altimeter.getAltimeter());
    Serial.println("m ");

    lastTime = now;
    altiVario.done();
  }
  
}
