#include <AltiVario.h>

#define FREQUENCY 10

AltiVario variometer;


unsigned long lastTime;
unsigned long now;



void setup() {
  // put your setup code here, to run once:

  variometer.begin();
  Serial.begin(115200);

  variometer.setQNH(1022);  // set the QNH to your local QNH of the day (sea level pressure), just search online QNH + your location
  
  lastTime = millis();
}

void loop() {
  now = millis();

  if(now - lastTime >= 1000 / FREQUENCY)
  {
    Serial.print(variometer.getVario(lastTime, now, FREQUENCY));
    Serial.print(" ");
    Serial.print(variometer.getVarioKalman(lastTime, now, FREQUENCY));
    Serial.println(" ");


    lastTime = now;
    variometer.done();
  }
  
}
