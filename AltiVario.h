/*
    The AltiVario library is a library that usses BMP280_DEV library to compute a linear function of the altitude changes.

    base library: https://github.com/MartinL1/BMP280_DEV

    used libraries License:

    The MIT License (MIT)
	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:
	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.

*/
#ifndef ALTIVARIO_H
#define ALTIVARIO_H

#include <BMP280_DEV.h>    


class AltiVario : public BMP280_DEV{

public:

    AltiVario();

    float QNH = 1013.25f;    //standard sea level pressure
    float meanValue = 0;    //the altitudee difference between two points, using the intermediar values between them
    uint8_t nrOfValues = 0; //the number of values/measurements for which you will compute the derivative to find the altitude slope
    float lastAltitude = 0; //previous reading of altitude, used to compute the altitude difference
    float altitude;         //the reading of altitude, which is global in order to be used in all the the functions and, save time 
    uint8_t deltaTime = 0;  //the time difference between more than 2 altutude readings
    bool measured = false;  //a flag that is used to prevent more than 1 readings fromm the sensor when we want both the altitude and the altitude difference

    /*
        Kalman filter parameters
        source: https://gist.github.com/Zymotico/836c5d82d5b52a2a3695
    */
    float varVolt = 1.E-04;  
    float varProcess = 4e-5;
    float Pc = 0.0;
    float G = 0.0;
    float P = 1.0;
    float Xp = 0.0;
    float Zp = 0.0;
    float Xe = 0.0;
    

    void begin();   //start the sensor communication protocol

    void setQNH(float qnh);     //set the sea level pressure of the day (@qnh)                              
    float getQNH();     //get the saved sea level pressure
    float computeQNH(float curentAlt);      //compute the sea level pressure knowing the altitude (@curentAlt) at which the measurements are taken
    uint8_t setQNHtoQFE();      //assume the base perssure for measureing the altitude is the curent altitude. This way, the ground will be at 0m altitude

    float getAltimeter(char unit);  //compute the current altitude
    float getVario(long unsigned int lastTime, long unsigned int now, uint8_t frequency);   //compute the altitude difference between the last (@lastTime) reading and now(@now) knowing the reading frequency (@frequency)
    float getVarioKalman(long unsigned int lastTime, long unsigned int now, uint8_t frequency); //apply Kalman filter on the getVario() output

    void done();    //notify the sensor that the curent cycle is over, and the next time the altitude is requested, it needs to recompute it

private:
    float derivatve(float point, long unsigned deltaTime);  //compute the derivative of an interval , in a given period of time
};


#endif