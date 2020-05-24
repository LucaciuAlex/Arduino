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

#include "AltiVario.h"

AltiVario::AltiVario() : BMP280_DEV() {}

/*
    Initialise the sensor communication method to I2C
    The default sensor address is 0x77, but in order to read the altitude, the sensor needs to be switched to the 0x76 BMP280_I2C_ALT_ADDR
    Oversampleing is done internaly on the sensor chip, and it is done by adding X signal waves together and sendingthe resulting wave to the microcontroller.
    The digitaleffect of Oversampling is simmilar to computing the mean of multiple readings 
    Oversampling possible values: OVERSAMPLING_OFF,OVERSAMPLING_X1, OVERSAMPLING_X2, OVERSAMPLING_X4, OVERSAMPLING_X8, OVERSAMPLING_X16
    IIR filter is a noise clearing, alsodirected implemented in the sensor chip. 
    Standby time is the time the sensor will swithc to SLEEP_MODE before being ready to perform the next reading
*/
void AltiVario::begin()
{
    BMP280_DEV::begin(BMP280_I2C_ALT_ADDR);              // Default initialisation with alternative I2C address (0x76), place the BMP280 into SLEEP_MODE 
    BMP280_DEV::setPresOversampling(OVERSAMPLING_X16);    // Set the pressure oversampling to X6
    BMP280_DEV::setTempOversampling(OVERSAMPLING_X2);    // Set the pressure oversampling to X2
    BMP280_DEV::setIIRFilter(IIR_FILTER_16);              // Set the IIR filter to setting 16
    BMP280_DEV::setTimeStandby(TIME_STANDBY_62MS);       //Set the sensor standby time to 62 mili seconds   
    BMP280_DEV::startNormalConversion();                 //Set the senor to NORMA_MODE  
}


/*
    set the sea level pressure to @QNH
    If not set, the default value is 1013.23f

    "QNH"  is the aeronautical term for the sea level pressure. 
    In order to compute the altitude, based on pressure, the sensor has to knnow the sea leel pressure.
    The class AltiVario stores the value for QNH, because the BMP280_DEV librarydoes not have a function that returnes it.
*/
void AltiVario::setQNH(float qnh)
{
    QNH = qnh;
    BMP280_DEV::setSeaLevelPressure(qnh);
}


/*
    return the value of sea level pressure that is being used to compute the altitude.
*/
float AltiVario::getQNH()
{
    return QNH;
}

/*
    Assume the sensor is stationary and compute the sea level pressure based on the curent altitude
    try getting the measurements with BMP280_DEV::getMeasurements() 
    if the sensor is not ready, the function BMP280_DEV::getMeasurements() willreturn 0, so several calls muight be necessary to get the values
    compute the QNH based on the temperature and pressure at the current location of the sensor
    return the computed @QNH
*/
float AltiVario::computeQNH(float curentAlt)
{
    float temp, press;
    while(!BMP280_DEV::getMeasurements(temp, press, altitude));
    QNH = powf((((curentAlt * 0.0065) / (temp + 273.15)) + 1),5.26) * press;
    return QNH;
}

/*
    QNH is the pressure at the sea level
    QFE is the pressure at the ground level
    THe altimeter will compute the altitude relative to the pressure provided.
    If it has the pressure set to the sea level presure, it will compute the altitude
    If it has the perssure set to the ground level pressure, it will compute vertical distance from the altimeter to the ground (altitude)
    Setting QNH to QFE basicaly takes the ground as the refference point for computing the altitude
    In this case, the altimeter will compute the "height", not the "altitude"
    BMP280_DEV::getPressure(&QNH) will return the value as a refference, through QNH pointer
    Also, set the internal seaLevelPressure of the BMP280_DEV library to the current pressure
*/
uint8_t AltiVario::setQNHtoQFE()
{
    BMP280_DEV::getPressure(QNH);
    BMP280_DEV::setSeaLevelPressure(QNH);
}


/*
    return the current altitude in meters 'm' or feets 'f'
    if the sensor is not ready, the function BMP280_DEV::getMeasurements() willreturn 0, so several calls muight be necessary to get the values
    if the function computes the altitude, will set the @measured flagg to true
    the @measured flagg is used to save time, and not rerequest the sensor to compute it again in the same cycle
    A "cycle" is considered one full run of the main loopfunction (while()/run()/loop()/...)
    If the mearuser flagg is true, use the allready available @altitude value 
    return the altitude
*/
float AltiVario::getAltimeter(char unit)
{
    if(!measured)
    {
        float temp, press;
        while(!BMP280_DEV::getMeasurements(temp, press, altitude))
        {
            measured = true;
            lastAltitude = altitude;
        }
    }

    if(unit == 'm')
        return altitude;

    if(unit == 'f')
        return altitude * 3.28;
}

/*
    compute the mean altitude change over values in a time interval
    the time interval depends on the frequencyof the readings, but will never excede 0.2s 
    in order to save space, the function will not store all the values, and will not have to rotate the, as they are read and discarded
    The mean value will contain @nrOfValues values in it, and, each time a new reading is done, in order to have @nrOfValues - 1, an average value will be subtracted
    the newly read value will be added to the @meanValue, such that it will contain again @nrOfValues values.
    we need the speed whith which the altitude is changing, so we ill compute the derivative ovet the interval of values
*/
float AltiVario::getVario(long unsigned int lastTime, long unsigned int now, uint8_t frequency)
{
    if(!measured)
    {
        float temp, press;
        while(!BMP280_DEV::getMeasurements(temp, press, altitude))
        {
            measured = true;
        }
    }

    if(nrOfValues < frequency / 5)
    {
        meanValue += (altitude - lastAltitude); 
        deltaTime += (now - lastTime);
        nrOfValues++;
    }
    else
    {
        meanValue -= (meanValue / nrOfValues);
        meanValue += (altitude - lastAltitude);

        deltaTime -= (deltaTime / nrOfValues);
        deltaTime += (now - lastTime);
    }

    lastAltitude = altitude;

    return derivatve(meanValue, deltaTime) ;
    
}

/*
    apply a kalman filter on the variometer values
    source: https://gist.github.com/Zymotico/836c5d82d5b52a2a3695
*/
float AltiVario::getVarioKalman(long unsigned int lastTime, long unsigned int now, uint8_t frequency)
{
    Pc = P + varProcess;
    G = Pc/(Pc + varVolt);  
    P = (1-G)*Pc;
    Xp = Xe;
    Zp = Xp;
    Xe = G*( getVario(lastTime, now, frequency) -Zp) + Xp;   // the kalman estimate of the sensor voltage

    return Xe ;
}

/*
    set the @measured flag back to false, at the end of a cycle.
    the @measured flagg is used to save time, and not rerequest the sensor to compute it again in the same cycle 
    A "cycle" is considered one full run of the main loopfunction (while()/run()/loop()/...)
*/
void AltiVario::done()
{
    measured = false;
}

/*
    compute the partial derivative over an interval of size @deltaDist, in a period of time @deltaTIme
    the interval is measured in meters and the time intervalis measured in mili seconds
    in order to compute a derivative over m/s we multiplymeters by 1000
*/
float AltiVario::derivatve(float deltaDist, long unsigned deltaTime)
{
    return ( deltaDist * 1000  ) / deltaTime ;
}
