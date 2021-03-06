#include <EYW.h>
#include <Wire.h>
#include "Servo.h"

 
//****************************************************************************************************
//---------------------------------------Altimeter Functions------------------------------------------
//****************************************************************************************************
 
char EYW::Altimeter::begin()
// Initialize library for subsequent pressure measurements
{
	float c3,c4,b1;

	
	//set the default pin #s
	buttonPin = 2;
	ledPin = 4;
	speakerPin = 5;

	//set the pinModes
	pinMode(buttonPin, INPUT);
	pinMode(ledPin, OUTPUT);
	pinMode(speakerPin, OUTPUT);
	
	//sets the AREF pin to external. Note that if nothing is connected to this pin
	//that the returned ADC values will not be accurate
	analogReference(EXTERNAL);
	 
	//The serial monitor is NOT hard coded to begin
	//Users should explicitly call this in their sketch
	//to read the calibration data
	//Serial.begin(9600);
	
	Serial.println("Altimeter Started!");
	
	
	// Start up the Arduino's "wire" (I2C) library:
	 
	Wire.begin();
	 
	// The BMP180 includes factory calibration data stored on the device.
	// Each device has different numbers, these must be retrieved and
	// used in the calculations when taking pressure measurements.
	 
	// Retrieve calibration data from device:
	 
	if (readInt(0xAA,AC1) &&
	readInt(0xAC,AC2) &&
	readInt(0xAE,AC3) &&
	readUInt(0xB0,AC4) &&
	readUInt(0xB2,AC5) &&
	readUInt(0xB4,AC6) &&
	readInt(0xB6,VB1) &&
	readInt(0xB8,VB2) &&
	readInt(0xBA,MB) &&
	readInt(0xBC,MC) &&
	readInt(0xBE,MD))
	{
		 
		c3 = 160.0 * pow(2,-15) * AC3;
		c4 = pow(10,-3) * pow(2,-15) * AC4;
		b1 = pow(160,2) * pow(2,-30) * VB1;
		c5 = (pow(2,-15) / 160) * AC5;
		c6 = AC6;
		mc = (pow(2,11) / pow(160,2)) * MC;
		md = MD / 160.0;
		x0 = AC1;
		x1 = 160.0 * pow(2,-13) * AC2;
		x2 = pow(160,2) * pow(2,-25) * VB2;
		y0 = c4 * pow(2,15);
		y1 = c4 * c3;
		y2 = c4 * b1;
		p0 = (3791.0 - 8.0) / 1600.0;
		p1 = 1.0 - 7357.0 * pow(2,-20);
		p2 = 3038.0 * 100.0 * pow(2,-36);
		 
		 
		return(1);
	}
	else
	{
		// Error reading calibration data; bad component or connection?
		return(0);
	}
}


//Overloaded function to accept pin #s for the button, led, and speaker
char EYW::Altimeter::begin(int bP, int lP, int sP)
// Initialize library for subsequent pressure measurements
{
	float c3,c4,b1;
	
	//set the pin #s to the user provided values
	buttonPin = bP;
	ledPin = lP;
	speakerPin = sP;
	
	//set the pinModes
	pinMode(buttonPin, INPUT);
	pinMode(ledPin, OUTPUT);
	pinMode(speakerPin, OUTPUT);

	//sets the AREF pin to external. Note that if nothing is connected to this pin
	//that the returned ADC values will not be accurate
	analogReference(EXTERNAL);

	//The serial monitor is NOT hard coded to begin
	//Users should explicitly call this in their sketch
	//to read the calibration data
	//Serial.begin(9600);
	
	Serial.println("Altimeter Started!");
	Serial.print("Button Attached To Pin # ");
	Serial.println(buttonPin);				
	Serial.print("LED Attached To Pin # ");
	Serial.println(ledPin);
	Serial.print("Speaker Attached To Pin # ");
	Serial.println(speakerPin);
	 
	// Start up the Arduino's "wire" (I2C) library:
	 
	Wire.begin();
	 
	// The BMP180 includes factory calibration data stored on the device.
	// Each device has different numbers, these must be retrieved and
	// used in the calculations when taking pressure measurements.
	 
	// Retrieve calibration data from device:
	 
	if (readInt(0xAA,AC1) &&
	readInt(0xAC,AC2) &&
	readInt(0xAE,AC3) &&
	readUInt(0xB0,AC4) &&
	readUInt(0xB2,AC5) &&
	readUInt(0xB4,AC6) &&
	readInt(0xB6,VB1) &&
	readInt(0xB8,VB2) &&
	readInt(0xBA,MB) &&
	readInt(0xBC,MC) &&
	readInt(0xBE,MD))
	{
		 
		c3 = 160.0 * pow(2,-15) * AC3;
		c4 = pow(10,-3) * pow(2,-15) * AC4;
		b1 = pow(160,2) * pow(2,-30) * VB1;
		c5 = (pow(2,-15) / 160) * AC5;
		c6 = AC6;
		mc = (pow(2,11) / pow(160,2)) * MC;
		md = MD / 160.0;
		x0 = AC1;
		x1 = 160.0 * pow(2,-13) * AC2;
		x2 = pow(160,2) * pow(2,-25) * VB2;
		y0 = c4 * pow(2,15);
		y1 = c4 * c3;
		y2 = c4 * b1;
		p0 = (3791.0 - 8.0) / 1600.0;
		p1 = 1.0 - 7357.0 * pow(2,-20);
		p2 = 3038.0 * 100.0 * pow(2,-36);
		 
		 
		return(1);
	}
	else
	{
		// Error reading calibration data; bad component or connection?
		return(0);
	}
}

char EYW::Altimeter::readInt(char address, int &value)
// Read a signed integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
	unsigned char data[2];
	 
	data[0] = address;
	if (readBytes(data,2))
	{
		value = (((int)data[0]<<8)|(int)data[1]);
		//if (*value & 0x8000) *value |= 0xFFFF0000; // sign extend if negative
		return(1);
	}
	value = 0;
	return(0);
}
 

//The following four functions all pertain to reading data from an i2c device
//*****************************************************************************

char EYW::Altimeter::readUInt(char address, unsigned int &value)
// Read an unsigned integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
	unsigned char data[2];
	 
	data[0] = address;
	if (readBytes(data,2))
	{
		value = (((unsigned int)data[0]<<8)|(unsigned int)data[1]);
		return(1);
	}
	value = 0;
	return(0);
}
 
 
char EYW::Altimeter::readBytes(unsigned char *values, char length)
// Read an array of bytes from device
// values: external array to hold data. Put starting register in values[0].
// length: number of bytes to read
{
	char x;
	 
	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(values[0]);
	_error = Wire.endTransmission();
	if (_error == 0)
	{
		Wire.requestFrom(BMP180_ADDR,length);
		while(Wire.available() != length) ; // wait until bytes are ready
		for(x=0;x<length;x++)
		{
			values[x] = Wire.read();
		}
		return(1);
	}
	return(0);
}
 
 
char EYW::Altimeter::writeBytes(unsigned char *values, char length)
// Write an array of bytes to device
// values: external array of data to write. Put starting register in values[0].
// length: number of bytes to write
{
	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(values,length);
	_error = Wire.endTransmission();
	if (_error == 0)
	return(1);
	else
	return(0);
}
 
//*******************************************************************************
//
//The Following six functions are from Sparkfun's BMP180 library and rely on the Bosch data sheet and coeffecients 
//
//*******************************************************************************

char EYW::Altimeter::startTemperature(void)
// Begin a temperature reading.
// Will return delay in ms to wait, or 0 if I2C error
{
	unsigned char data[2], result;
	 
	data[0] = BMP180_REG_CONTROL;
	data[1] = BMP180_COMMAND_TEMPERATURE;
	result = writeBytes(data, 2);
	if (result) // good write?
	return(5); // return the delay in ms (rounded up) to wait before retrieving data
	else
	return(0); // or return 0 if there was a problem communicating with the BMP
}
 
 
char EYW::Altimeter::getTemperature(float &T)
// Retrieve a previously-started temperature reading.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startTemperature() to have been called prior and sufficient time elapsed.
// T: external variable to hold result.
// Returns 1 if successful, 0 if I2C error.
{
	unsigned char data[2];
	char result;
	float tu, a;
	 
	data[0] = BMP180_REG_RESULT;
	 
	result = readBytes(data, 2);
	if (result) // good read, calculate temperature
	{
		tu = (data[0] * 256.0) + data[1];
		a = c5 * (tu - c6);
		T = a + (mc / (a + md));
		 
	}
	return(result);
}
 
 
char EYW::Altimeter::startPressure(char oversampling)
// Begin a pressure reading.
// Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
// Will return delay in ms to wait, or 0 if I2C error.
{
	unsigned char data[2], result, delay;
	 
	data[0] = BMP180_REG_CONTROL;
	 
	switch (oversampling)
	{
		case 0:
		data[1] = BMP180_COMMAND_PRESSURE0;
		delay = 5;
		break;
		case 1:
		data[1] = BMP180_COMMAND_PRESSURE1;
		delay = 8;
		break;
		case 2:
		data[1] = BMP180_COMMAND_PRESSURE2;
		delay = 14;
		break;
		case 3:
		data[1] = BMP180_COMMAND_PRESSURE3;
		delay = 26;
		break;
		default:
		data[1] = BMP180_COMMAND_PRESSURE0;
		delay = 5;
		break;
	}
	result = writeBytes(data, 2);
	if (result) // good write?
	return(delay); // return the delay in ms (rounded up) to wait before retrieving data
	else
	return(0); // or return 0 if there was a problem communicating with the BMP
}
 
 
char EYW::Altimeter::getPressure(float &P, float &T)
// Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startPressure() to have been called prior and sufficient time elapsed.
// Requires recent temperature reading to accurately calculate pressure.
 
// P: external variable to hold pressure.
// T: previously-calculated temperature.
// Returns 1 for success, 0 for I2C error.
 
// Note that calculated pressure value is absolute mbars, to compensate for altitude call sealevel().
{
	unsigned char data[3];
	char result;
	float pu,s,x,y,z;
	 
	data[0] = BMP180_REG_RESULT;
	 
	result = readBytes(data, 3);
	if (result) // good read, calculate pressure
	{
		pu = (data[0] * 256.0) + data[1] + (data[2]/256.0);
		 
		s = T - 25.0;
		x = (x2 * pow(s,2)) + (x1 * s) + x0;
		y = (y2 * pow(s,2)) + (y1 * s) + y0;
		z = (pu - x) / y;
		P = (p2 * pow(z,2)) + (p1 * z) + p0;
	}
	return(result);
}

float EYW::Altimeter::sealevel(float P, float A)
// Given a pressure P (mb) taken at a specific altitude (meters),
// return the equivalent pressure (mb) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
	return(P/pow(1-(A/44330.0),5.255));
}
 
 
float EYW::Altimeter::altitude(float P, float P0)
// Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
// return altitude (meters) above baseline.
{
	return(44330.0*(1-pow(P/P0,1/5.255)));
}
 
 
char EYW::Altimeter::getError(void)
// If any library command fails, you can retrieve an extended
// error code using this command. Errors are from the wire library:
// 0 = Success
// 1 = Data too long to fit in transmit buffer
// 2 = Received NACK on transmit of address
// 3 = Received NACK on transmit of data
// 4 = Other error
{
	return(_error);
}

//***************************************************************************************************

char EYW::Altimeter::calibrate(int num)
//Given a number 'num' the calibration scheme will take average measurements of
//a baseline zero alititude.
{
	float temperature;
	float pressure;
	float total = 0;
	for (int i=0; i<num; i++)
	{
		startTemperature();
		delay(5);//factory delay recommendation
		getTemperature(temperature);
		startPressure(3);
		delay(26);//factory delay recommendation
		getPressure(pressure,temperature);
		total += sealevel(pressure,0.0);
		 
	}
	baselineSLP = total/(float)num;
	if (baselineSLP != -999.99) return 1; //check if the calibration scheme has been correctly run
	else return 0;
}
char EYW::Altimeter::calibrate(int num, float filterVal)
//Given a number num the calibration scheme will take num measurements of
//a baseline zero alititude. Optionally, set the low pass filter with filterVal
//filterVal is 0.0 - 1.0 and defaults to 1.0 (filter is off)
//This should only be used for advanced debugging
{
	float temperature;
	float pressure;
	float avArray[num];
	float heightLPF;
	float total;
	for (int i=0; i<num; i++)
	{
		startTemperature();
		delay(5);//factory delay recommendation
		getTemperature(temperature);
		startPressure(3);
		delay(26);//factory delay recommendation
		getPressure(pressure,temperature);
		avArray[i] = sealevel(pressure,0.0);
		//initializes the first value of the LPF to the first reading
		if (i==0) heightLPF = avArray[0];
		//Low Pass Filter algorithm
		if (filterVal < 1)
		{
			heightLPF = (avArray[i] * filterVal) + (heightLPF * (1 - filterVal));
			avArray[i] = heightLPF;
		}
	}

	//take the average of the low pass filtered values
	for (int i = 0; i < num; i++)
	{
		total += avArray[i];
	}
	baselineSLP = total / float(num);
	if (baselineSLP != -999.99) return 1; //check if the calibration scheme has been correctly run
	else return 0;
}
 
void EYW::Altimeter::alarm(void)
//Sounds the alititude alarm on the specified pin number
//Defaults to 2 tones, at 880 Hz, for 1/4 second each
{
	int num = 2;
	int freq = 880;
        int duration = 250;
	//play the tone num times at freq frequency for dur milliseconds
	for (int i = 0; i < num; i++)
	{
		digitalWrite(ledPin,HIGH);
		tone(speakerPin, freq, duration); delay(duration);
		digitalWrite(ledPin,LOW);
		delay(duration);
	}
}

void EYW::Altimeter::alarm(int num, int freq, int dur)
//Sounds the altitude alarm on pinNum for 'num' times at 'freq' frequency for dur duration
{
	int duration = dur;
	//play the tone num times at freq frequency for dur milliseconds
	for (int i = 0; i < num; i++)
	{
		digitalWrite(ledPin,HIGH);
		tone(speakerPin, freq, duration); delay(duration);
		digitalWrite(ledPin,LOW);
		delay(duration);
	}
}

float EYW::Altimeter::getHeight(void)
//retrieves the current altitude reading and returns the value in feet
//uses the calibration scheme to reference zero height
{
	float temperature, pressure, height;
	 
	startTemperature();
	delay(5);
	getTemperature(temperature);
	startPressure(3);
	delay(26);
	getPressure(pressure,temperature);
	height = altitude(pressure, baselineSLP);
	return (height);
	 
}
 
float EYW::Altimeter::getHeightAvg(int num)
//runs getHeight 'num' times and averages those readings
//returns the average current altitude in feet
{
	float temperature, pressure, height, total = 0;
	 
	for (int i = 0; i < num; i++)
	{
		startTemperature();
		delay(5);
		getTemperature(temperature);
		startPressure(3);
		delay(26);
		getPressure(pressure,temperature);
		total += altitude(pressure, baselineSLP); //add all the measurements together
	}
	 
	height = total / num; //divide the total by the number to get the mean
	return (height);
}
 

void EYW::Altimeter::led(int status)
//controls the LED on the defined led pin #
//takes an argument of true/HIGH/1 and false/LOW/0
{
	if (status == 1) digitalWrite(ledPin, HIGH);
	else if (status == 0) digitalWrite(ledPin, LOW);
	else return;
}
 
//****************************************************************************************************
//-----------------------------------------Camera Functions-------------------------------------------
//****************************************************************************************************


char EYW::Camera::begin(void)
//the begin function initializes the servo Pin #, the push button pin # and attaches the servo object
//Defaults to the Button on pin 2, the servo on pin 3
{
	//set the default pin #s
	buttonPin = 2;
	ledPin = 4;
	speakerPin = 5;
	
	servoPin = 3;

	//set the pin modes
	pinMode(buttonPin, INPUT);
	pinMode(ledPin, OUTPUT);
	pinMode(speakerPin, OUTPUT);

	pinMode(servoPin, OUTPUT);

	//set all check flags to false
	setFlags();

	//The serial monitor is NOT hard coded to begin
	//Users should explicitly call this in their sketch
	//to read the calibration data
	//Serial.begin(9600);
	
	Serial.println("Camera Started!");

	return(1);
}

char EYW::Camera::begin(int bP, int lP, int sP)
//begin initializes the servo on pin sP and the pushbutton on pin bP
{
	
	//set the pin #'s to the user defined values
	buttonPin = bP;
	ledPin = lP;
	speakerPin = sP;
	
	servoPin = 3;

	//set the pin modes
	pinMode(buttonPin, INPUT);
	pinMode(ledPin, OUTPUT);
	pinMode(speakerPin, OUTPUT);

	pinMode(servoPin, OUTPUT);

	//set the check flags to false
	setFlags();

	//The serial monitor is NOT hard coded to begin
	//Users should explicitly call this in their sketch
	
	//Serial.begin(9600);
	
	Serial.println("Camera Started!");
	Serial.print("Button Attached To Pin # ");
	Serial.println(buttonPin);				
	Serial.print("LED Attached To Pin # ");
	Serial.println(ledPin);
	Serial.print("Speaker Attached To Pin # ");
	Serial.println(speakerPin);

	return(1);
}

char EYW::Camera::begin(int bP, int lP, int sP, int smP)
//begin initializes the servo on pin sP and the pushbutton on pin bP
{
	
	//set the pin #'s to the user defined values
	buttonPin = bP;
	ledPin = lP;
	speakerPin = sP;
	
	servoPin = smP;

	//set the pin modes
	pinMode(buttonPin, INPUT);
	pinMode(ledPin, OUTPUT);
	pinMode(speakerPin, OUTPUT);

	pinMode(servoPin, OUTPUT);

	//set the check flags to false
	setFlags();

	//The serial monitor is NOT hard coded to begin
	//Users should explicitly call this in their sketch
	
	//Serial.begin(9600);
	
	Serial.println("Camera Started!");
	Serial.print("Button Attached To Pin # ");
	Serial.println(buttonPin);				
	Serial.print("LED Attached To Pin # ");
	Serial.println(ledPin);
	Serial.print("Speaker Attached To Pin # ");
	Serial.println(speakerPin);

	return(1);
}


char EYW::Camera::calibrate()
//the calibrate function is used to define the left and right limits of the servo motor and it defines the length
//of time for each movement.
{
	
	//set the default pin for the servo and the hard limits of 20 and 90
	servoLeftStop = 20;
	servoRightStop = 90;

	//set the default time to move the servo from 90 -> 20 
	//This is calculated with a measured time of 3ms per 10 degrees
	moveDelay = 210;

	motor.attach(servoPin);

	//"calibrates" the motor by moving left to right
	setMotor(moveDelay,moveDelay);
	return(1);
}

char EYW::Camera::calibrate(int lS, int rS)
//the calibrate function  defines the servo pin # smP
//the left limit lS, and the right limit rS of the servo motor 
{
	
	float limitDiff;
	
	//set the servo pin #
		
	//hard coded stops for the motor
	//measured values for the EYW servo indicate that anything below 20
	//or above 180 doesn't work
	if (lS>180) lS = 180;
	else if (lS < 20) lS = 20;
	if (rS>180) rS = 180;
	else if (rS<20) rS = 20;
	
	servoLeftStop = lS;
	servoRightStop = rS;

	//set the movement delay for servo action
	//based on a measured 3ms for 10 degrees of motion
	limitDiff = (float)abs(servoRightStop - servoLeftStop)/10.0;
	moveDelay = limitDiff * 30;
	
	motor.attach(servoPin);
	setMotor(moveDelay,moveDelay);
	return(1);
}

char EYW::Camera::beginTimer(unsigned long tl)
//start the timer using the number of seconds tl
{
	timerLength = tl;
	timeElapsed = millis();
	timerFlag = true;
	return(1);
}

char EYW::Camera::endTimer(void)
{
	timerFlag = false;
	return (1);
}

char EYW::Camera::getPicture()
//take a picture using the defined left and right limits from calibrate. 
//defaults to a depress length and refresh length 
{
	//after the servo moves for moveDelay (set in calibration)
	//the servo holds for 500 ms
	//and then waits 1 second before it can trigger again
	pressLength = 500 + moveDelay;
	picDelay = 1000 + moveDelay;

	//take a picture
	setMotor(pressLength,picDelay);

	//set all flags to false after a picture has been taken
	setFlags();
	return(1);
}

char EYW::Camera::getPicture(unsigned long pL, unsigned long pD)
//take a picture using the defined left and right limits from calibrate. 
//press length and refresh length are user provided
{
	//pL and pD define the length of time to depress the camera button
	//and the length of time to wait before another picture can be taken
	
	pressLength = pL + moveDelay;
	picDelay = pD + moveDelay;
	setMotor(pressLength,picDelay);
	//set all flags to false after a picture has been taken
	setFlags();
	return(1);
}

boolean EYW::Camera::buttonPressed(void)
//checks if the push button has been pressed
{
	//this debounce is added to prevent unintentional button depressess
	//required for less than ideal mechanical switches
	if ((millis() - elapsed) > debounce)
	{
		buttonFlag = digitalRead(buttonPin);
		elapsed = millis();
	        if (buttonFlag == HIGH) return true;
	}
	return false;


}

boolean EYW::Camera::timerExpired(void)
//check if the timer has expired. startTimer must be called prior to checking if it has expired.
{
	//checks that the timer has indeed been started
	if (timerFlag)
	{
		if ((millis() - timeElapsed) >= timerLength)
		{
			return true;	
		}
	}
	else
	{
		return false;
	}
}

void EYW::Camera::alarm(void)
//default alarm on the speaker pin and LED pin
// sounds 1 time at 523 hertz for 1/4 of a second
{
	int num = 1;
	int freq = 523;
        int duration = 250;
	//play the tone num times at freq frequency for dur milliseconds
	for (int i = 0; i < num; i++)
	{
		digitalWrite(ledPin,HIGH);
		tone(speakerPin, freq, duration); delay(duration);
		digitalWrite(ledPin,LOW);
		if (i>0) delay(duration);
	}
}

void EYW::Camera::alarm(int num, int freq, int duration)
//Sounds the alarm on the speaker pin and the LED pin
//for 'num' times at 'freq' frequency for time 'duration'
{
	//play the tone num times at freq frequency for dur milliseconds
	for (int i = 0; i < num; i++)
	{
		digitalWrite(ledPin,HIGH);
		tone(speakerPin, freq, duration); delay(duration);
		digitalWrite(ledPin,LOW);
		if (i>0) delay(duration);
	}
}

void EYW::Camera::led(int status)
//controls the LED on the defined led pin #
//takes an argument of true/HIGH/1 and false/LOW/0
{
	if (status == 1) digitalWrite(ledPin, HIGH);
	else if (status == 0) digitalWrite(ledPin, LOW);
	else return;
}

void EYW::Camera::setFlags(void)
//set all of the boolean flags to false
{
	buttonFlag = false;
	timerFlag = false;

}

//command the stepper motor to the limits define in calibrate
void EYW::Camera::setMotor(unsigned long d1, unsigned long d2)
{
	motor.write(servoLeftStop);
	delay(d1);
	motor.write(servoRightStop);
	delay(d2);
}
//****************************************************************************************************
//-------------------------------------Accelerometer Functions-----------------------------------------
//****************************************************************************************************

void EYW::Accelerometer::begin()
//Initialize the accelerometer to default button, led, and speaker pin #s
//sets the x, y, and z axes to analog pin #s
//provides default calibration numbers (that are likely not accurate)
{
	//default pin #s
	buttonPin = 2;
	ledPin = 4;
	speakerPin = 5;
	
	// x-Axis = A0, y-Axis = A1, z-Axis = A2
	xPin = 14;
	yPin = 15;
	zPin = 16;
	
	//These are empirical calibration numbers that will provide DECENT readings
	//but not recommended to rely on these
	calArray[0] = 511.0;
	calArray[1] = 19.0;
	calArray[2] = 511.0;
	calArray[3] = 19.0;
	calArray[4] = 511.0;
	calArray[5] = 19.0;

	//notes that calibration has not been run
	calFlag = false;
	
	//sets the AREF pin to external. Note that if nothing is connected to this pin
	//that the returned ADC values will not be accurate
	analogReference(EXTERNAL);
	
	//set the appropriate pin modes
	pinMode(buttonPin, INPUT);
	pinMode(ledPin, OUTPUT);
	pinMode(speakerPin, OUTPUT);
	pinMode(xPin, INPUT);
	pinMode(yPin, INPUT);
	pinMode(zPin, INPUT);

	//The serial monitor is NOT hard coded to begin
	//Users should explicitly call this in their sketch
	//to read the calibration data

	//Serial.begin(9600);
	
	Serial.println("Accelerometer Started!");
}

void EYW::Accelerometer::begin(int bP, int lP, int sP)
//Initialize the accelerometer to with user supplied pins for the button, LED and speaker
//sets the x, y, and z axes to analog pin #s
//provides default calibration numbers (that are likely not accurate)
{
	//set pin #s
	buttonPin = bP;
	ledPin = lP;
	speakerPin = sP;

	// x-Axis = A0, y-Axis = A1, z-Axis = A2
	xPin = 14;
	yPin = 15;
	zPin = 16;

	//These are empirical calibration numbers that will provide DECENT readings
	//but not recommended to rely on these
	calArray[0] = 511.0;
	calArray[1] = 19.0;
	calArray[2] = 511.0;
	calArray[3] = 19.0;
	calArray[4] = 511.0;
	calArray[5] = 19.0;
	calFlag = false;

	//sets the AREF pin to external. Note that if nothing is connected to this pin
	//that the returned ADC values will not be accurate
	analogReference(EXTERNAL);

	//sets the appropriate pinModes
	pinMode(buttonPin, INPUT);
	pinMode(ledPin, OUTPUT);
	pinMode(speakerPin, OUTPUT);
	pinMode(xPin, INPUT);
	pinMode(yPin, INPUT);
	pinMode(zPin, INPUT);

	//The serial monitor is NOT hard coded to begin
	//Users should explicitly call this in their sketch
	//to read the calibration data

	//Serial.begin(9600);
	Serial.println("Accelerometer Started!");
	Serial.print("Button Attached To Pin # ");
	Serial.println(buttonPin);				
	Serial.print("LED Attached To Pin # ");
	Serial.println(ledPin);
	Serial.print("Speaker Attached To Pin # ");
	Serial.println(speakerPin);
}

void EYW::Accelerometer::calibrate(void)
//Calibration routine takes 10 samples from 6 distinct orientations
//with reference to gravity. + and - readings should read +1G and -1G
//The median between two readings provides the zero-G reading
//and the difference between the median and the high or low reading
//provide the mV/G value
{
	//hard coded 10 samples to average per reading
	int samples = 10;
	int xMin, xMax, yMin, yMax, zMin, zMax;

	//each of the six orientations will wait for a button press
	//before the readings begin
	
	Serial.println("\nPress Button to Start +X Calibration");
	while (buttonPressed() == false);
	xMax = readAccels(xPin, samples);
	Serial.println(xMax);
	Serial.println("Done");

	Serial.println("Press Button to Start -X Calibration");
	while (buttonPressed() == false);
	xMin = readAccels(xPin, samples);
	Serial.println(xMin);
	Serial.println("Done");

	Serial.println("Press Button to Start +Y Calibration");
	while (buttonPressed() == false);
	yMax = readAccels(yPin, samples);
	Serial.println(yMax);
	Serial.println("Done");

	Serial.println("Press Button to Start -Y Calibration");
	while (buttonPressed() == false);
	yMin = readAccels(yPin, samples);
	Serial.println(yMin);
	Serial.println("Done");

	Serial.println("Press Button to Start +Z Calibration");
	while (buttonPressed() == false);
	zMax = readAccels(zPin, samples);
	Serial.println("Done");

	Serial.println("Press Button to Start -Z Calibration");
	while (buttonPressed() == false);
	zMin = readAccels(zPin, samples);
	Serial.println("Done");
	
	//indices 0,2,4 (x,y,z) are the zero-G readings
	//indices 1,3,5 (x,y,z) are the mV/G values
	calArray[0] = ((float)(xMax + xMin) / 2.0);
  	calArray[1] = ((float)(xMax - xMin) / 2.0);
  	calArray[2] = ((float)(yMax + yMin) / 2.0);
  	calArray[3] = ((float)(yMax - yMin) / 2.0);
  	calArray[4] = ((float)(zMax + zMin) / 2.0);
 	calArray[5] = ((float)(zMax - zMin) / 2.0);
	
	Serial.println("Calibration Complete!\n");
	Serial.print("X-axis Zero G Value: ");
	Serial.println(calArray[0],2);
	Serial.print("X-axis Step Value: ");
	Serial.println(calArray[1],2);	

	Serial.print("Y-axis Zero G Value: ");
	Serial.println(calArray[2],2);
	Serial.print("Y-axis Step Value: ");
	Serial.println(calArray[3],2);

	Serial.print("Z-axis Zero G Value: ");
	Serial.println(calArray[4],2);
	Serial.print("Z-axis Step Value: ");
	Serial.println(calArray[5],2);

	//force user to press button to continue
	//allows time to record the calibration value for future use
	Serial.println("\nPress Button to Continue...");
	while(buttonPressed() == false);
	
	//calibration has been run
	calFlag = true;

}

void EYW::Accelerometer::calibrate(float x1, float x2, float y1, float y2, float z1, float z2)
//user provides calibration data obtained from a .calibrate() function call
//Calibration data is usually stable if there aren't large temp/weather changes
{
	calArray[0] = x1;
	calArray[1] = x2;
	calArray[2] = y1;
	calArray[3] = y2;
	calArray[4] = z1;
	calArray[5] = z2;

	//a read out to confirm the calibration data was input correctly
	Serial.println("Calibration Complete!\n");
	Serial.print("X-axis Zero G Value: ");
	Serial.println(calArray[0],2);
	Serial.print("X-axis Step Value: ");
	Serial.println(calArray[1],2);	

	Serial.print("Y-axis Zero G Value: ");
	Serial.println(calArray[2],2);
	Serial.print("Y-axis Step Value: ");
	Serial.println(calArray[3],2);

	Serial.print("Z-axis Zero G Value: ");
	Serial.println(calArray[4],2);
	Serial.print("Z-axis Step Value: ");
	Serial.println(calArray[5],2);

	//force the user to press a button to continue
	//allows user to verify the values are accurate
	Serial.println("\nPress Button to Continue...");
	while(buttonPressed() == false);

	//calibration has been run
	calFlag = true;

}

void EYW::Accelerometer::alarm (void)
//sound the alarm on the speaker pin and the LED pin
//sound 5 times, at 392 Hz for 1/10 second each
{
	int duration = 100;
	int num = 5;
	int freq = 392;

	for (int i = 0; i < num; i++)
	{
		digitalWrite(ledPin,HIGH);
		tone(speakerPin, freq, duration); delay(duration);
		digitalWrite(ledPin, LOW);
		if (i>0) delay(duration);
	}
}

void EYW::Accelerometer::alarm (int num, int freq, int dur)
//sound the alarm on the speaker pin and the LED pin
//sounds for 'num' times, at 'freq' Hz for 'dur' seconds each
{
	for (int i = 0; i < num; i++)
	{
		digitalWrite(ledPin,HIGH);
		tone(speakerPin, freq, dur); delay(dur);
		digitalWrite(ledPin, LOW);
		if (i>0) delay(dur);
	}
}

void EYW::Accelerometer::led(int status)
//controls the LED on the defined led pin #
//takes an argument of true/HIGH/1 and false/LOW/0
{
	if (status == 1) digitalWrite(ledPin, HIGH);
	else if (status == 0) digitalWrite(ledPin, LOW);
	else return;
}

float EYW::Accelerometer::getAccel(void)
//read the acceleration data from the analog pins
//computes the resultant magnitude
{
	float accelMag;
	accelArray[0] = readAccels(xPin, 1);
	accelArray[1] = readAccels(yPin, 1);
	accelArray[2] = readAccels(zPin, 1);
	
	//accleration vector is the resultant of each reading
	accelMag = sqrt(accelArray[0]*accelArray[0] + accelArray[1]*accelArray[1] + accelArray[2]*accelArray[2]);
	return (accelMag);
}

float EYW::Accelerometer::getAccel(char axis)
//read the acceleration data from the specified axis
//constants xAxis, yAxis, and zAxis can be used
//as well as (char) x, y, z
{
	float retVal;
	switch (axis)
	{
		case 'x':
			retVal = (readAccels(xPin, 1));
			return retVal;
		case 'y':
			retVal = (readAccels(yPin, 1));
			return retVal;
		case 'z':
			retVal = (readAccels(zPin, 1));
			return retVal;
	}
}

float EYW::Accelerometer::getAccelAvg(int num)
//reads the acceleration data from the analog pins, and averages 'num' readings for each axis
//returns the magnitude of the resultant accleration vector
{
	float accelMag;
	accelArray[0] = readAccels(xPin, num);
	accelArray[1] = readAccels(yPin, num);
	accelArray[2] = readAccels(zPin, num);

	accelMag = sqrt(accelArray[0]*accelArray[0] + accelArray[1]*accelArray[1] + accelArray[2]*accelArray[2]);
	return (accelMag);
}

float EYW::Accelerometer::getAccelAvg(char axis, int num)
//reads the acceleration data from the specified axis, and averages 'num' readings for each axis
//constants xAxis, yAxis, and zAxis can be used, as well as (char) x, y, z
{
	switch (axis)
	{
		case 'x':
			return (readAccels(xPin, num));
		case 'X':
			return (readAccels(xPin, num));
		case 'y':
			return (readAccels(yPin, num));
		case 'Y':
			return (readAccels(yPin, num));
		case 'z':
			return (readAccels(zPin, num));
		case 'Z':
			return (readAccels(zPin, num));
	}

}

boolean EYW::Accelerometer::buttonPressed()
//checks if the pushbutton has been pressed
{
	int buttonState;

	//this debounce is added to prevent unintentional button depressess
	//required for less than ideal mechanical switches
	if ((millis() - elapsed) > debounce)
	{
		buttonState = digitalRead(buttonPin);
		elapsed = millis();
	        if (buttonState == HIGH) return true;
	}
	return false;
}

float EYW::Accelerometer::readAccels(int axis, int num)
//actual call to read each analog pin on 'num' number of times.
{
	long reading = 0;
	float acc;
	
	//reading the pin once, with a delay of 1 mS allows the pin to "settle"
	analogRead(axis);
	delay(1);
	
	for (int i = 0; i < num; i++)
	{
		reading += analogRead(axis);
		delay(1);
	}

	//find the mean of the readings
	reading /= num;
	
	//if calibration has been run, use the calibration values to return the acceleration in G's
	if (calFlag == true)
	{
		switch (axis)
		{
			case 14:
				acc = ((float)reading - calArray[0])/(calArray[1]);
				return acc;
			case 15:
				acc = ((float)reading - calArray[2])/(calArray[3]);
				return acc;
			case 16:
				acc = ((float)reading - calArray[4])/(calArray[5]);
				return acc;
		}
	}
	
	//otherwise return the raw reading (the raw reading is required for 
	//the calibration routine and debugging
	return (float)reading;

}
//****************************************************************************************************
//-------------------------------------RangFinder Functions-----------------------------------------
//****************************************************************************************************

void EYW::RangeFinder::begin(void)
//Initialize the range finder default button, led, and speaker pin #s
//set max pulse-wait time (correlates to max distance sensor will look for)
{
	//default pin #s
	buttonPin = 2;
	ledPin = 4;
	speakerPin = 5;

	triggerPin = 6;
	echoPin = 7;

	pinMode(buttonPin, INPUT);
	pinMode(ledPin, OUTPUT);
	pinMode(speakerPin, OUTPUT);
	pinMode(triggerPin, OUTPUT);
	pinMode(echoPin, INPUT);


	maxWait = 9000;
	conversion = 58;

}

void EYW::RangeFinder::begin(int bP, int lP, int sP)
//Initialize the range finder with user supplied pins for the button, LED and speaker
{
	buttonPin = bP;
	ledPin = lP;
	speakerPin = sP;

	triggerPin = 6;
	echoPin = 7;

	pinMode(buttonPin, INPUT);
	pinMode(ledPin, OUTPUT);
	pinMode(speakerPin, OUTPUT);
	pinMode(triggerPin, OUTPUT);
	pinMode(echoPin, INPUT);


	maxWait = 9000;
	conversion = 58;
}

void EYW::RangeFinder::begin(int bP, int lP, int sP, int tP, int eP)
//Initialize the range finder with user supplied pins for the button, LED and speaker
//Initialize the range finder trigger pin (tP) and echo pin (eP)
{
	buttonPin = bP;
	ledPin = lP;
	speakerPin = sP;

	triggerPin = tP;
	echoPin = eP;

	pinMode(buttonPin, INPUT);
	pinMode(ledPin, OUTPUT);
	pinMode(speakerPin, OUTPUT);
	pinMode(triggerPin, OUTPUT);
	pinMode(echoPin, INPUT);


	maxWait = 9000;
	conversion = 58;
}

void EYW::RangeFinder::calibrate(unsigned int checkDist)
//Calibration routine checks that the sensor is registering the correct values
//If measured range matches checkDist, the LED will stay solid, indicating success
//if range does not match, LED will blink continuously, and alarm will beep
//if checkDist is equal to 0, calibrate will not run
{
	unsigned long distance = getDistance();	
	if (checkDist == 0) return;
	
	while(1)
	{
		delay(100);
		distance = getDistance();
		if (abs(distance - checkDist) <= 1)
		{
			alarm(1,659,3000);
			return;
		}

		else 
		{
			alarm(1,659,1000);
		}
	}
}

void EYW::RangeFinder::calibrate(unsigned int checkDist, int change)
//Calibration routine checks that the sensor is registering the correct values
//change takes both negative/positive values to alter the conversion factor (microseconds per cm)
//if sensor is not registering correct values, try running calibrate(5, 1), or calibrate(5, -1) until correct
{

	conversion += change;
	unsigned long distance = getDistance();	
	
	while(1)
	{
		distance = getDistance();
		if (abs(distance - checkDist) <= 1)
		{
			alarm(1,659,3000);
			return;
		}

		else 
		{
			alarm(3,659,1000);
		}
	}
}


void EYW::RangeFinder::alarm (void)
//sound the alarm on the speaker pin and the LED pin
//sound 6 times, at 659 Hz for 1/20 second each
{
	int duration = 50;
	int num = 6;
	int freq = 659;

	
	for (int i = 0; i < num; i++)
	{
		digitalWrite(ledPin,HIGH);
		tone(speakerPin, freq, duration); delay(duration);
		digitalWrite(ledPin, LOW);
		if (i>0) delay(duration);
	}
}

void EYW::RangeFinder::alarm (int num, int freq, int dur)
//sound the alarm on the speaker pin and the LED pin
//sounds for 'num' times, at 'freq' Hz for 'dur' seconds each
{
	for (int i = 0; i < num; i++)
	{
		digitalWrite(ledPin,HIGH);
		tone(speakerPin, freq, dur); delay(dur);
		digitalWrite(ledPin, LOW);
		if (i>0) delay(dur);
	}
}

void EYW::RangeFinder::led(int status)
//controls the LED on the defined led pin #
//takes an argument of true/HIGH/1 and false/LOW/0
{
	if (status == 1) digitalWrite(ledPin, HIGH);
	else if (status == 0) digitalWrite(ledPin, LOW);
	else return;
}

int EYW::RangeFinder::getDistance()
//sets the trigger pin high and listens on the echo pin for the ultrasonic return pulse
//returns distance in centimeters
{
	
	unsigned long duration;

	digitalWrite(triggerPin, LOW);
	delayMicroseconds(2);
	digitalWrite(triggerPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(triggerPin, LOW);

	duration = pulseIn(echoPin, HIGH, maxWait);
	if (duration == 0) return 999;
	else return ((int)duration / conversion);
}

