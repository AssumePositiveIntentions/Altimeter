#include "Arduino.h"
#include "Servo.h"

#ifndef EYW_H
#define EYW_H

//constants used for i2c communication with the altimeter
#define BMP180_ADDR 0x77 // 7-bit address
#define	BMP180_REG_CONTROL 0xF4
#define	BMP180_REG_RESULT 0xF6
#define	BMP180_COMMAND_TEMPERATURE 0x2E
#define	BMP180_COMMAND_PRESSURE0 0x34
#define	BMP180_COMMAND_PRESSURE1 0x74
#define	BMP180_COMMAND_PRESSURE2 0xB4
#define	BMP180_COMMAND_PRESSURE3 0xF4
//constants used for i2c communication with the altimeter

static const int debounce = 200;	//used to debounce the pushbutton
static unsigned long elapsed = 0; 	//used to debounce the pushbutton

//constants for retrieving accelerometer axis values (may not be needed)
static const char xAxis = 'x';		
static const char yAxis = 'y';
static const char zAxis = 'z';


//Main class EYW contains all sensor classes
class EYW
{
	public:
		class Altimeter
		//The Altimeter class is used to control the BMP180 pressure sensor
		//and uses code from Sparkfuns library https://github.com/sparkfun/BMP180_Breakout
		{
			public:
			//all public function and variables are available for you to call and use in your sketch

				char begin();
				// call .begin() to initialize BMP180 before use
				// returns 1 if success, 0 if failure (bad component or I2C bus shorted?)
				// defaults to the button pin # 2, LED on pin # 4 and the speaker on # 5
				
				char begin(int bP, int lP, int sP);
				// call .begin() to initialize BMP180 before use
				// returns 1 if success, 0 if failure (bad component or I2C bus shorted?)
				// inputs for button pin (bP), LED pin (lP), and speaker pin (sP)

				char calibrate(int num);
				//call .calibrate() to calibrate zero height using 'num' numbers of readings

				char calibrate(int num, float filterVal);
				// call .calibrate() to calibrate zero height, optionally provide
				// 'filterVal' to set the Low Pass Filter from 0.0-1.0 (1.0 is no filtering) 

				void alarm(void);
				// sound the alarm on speakerPin. Default is 2x at 880 Hz for 250 mS

				void alarm(int num, int freq, int dur);
				//sound the alarm on num number of times at 'freq' frequency, for duration dur
				
				void led(int status);
				//controls the LED on the defined led pin # takes an argument of true/HIGH/1 and false/LOW/0

				float getHeight(void);
				// call .getHeight to use the sensors temperature and pressure readings
				// to determine your current altitude. .calibrate() must be called first
				// for accurate readingsconstants used for i2c communication with the altimeter

				float getHeightAvg(int num);
				// call .getHeightAvg(#) to average 'num' number of .getHeight() calls
				// usefull for more accurate readings


			private:
			//all private functions and variables cannot be accessed from your sketch

				char startTemperature(void);
				// command BMP180 to start a temperature measurement

                		char getTemperature(float &T);
				// return temperature measurement from previous startTemperature command
				// places returned value in T variable (deg C)
				// returns 1 for success, 0 for fail

				char startPressure(char oversampling);
				// command BMP180 to start a pressure measurement
				// oversampling: 0 - 3 for oversampling value

				char getPressure(float &P, float &T);
				// return absolute pressure measurement from previous startPressure command
				// note: requires previous temperature measurement in variable T
				// places returned value in P variable (mbar)
				// returns 1 for success, 0 for fail

				float sealevel(float P, float A);
				// convert absolute pressure to sea-level pressure (as used in weather data)
				// P: absolute pressure (mbar), A: current altitude (meters)
				// returns sealevel pressure in mbar. This is used for calibrating your zero height

				float altitude(float P, float P0);
				// convert absolute pressure to altitude (given baseline pressure; sea-level, runway, etc.)
				// P: absolute pressure (mbar), P0: fixed baseline pressure (mbar)
				// returns signed altitude in meters. 

				char readInt(char address, int &value);
				// read an signed int (16 bits) from a BMP180 register
				// address: BMP180 register address
				// value: external signed int for returned value (16 bits)
				// returns 1 for success, 0 for fail, with result in value

				char readUInt(char address, unsigned int &value);
				// read an unsigned int (16 bits) from a BMP180 register
				// address: BMP180 register address
				// value: external unsigned int for returned value (16 bits)
				// returns 1 for success, 0 for fail, with result in value

				char readBytes(unsigned char *values, char length);
				// read a number of bytes from a BMP180 register
				// values: array of char with register address in first location [0]
				// length: number of bytes to read back
				// returns 1 for success, 0 for fail, with read bytes in values[] array

				char writeBytes(unsigned char *values, char length);
				// write a number of bytes to a BMP180 register (and consecutive subsequent registers)
				// values: array of char with register address in first location [0]
				// length: number of bytes to write
				// returns 1 for success, 0 for fail
				
				char getError(void);
				//used to determine which errors the altimeter functions have returned

				//variables used by the i2C and calibration methods
				int AC1,AC2,AC3,VB1,VB2,MB,MC,MD;
				unsigned int AC4,AC5,AC6;
				float c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2;
				
				//variables used by begin and calibrate	
				int buttonPin, speakerPin, ledPin;
				float baselineSLP;
				char _error;

		};
		class Camera
		// The Camera class is used to control the servo motor that activates your camera
			
		{
			public:
			//all public function and variables are available for you to call and use in your sketch

				char begin(void);
				// use to initialize the inputs and outputs
				// defaults to the button pin # 2, LED on pin # 4, speaker on # 5, and servo motor on pin 3
				
				char begin(int bP, int lP, int sP);
				// inputs for button pin (bP), LED pin (lP), speaker pin (sP). Default servo motor to pin 3

				char begin(int bP, int lP, int sP, int smP);
				// inputs for button pin (bP), LED pin (lP), speaker pin (sP), and servo motor pin (smP)

				char calibrate(void);
				//used to calibrate the servo motor movement extents, 
				//defaults to servo pin # 3, with hard stops at 20 and 90 degrees
				

				char calibrate(int lS, int rS);
				//define servo pin with smP, lS left stop, and rS rightStop
				
				void alarm(void);
				//default alarm on the speaker pin and LED pin
				// sounds 1 time at 523 hertz for 1/4 of a second

				void alarm(int num, int freq, int dur);
				//Sounds the alarm on the speaker pin and the LED pin
				//for 'num' times at 'freq' frequency for time 'duration'

				void led(int status);
				//controls the LED on the defined led pin # takes an argument of true/HIGH/1 and false/LOW/0

				char beginTimer(unsigned long tl);
				//start the timer to countdown from tl in milliseconds
				
				char endTimer (void);

				char getPicture(void);
				//activates the servo to take a picture using the calibrated servo stops
				//defaults to hold the camera button for 1/2 second
				//defaults to wait 1 seconds before it can take another picture
				
				char getPicture(unsigned long pL, unsigned long pD);
				//activates the servo, and takes two arguments pL for press length (how long
				//to press the camera button), and pD for picture delay (how long to wait before
				//the camera can take another picture)

				boolean buttonPressed(void);
				//Checks if the pushbutton has been pressed. Returns true or false

				boolean timerExpired(void);
				//checks if the timer has expired. Returns true or false
				
				Servo motor; //the servo object name. You can control the servo directly 
					     //using motor.write();
			
												
			private:
			//all private functions and variables CANNOT be accessed from your sketch

				void setFlags(void);
				//set the boolean flags: buttonFlag, timerFlag, and beginCapture to false

				void setMotor(unsigned long d1, unsigned long d2);
				//commands the motor to move from the calibrated left stop to the right stop
				//using the delays d1 and d2 for each respective movement

				int servoLeftStop,servoRightStop,buttonPin,ledPin, speakerPin,servoPin;
				//these variables can only be set using the begin and calibrate functions

				unsigned long timerLength, timeElapsed, pressLength, picDelay, moveDelay;;
				
				boolean buttonFlag, timerFlag; //these variables cannot be accessed from your sketch
				
		};

		class Accelerometer

		{
			public:
				
				void begin (void);
				//Initialize the accelerometer to default button, led, and speaker pin #s
				//sets the x, y, and z axes to analog pin #s
				//provides default calibration numbers (that are likely not accurate)

				void begin (int bP, int lP, int sP);
				//Initialize the accelerometer to with user supplied pins for the button, LED and speaker
				//sets the x, y, and z axes to analog pin #s
				//provides default calibration numbers (that are likely not accurate)

				void calibrate (void);
				//Calibration routine takes samples from 6 distinct orientations
				//with reference to gravity. Prints two values for each axis
				//the zero-G reading, and the mV/G value.

				void calibrate (float x1, float x2, float y1, float y2, float z1, float z2);
				//user provides calibration data obtained from a .calibrate() function call
				
				void alarm (void);
				//sound the alarm on the speaker pin and the LED pin
				//sound 5 times, at 392 Hz for 1/10 second each
				
				void alarm(int num, int freq, int dur);
				//sound the alarm on the speaker pin and the LED pin
				//sounds for 'num' times, at 'freq' Hz for 'dur' seconds each

				void led(int status);
				//controls the LED on the defined led pin #, takes an argument of true/HIGH/1 and false/LOW/0

				float getAccel(void);
				//read the acceleration data from the analog pins
				//computes the resultant magnitude

				float getAccel(char axis);
				//read the acceleration data from the specified axis
				//constants xAxis, yAxis, and zAxis can be used
				//as well as (char) x, y, z

				float getAccelAvg(int num);
				//reads the acceleration data from the analog pins, and averages 'num' readings for each axis
				//returns the magnitude of the resultant accleration vector

				float getAccelAvg(char axis, int num);
				//reads the acceleration data from the specified axis, and averages 'num' readings for each axis
				//constants xAxis, yAxis, and zAxis can be used, as well as (char) x, y, z

				boolean buttonPressed(void);
				//checks if the pushbutton has been pressed



			private:
				
				float readAccels (int axis, int num);
				//actual call to read each analog pin on 'num' number of times.
				//Returns the raw value if calibration has not been run
				//or returns acceleration in G's if it has
				
				//private variables used to set pins, acclerations, and calibrations
				int buttonPin, speakerPin, ledPin;
				int xPin, yPin, zPin;
				float calArray[6];
				float accelArray[3];
				boolean calFlag;
				

		};	

	class RangeFinder
	{
		public:
			
				void begin (void);
				//Initialize the range finder default button, led, and speaker pin #s
				//set max pulse-wait time (correlates to max distance sensor will look for)

				void begin (int bP, int lP, int sP);
				//Initialize the range finder with user supplied pins for the button, LED and speaker

				void begin (int bP, int lP, int sP, int tP, int eP);
				//Initialize the range finder with user supplied pins for the button, LED and speaker
				//Initialize the range finder trigger pin (tP) and echo pin (eP)

				void calibrate (unsigned int checkDist);
				//Calibration routine checks that the sensor is registering the correct values
				//If measured range matches 10cm, the LED will stay solid
				//if range does not match, LED will blink continuously, and alarm will beep

				void calibrate (unsigned int checkDist, int change);
				//Calibration routine checks that the sensor is registering the correct values
				//change takes both negative/positive values to alter the conversion factor (microseconds per cm)
				//if sensor is not registering correct values, try running calibrate(5, 1), or calibrate(5, -1) to correct
				
				void alarm (void);
				//sound the alarm on the speaker pin and the LED pin
				//sound 6 times, at 659 Hz for 1/20 second each
				
				void alarm(int num, int freq, int dur);
				//sound the alarm on the speaker pin and the LED pin
				//sounds for 'num' times, at 'freq' Hz for 'dur' seconds each

				void led(int status);
				//controls the LED on the defined led pin #, takes an argument of true/HIGH/1 and false/LOW/0
				
				int getDistance();
				//sets the trigger pin high and listens on the echo pin for the ultrasonic return pulse
				//returns distance in centimeters
	
		private:
				//private variables used by begin and calibrate
				int buttonPin, speakerPin, ledPin;
				int triggerPin, echoPin, maxWait;
				int conversion;

	};				
};


#endif
