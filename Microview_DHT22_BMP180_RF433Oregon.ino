// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

#include "DHT.h"
#include <MicroView.h>

#include <Wire.h>
#define DELAY_MEASURE 120000		//1min
#define DELAY_BATTERY 60000 //1Min
#define DELAY_DISPLAY 60000
#define DURATION_DISPLAY 10000
#define DELAY_BUTTON 500
//#define THN132N
//#define BTHR968 KO
//#define BMP_180


#include "MAX17043.h"
MAX17043 batteryMonitor;
#define BATT_CRITIC_LEVEL 10
long  previousBatMillis  =   0 ;
float previousBatValue =0;
float cellVoltage =0;
float stateOfCharge =0;
float deriv =0;
float previousderiv=0;
unsigned int counterdelay =1;

long  previousMillis  =   0 ;   // will store last time LED was updated
bool firstdata = false;

//bool firstdisplay=false;
long previousDisplayMillis =0;
bool bScreenOn=false;

float h = 0;
float t = 0; // Read temperature as Celsius
float f = 0; // Read temperature as Fahrenheit
float hi = 0;
float _previous_hi = 0;
float _previous_h = 0;
float _previous_f = 0;
float _previous_t = 0;

int buttonPin = A0;     	// push button pin
int buttonState = 0;		// variable to store the pushbutton status
long  previousButtMillis  =   0 ;



// You will need to create an SFE_BMP180 object, here called "pressure":

#ifdef BMP_180
#include <SFE_BMP180.h>
SFE_BMP180 pressure;
#define ALTITUDE 147 // Altitude of Toulouse in meters
double T,P,p0,a;
#endif

#define DHTPIN 2     // what pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor for normal 16mhz Arduino
DHT dht(DHTPIN, DHTTYPE);
// NOTE: For working with a faster chip, like an Arduino Due or Teensy, you
// might need to increase the threshold for cycle counts considered a 1 or 0.
// You can do this by passing a 3rd parameter for this threshold.  It's a bit
// of fiddling to find the right value, but in general the faster the CPU the
// higher the value.  The default for a 16mhz AVR is a value of 6.  For an
// Arduino Due that runs at 84mhz a value of 30 works.
// Example to initialize DHT sensor for Arduino Due:
//DHT dht(DHTPIN, DHTTYPE, 30);


const byte TX_PIN = 3;

const unsigned long TIME = 512;
const unsigned long TWOTIME = TIME*2;

#define SEND_HIGH() digitalWrite(TX_PIN, HIGH)
#define SEND_LOW() digitalWrite(TX_PIN, LOW)

// Buffer for Oregon message
#ifdef THN132N
byte OregonMessageBuffer[8];
#elif defined(BTHR968)
byte OregonMessageBuffer[10];
#else
byte OregonMessageBuffer[9];
#endif

/**
* \brief    Send logical "0" over RF
* \details  azero bit be represented by an off-to-on transition
* \         of the RF signal at the middle of a clock period.
* \         Remenber, the Oregon v2.1 protocol add an inverted bit first
*/
inline void sendZero(void)
{
	SEND_HIGH();
	delayMicroseconds(TIME);
	SEND_LOW();
	delayMicroseconds(TWOTIME);
	SEND_HIGH();
	delayMicroseconds(TIME);
}

/**
* \brief    Send logical "1" over RF
* \details  a one bit be represented by an on-to-off transition
* \         of the RF signal at the middle of a clock period.
* \         Remenber, the Oregon v2.1 protocol add an inverted bit first
*/
inline void sendOne(void)
{
	SEND_LOW();
	delayMicroseconds(TIME);
	SEND_HIGH();
	delayMicroseconds(TWOTIME);
	SEND_LOW();
	delayMicroseconds(TIME);
}

/**
* Send a bits quarter (4 bits = MSB from 8 bits value) over RF
*
* @param data Source data to process and sent
*/

/**
* \brief    Send a bits quarter (4 bits = MSB from 8 bits value) over RF
* \param    data   Data to send
*/
inline void sendQuarterMSB(const byte data)
{
	(bitRead(data, 4)) ? sendOne() : sendZero();
	(bitRead(data, 5)) ? sendOne() : sendZero();
	(bitRead(data, 6)) ? sendOne() : sendZero();
	(bitRead(data, 7)) ? sendOne() : sendZero();
}

/**
* \brief    Send a bits quarter (4 bits = LSB from 8 bits value) over RF
* \param    data   Data to send
*/
inline void sendQuarterLSB(const byte data)
{
	(bitRead(data, 0)) ? sendOne() : sendZero();
	(bitRead(data, 1)) ? sendOne() : sendZero();
	(bitRead(data, 2)) ? sendOne() : sendZero();
	(bitRead(data, 3)) ? sendOne() : sendZero();
}

/******************************************************************/
/******************************************************************/
/******************************************************************/

/**
* \brief    Send a buffer over RF
* \param    data   Data to send
* \param    size   size of data to send
*/
void sendData(byte *data, byte size)
{
	for(byte i = 0; i < size; ++i)
	{
		sendQuarterLSB(data[i]);
		sendQuarterMSB(data[i]);
	}
}

/**
* \brief    Send an Oregon message
* \param    data   The Oregon message
*/
void sendOregon(byte *data, byte size)
{
	sendPreamble();
	//sendSync();
	sendData(data, size);
	sendPostamble();
}

/**
* \brief    Send preamble
* \details  The preamble consists of 16 "1" bits
*/
inline void sendPreamble(void)
{
	byte PREAMBLE[]={
		0xFF,0xFF  };
	sendData(PREAMBLE, 2);
}

/**
* \brief    Send postamble
* \details  The postamble consists of 8 "0" bits
*/
inline void sendPostamble(void)
{
#ifdef THN132N
	sendQuarterLSB(0x00);
#else
	byte POSTAMBLE[]={
		0x00  };
	sendData(POSTAMBLE, 1);
#endif
}

/**
* \brief    Send sync nibble
* \details  The sync is 0xA. It is not use in this version since the sync nibble
* \         is include in the Oregon message to send.
*/
inline void sendSync(void)
{
	sendQuarterLSB(0xA);
}

/******************************************************************/
/******************************************************************/
/******************************************************************/

/**
* \brief    Set the sensor type
* \param    data       Oregon message
* \param    type       Sensor type
*/
inline void setType(byte *data, byte* type)
{
	data[0] = type[0];
	data[1] = type[1];
}

/**
* \brief    Set the sensor channel
* \param    data       Oregon message
* \param    channel    Sensor channel (0x10, 0x20, 0x30)
*/
inline void setChannel(byte *data, byte channel)
{
	data[2] = channel;
}

/**
* \brief    Set the sensor ID
* \param    data       Oregon message
* \param    ID         Sensor unique ID
*/
inline void setId(byte *data, byte ID)
{
	data[3] = ID;
}
void setLastBaro(byte *data, byte lastbyte)
{
	data[9] = lastbyte;
}

/**
* \brief    Set the sensor battery level
* \param    data       Oregon message
* \param    level      Battery level (0 = low, 1 = high)
*/
void setBatteryLevel(byte *data, byte level)
{
	if(!level) data[4] = 0x0C; //low level
	else data[4] = 0x00;
}

/**
* \brief    Set the sensor temperature
* \param    data       Oregon message
* \param    temp       the temperature
*/
void setTemperature(byte *data, float temp)
{
	// Set temperature sign
	if(temp < 0)
	{
		data[6] = 0x08;
		temp *= -1;
	}
	else
	{
		data[6] = 0x00;
	}

	// Determine decimal and float part
	int tempInt = (int)temp;
	int td = (int)(tempInt / 10);
	int tf = (int)round((float)((float)tempInt/10 - (float)td) * 10);

	int tempFloat =  (int)round((float)(temp - (float)tempInt) * 10);

	// Set temperature decimal part
	data[5] = (td << 4);
	data[5] |= tf;

	// Set temperature float part
	data[4] |= (tempFloat << 4);
}

/**
* \brief    Set the sensor humidity
* \param    data       Oregon message
* \param    hum        the humidity
*/
void setHumidity(byte* data, byte hum)
{
	data[7] = (hum/10);
	data[6] |= (hum - data[7]*10) << 4;
}
/**
* \brief    Set the sensor humidity
* \param    data       Oregon message
* \param    hum        the humidity
*/
void setBarometric(byte* data, int bar)
{
	byte l_bardata = (byte)(bar-856);
	if (l_bardata>255)
	l_bardata=255;
	data[8] = l_bardata;

	Serial.println();
	Serial.print("Bar raw: ");
	Serial.print(l_bardata,HEX);
	Serial.print("\n");

}

/**
* \brief    Sum data for checksum
* \param    count      number of bit to sum
* \param    data       Oregon message
*/
int Sum(byte count, const byte* data)
{
	int s = 0;

	for(byte i = 0; i<count;i++)
	{
		s += (data[i]&0xF0) >> 4;
		s += (data[i]&0xF);
	}

	if(int(count) != count)
	s += (data[count]&0xF0) >> 4;

	return s;
}

/**
* \brief    Calculate checksum
* \param    data       Oregon message
*/
void calculateAndSetChecksum(byte* data)
{
#ifdef THN132N
	int s = ((Sum(6, data) + (data[6]&0xF) - 0xa) & 0xff);

	data[6] |=  (s&0x0F) << 4;
	data[7] =  (s&0xF0) >> 4;
#elif defined(BTHR968)

	data[9] = 0x31;
#else
	data[8] = ((Sum(8, data) - 0xa) & 0xFF);
#endif
}
void DisplayDbgMessageOnMV(char*string)
{
	uView.setCursor(0,0);
	uView.print(string);
	uView.display();

}

void setup() {
	pinMode(TX_PIN, OUTPUT);
	Wire.begin();
	
	pinMode(buttonPin, INPUT);  	// initialize the pushbutton pin as an input
	digitalWrite(buttonPin,HIGH);  	// set Internal pull-up

	Serial.begin(9600);
	Serial.println("DHTxx test!");
	Serial.println("\n[Oregon V2.1 encoder]");

	SEND_LOW();

#ifdef THN132N
	// Create the Oregon message for a temperature only sensor (TNHN132N)
	byte ID[] = {
		0xEA,0x4C  };
#elif defined(BTHR968)

	byte ID[] = {
		0x5A,0x6D  };
#else
	// Create the Oregon message for a temperature/humidity sensor (THGR2228N)
	byte ID[] = {
		0x1A,0x2D  };
#endif
	setType(OregonMessageBuffer, ID);

#ifdef BTHR968
	setChannel(OregonMessageBuffer, 0x4);
	setId(OregonMessageBuffer, 0x7A);
#else
	setChannel(OregonMessageBuffer, 0x20);
	setId(OregonMessageBuffer, 0xBB);
#endif

	
	//dht.begin();

#ifdef BMP_180
	if (pressure.begin())
	{
		Serial.println("BMP180 init success");
		DisplayDbgMessageOnMV("BMP180 init success");
	}
	else
	{
		// Oops, something went wrong, this is usually a connection problem,
		// see the comments at the top of this sketch for the proper connections.
		Serial.println("BMP180 init fail\n\n");
		DisplayDbgMessageOnMV("BMP180 init fail");
		while(1); // Pause forever.
	}
#endif
	firstdata=true;
	//firstdisplay=true;


}

void GetDataAndSend()
{
	Serial.print("\nGetDataAndSend\n");
	// Wait a few seconds between measurements.
	//if (firstdata == true)
	dht.begin();
	delay(2000);
	
	char status;
#ifdef BMP_180

	// Loop here getting pressure readings every 10 seconds.

	// If you want sea-level-compensated pressure, as used in weather reports,
	// you will need to know the altitude at which your measurements are taken.
	// We're using a constant called ALTITUDE in this sketch:

	Serial.println();
	Serial.print("provided altitude: ");
	Serial.print(ALTITUDE,0);
	Serial.print(" meters, ");
	Serial.print(ALTITUDE*3.28084,0);
	Serial.println(" feet\n");

	// If you want to measure altitude, and not pressure, you will instead need
	// to provide a known baseline pressure. This is shown at the end of the sketch.

	// You must first get a temperature measurement to perform a pressure reading.

	// Start a temperature measurement:
	// If request is successful, the number of ms to wait is returned.
	// If request is unsuccessful, 0 is returned.

	status = pressure.startTemperature();
	if (status != 0)
	{
		// Wait for the measurement to complete:
		delay(status);

		// Retrieve the completed temperature measurement:
		// Note that the measurement is stored in the variable T.
		// Function returns 1 if successful, 0 if failure.

		status = pressure.getTemperature(T);
		if (status != 0)
		{
			// Print out the measurement:
			Serial.print("temperature: ");
			Serial.print(T,2);
			Serial.print(" deg C\n");


			// Start a pressure measurement:
			// The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
			// If request is successful, the number of ms to wait is returned.
			// If request is unsuccessful, 0 is returned.

			status = pressure.startPressure(3);
			if (status != 0)
			{
				// Wait for the measurement to complete:
				delay(status);

				// Retrieve the completed pressure measurement:
				// Note that the measurement is stored in the variable P.
				// Note also that the function requires the previous temperature measurement (T).
				// (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
				// Function returns 1 if successful, 0 if failure.

				status = pressure.getPressure(P,T);
				if (status != 0)
				{
					// Print out the measurement:
					Serial.print("absolute pressure: ");
					Serial.print(P,2);
					Serial.print(" mb, ");


					// The pressure sensor returns abolute pressure, which varies with altitude.
					// To remove the effects of altitude, use the sealevel function and your current altitude.
					// This number is commonly used in weather reports.
					// Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
					// Result: p0 = sea-level compensated pressure in mb

					p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)
					Serial.print("relative (sea-level) pressure: ");
					Serial.print(p0,2);
					Serial.print(" mb, ");

					// On the other hand, if you want to determine your altitude from the pressure reading,
					// use the altitude function along with a baseline pressure (sea-level or other).
					// Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
					// Result: a = altitude in m.

					a = pressure.altitude(P,p0);
					Serial.print("computed altitude: ");
					Serial.print(a,0);
					Serial.print(" meters, ");

				}
				else
				{		Serial.println("error retrieving pressure measurement\n");
					DisplayDbgMessageOnMV("error retrieving pressure measurement");
				}
			}else
			{		Serial.println("error starting pressure measurement\n");
				DisplayDbgMessageOnMV("error starting pressure measurement");
			}
			
		}
		else
		{		Serial.println("error retrieving temperature measurement\n");
			DisplayDbgMessageOnMV("error retrieving temperature measurement");
		}
	}else
	{		Serial.println("error starting temperature measurement\n");
		DisplayDbgMessageOnMV("error starting temperature measurement");
	}

#endif //BMP_180
	// Reading temperature or humidity takes about 250 milliseconds!
	// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
	h = dht.readHumidity();
	// Read temperature as Celsius
	t = dht.readTemperature();
	// Read temperature as Fahrenheit
	//f = dht.readTemperature(true);

	// Check if any reads failed and exit early (to try again).
	if (isnan(h) || isnan(t) /*|| isnan(f)*/) {
		Serial.println("Failed to read from DHT sensor!");
		DisplayDbgMessageOnMV("Failed to read from DHT sensor!");
		return;
	}




	// Compute heat index
	// Must send in temp in Fahrenheit!
	//hi = dht.computeHeatIndex(f, h);

	Serial.print("Humidity: ");
	Serial.print(h);
	Serial.print(" %\t");
	Serial.print("Temperature: ");
	Serial.print(t);
	Serial.print(" °C ");
	/*Serial.print(f);
	Serial.print(" °F\t");*/
	//Serial.print("Heat index: ");
	//Serial.print(hi);
	//Serial.println(" °F\n");

	digitalWrite(DHTPIN, LOW);

	if (((unsigned int) h!=(unsigned int) _previous_h)||(t!=_previous_t))
	{
		Serial.print("GetDataAndSend : new data\n");

		// Get Temperature, humidity and battery level from sensors
		// (ie: 1wire DS18B20 for température, ...)
		if (stateOfCharge < BATT_CRITIC_LEVEL)
			setBatteryLevel(OregonMessageBuffer, 0); // 0 : low, 1 : high
		else
			setBatteryLevel(OregonMessageBuffer, 1); // 0 : low, 1 : high
		setTemperature(OregonMessageBuffer, t);

	#ifndef THN132N
		// Set Humidity
		setHumidity(OregonMessageBuffer, h);
	#endif
	#ifdef BTHR968
		setBarometric(OregonMessageBuffer,(int) p0);
		//setLastBaro(OregonMessageBuffer,0x31);
	#endif
		// Calculate the checksum
		calculateAndSetChecksum(OregonMessageBuffer);

		//TEST
		/*OregonMessageBuffer[0]= 0x5A;
	OregonMessageBuffer[1]= 0x6D;
	OregonMessageBuffer[2]= 0x00;
	OregonMessageBuffer[3]= 0x7A;
	OregonMessageBuffer[4]= 0x10;
	OregonMessageBuffer[5]= 0x23;
	OregonMessageBuffer[6]= 0x30;
	OregonMessageBuffer[7]= 0x83;
	OregonMessageBuffer[8]= 0x86;
	OregonMessageBuffer[9]= 0x31;*/


		// Show the Oregon Message
		for (byte i = 0; i < sizeof(OregonMessageBuffer); ++i)   {
			Serial.print(OregonMessageBuffer[i] >> 4, HEX);
			Serial.print(OregonMessageBuffer[i] & 0x0F, HEX);
		}

		// Send the Message over RF
		sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));
		// Send a "pause"
		SEND_LOW();
		delayMicroseconds(TWOTIME*8);
		// Send a copie of the first message. The v2.1 protocol send the
		// message two time
		sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));

		// Wait for 30 seconds before send a new message
		SEND_LOW();
		
		_previous_h=h;
		_previous_t=t;
	}
	else
		Serial.print("GetDataAndSend : same data\n");	

}
void GetBatteryData()
{
	Serial.print("\nGetBatteryData\n");	
	cellVoltage = batteryMonitor.getVCell();
	/*Serial.print("Voltage:\t\t");
Serial.print(cellVoltage, 4);
Serial.println("V");*/

	

	stateOfCharge = batteryMonitor.getSoC();
	/*Serial.print("State of charge:\t");
Serial.print(stateOfCharge);
Serial.println("%");*/

	

	//Calcul TMP
	if ((stateOfCharge==previousBatValue)||(previousBatValue==0))
	{
		counterdelay++;
		deriv =0;
	}
	else
	{
		deriv = (stateOfCharge-previousBatValue)/(DELAY_BATTERY*counterdelay*1.0)*60000;
		counterdelay=1;

		
	}
	
	


	
	previousBatValue = stateOfCharge;

}
void DisplayData()
{
	Serial.print("\nDisplayData\n");
	uView.begin();		// begin of MicroView
	
        uView.clear(PAGE);	// erase the memory buffer, when next uView.display() is called, the OLED will be cleared.
	uView.setCursor(0,0);
	uView.print("Hum ");
	uView.setCursor(30,0);
	uView.print(h);

	//uView.print(" % ");

	uView.setCursor(0,10);
	uView.print("Temp ");
	uView.setCursor(30,10);
	uView.print(t);
#ifdef BMP_180
	uView.setCursor(0,20);
	uView.print("Pr ");
	uView.setCursor(20,20);
	uView.print(p0);
	uView.setCursor(0,30);
	uView.print("Tp ");
	uView.setCursor(30,30);
	uView.print(T);
	uView.setCursor(35,30);
#endif
	uView.setCursor(0,20);		
	uView.print(cellVoltage);			
	uView.print("V");
	uView.setCursor(0,30);		
	uView.print(stateOfCharge);			
	uView.print("%");

	if (deriv!=0)
	{
		uView.setCursor(0,40);		
		uView.print(deriv);			
		uView.print("%/min");
	}


	uView.display();
	bScreenOn = true;



}

void ClearScreen()
{
	Serial.print("\nClearScreen\n");	
	uView.clear(PAGE);
	uView.display();
	uView.end();
	bScreenOn = false;
	

}
bool GetButtonState()
{
	//Serial.print("GetButtonState ...\n");
	int l_iState = 0;
	l_iState = digitalRead(buttonPin);
	if (l_iState!=buttonState)
	{
		Serial.print("GetButtonState State Changed\n");
		buttonState = l_iState;
		if ((bScreenOn == false))
			bScreenOn = true;
                 
		return true;
	
	}
	else
	{
		return false;
	
	}
}
void loop() {


	unsigned   long  currentMillis  =  millis ();



	if (( currentMillis  -  previousMillis  >  DELAY_MEASURE )||(firstdata==true))
	{
		
		
		previousMillis  =  currentMillis ;
		GetDataAndSend();
		
		if (firstdata == true)
		{
			GetBatteryData();
			DisplayData();			
			firstdata=false;
		}
		
	}


	
/*	if (( currentMillis  -  previousBatMillis  >  DELAY_BATTERY ))
	{
		previousBatMillis  =  currentMillis ;
		GetBatteryData();
		
		
	}
*/
	if (( currentMillis  -  previousButtMillis)  >  DELAY_BUTTON )
	{
		previousButtMillis  =  currentMillis ;
		
		if (GetButtonState() == true) // if changed state
		{
			if ((bScreenOn==true)&&(buttonState == LOW))
			{
				previousDisplayMillis = currentMillis;
				GetBatteryData();
				DisplayData();
			}
		
		}
		
		
	}
	if (( (currentMillis  -  previousDisplayMillis)  >  DURATION_DISPLAY )&&(bScreenOn==true))
	{
		ClearScreen();
				
	}


}

