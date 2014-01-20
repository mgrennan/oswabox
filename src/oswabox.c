/*
//  oswabox.c
//  Mark Grennan - 2014-01-17
//
*/
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>

#include <wiringPi.h>
#include <gertboard.h>
#include <maxdetect.h>
#include "gps.h"

#define LED 3					 // The wiringPi pin for the LED
#define RHT03_PIN  7			 // The wiringPi pin for the RHT03
#define WIND_PIN 0

#define Version 0.3
#define wind_avg 1				 // Number of seconds to average for wind speed
#define rain_avg 1
#define samples 50				 // Number of AD samples to take
#define wait 10					 // Time delay between ADC reads

// the event counter
volatile int WindCounter = 0;
volatile int RainCounter = 0;

//#define DEBUG				 // uncomment to do debuging

// WindInterrupt:  called every time a Wind event occurs
void WindInterrupt(void)
{
	WindCounter++;
}


// RainInterrupt:  called every time a Wind event occurs
void RainInterrupt(void)
{
	RainCounter++;
}


float wind_speed()
{
	WindCounter = 0;
	delay( wind_avg * 1000 );	 // Take an average

	/* Wind is 1.492 mph per event */
	#ifdef DEBUG
	printf( "%4.2f\n", (WindCounter * 1.492) / wind_avg);
	#endif

	return ((WindCounter * 1.492) / wind_avg);

}


float rain_fall()
{
	RainCounter = 0;

	delay( rain_avg * 1000 );	 // Take an average

	/* event rate is 0.011 inches per event */
	#ifdef DEBUG
	printf( "%4.2f\n", (RainCounter * 0.011) / rain_avg );
	#endif

	return ((RainCounter * 0.011) / rain_avg);
}


//
// Return the air Humidity temp(0) or the temp in centagrade temp(1);
//
float temp(int flag)
{
	int temp, rh ;
	int newTemp, newRh ;

	temp = rh = newTemp = newRh = 0 ;

	wiringPiSetup () ;
	piHiPri       (55) ;

	sleep (1) ;

	readRHT03 (RHT03_PIN, &newTemp, &newRh) ;

	if ((temp != newTemp) || (rh != newRh))
	{
		temp = newTemp ;
		rh   = newRh ;
	}

	#ifdef DEBUG
	printf ("Temperature: %5.1f c = %5.2f f\n", temp / 10.0, ((temp /10.0) * 1.8) + 32) ;
	printf ("   Humidity: %5.1f%%\n", rh / 10.0, (rh / 10.0) * 3.2808) ;
	#endif

	if ( flag )
	{
		return temp / 10.0 ;
	}
	else
	{
		return rh / 10.0 ;
	}
}


//
// Return air pressure in hPa
//
float pressure()
{
	float temp = 24.9;
	float pressure = 95693.0;
	float rawAltitude = 480.17;
	float altitude = 400.72;

	// read current temp
	//    temp = readTemperature();

	// Read the current barometric pressure level
	//    pressure = readPresure();

	// To calculate altitude based on an estimated mean sea level pressure
	// (1013.25 hPa) call the function as follows, but this won't be very accurate
	//    rawAltitude = readAltitude();

	// To specify a more accurate altitude, enter the correct mean sea level
	// pressure level.  For example, if the current pressure level is 1023.50 hPa
	// enter 102350 since we include two decimal places in the integer value
	//    altitude = bmp.readAltitude(currentBMP);

	#ifdef DEBUG
	printf("Temperature:  % 4.2f C  \t= % 4.2f F\n", temp, (temp * 1.8) + 32);
	printf("Pressure:     % 4.2f hPa  \t= % 4.2f inch of mercury\n", pressure / 100.0, (pressure /100) * 0.02953);
	printf("Raw Altitude: % 4.2f m  \t= % 4.2f f\n", rawAltitude, rawAltitude * 3.2808);
	printf("Altitude:     % 4.2f m  \t= % 4.2f f\n", altitude, altitude * 3.2808);
	#endif

	return pressure / 100.0 ;
}


/*
// Read the ADC data fromt the SPI buss
// for Details see:
//    https://projects.drogon.net/understanding-spi-on-the-raspberry-pi/
//    http://ww1.microchip.com/downloads/en/DeviceDoc/21295d.pdf - Page 21
//
//      Start Bit Sel/Diff bit
//              V V
//      000000001 1xxx0000 000000000
//                 ^^^
//                 ADC Address
//
//  The bottem 10 bits are space for the returning data.
*/
long readadc(adcnum)
{
	uint8_t buff[3] = { 0b00000001, 0b10000000, 0b00000000 }
	;
	long adc;
	char *b;

	buff[1] += adcnum << 4 ;

	wiringPiSPIDataRW(0, buff, 3);

	//	adc = ((buff[1] & 3) << 8) + buff[2];
								 // 10 bits of data
	adc = ((buff[1] * 256 ) + buff[2]) & 0b1111111111 ;

	return adc;
}


/*
//
//
*/
float read_adc_dev(int pin)
{
	int i ;
	uint16_t ob ;
	float tot, avg, volt, value ;
	/*
	//   vcc ----R1--+--R2---- GND   If device is R1 its a Pull-Up
	//               |               If device is R2 its a Pull-Down
	//              ADC
	*/
	struct device
	{
		int pullup ;			 // 1 = device pulls up the Pin
		float refvolt ;			 // reverence voltage
		float resistance ;		 // resistence in ohms
		char *name ;			 // device name
	}
	dev[8] =
	{
		{ 0, 3.3,  10000.0, "LDR light sensor" },
		{ 0, 3.3,  22000.0, "TGS2600         " },
		{ 0, 3.3,  10000.0, "MiSC-2710       " },
		{ 0, 3.3, 100000.0, "MiCS=5525       " },
		{ 0, 3.3,  01000.0, "Sound           " },
		{ 0, 5.0,  10000.0, "Wind Direction  " },
		{ 0, 3.3,  10000.0, "Open            " },
		{ 0, 3.3,    990.0, "Test Voltage    " }
	} ;

								 // initialize the WiringPi API channel and speed
	if (wiringPiSPISetup (0, 1500000) < 0)
		return -1 ;				 // the mcp3008 wants clock speed between 1.35 and 3.6Mz

	ob = 0;
	tot = 0;
	for (i=0; i<samples; i++)	 // Read samples
	{
		ob =  readadc(pin);
		tot = tot + (ob * 1.0) ;
		delay( wait ) ;
	}
	avg = tot / samples ;		 // calculate the average of the readings

	// See http://en.wikipedia.org/wiki/Voltage_divider

								 // calculate the average voltage
	volt = (dev[pin].refvolt / 1023.0) * avg ;
	// reference voltage / 10bit AD (1023) * reading

	if ( dev[pin].pullup )		 // If this is a pullup resister
	{
		value = (( dev[pin].resistance * dev[pin].refvolt ) / volt ) - dev[pin].resistance ;
	}
	else						 // this is a pulldown resister
	{
		value = dev[pin].resistance / (( dev[pin].refvolt / volt ) - 1 ) ;
	}

	#ifdef DEBUG				 /* display the average value */
	printf("Pin %d:%s  %09.2f = %06.4f volts = %010.2f Ohms\n", pin, dev[pin].name, avg, volt, value ) ;
	#endif						 /* DEBUG */

	return value ;
}


main (int argc, char **argv)
{
	int GPSflag = 0;
	int Headerflag = 0;
	int Bannerflag = 1;
	char *output = NULL;			 // h=HUMAN, c=CSV, s=SQL
	int index;
	int c;
	float value;

	loc_t data ;				 // Create GPS structure

	opterr = 0;

	while ((c = getopt (argc, argv, "bgho:")) != -1)
		switch (c)
		{
			case 'b':
				Bannerflag = 0;
				break;
			case 'g':
				GPSflag = 1;
				break;
			case 'h':
				Headerflag = 1;
				break;	
			case 'o':
				output = optarg;
				break;
			case '?':
				if (optopt == 'o')
					fprintf (stderr, "Option -%c requires an argument.\n", optopt);
				else if (isprint (optopt))
					fprintf (stderr, "Unknown option `-%c'.\n", optopt);
				else
					fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
				return 1;
			default:
				abort ();
		}

	#ifdef DEBUG				 // debug command line arguments
	printf ("GPSflag = %d, output = %s\n", GPSflag, output);

	for (index = optind; index < argc; index++)
		printf ("Non-option argument %s\n", argv[index]);
	#endif						 /* DEBUG */

	if (Bannerflag)
	{
		printf("Open Source Weather and Air quality Box (OSWABox)\n");
		printf(" - Version %4.2f\n\n",Version);
	}

	//
	// Make sure all the GPI pins are set correctly
	//
	wiringPiSetup () ;
	pinMode (LED, OUTPUT) ;

	if ( GPSflag )
	{
		gps_init();				 // Open serial port
	}

	//
	// Setup Interups for the Wind and Rain Devices
	//
	if (wiringPiSetup () < 0)
	{
		printf (stderr, "Unable to setup wiringPi.\n");
		return 1;
	}

	// set Pin 17/0 generate an interrupt on high-to-low transitions
	if ( wiringPiISR (WIND_PIN, INT_EDGE_FALLING, &WindInterrupt) < 0 )
	{
		printf (stderr, "Unable to setup wind interrupts\n");
		return 1;
	}

	//
	// Start reading data
	//
	digitalWrite (LED, HIGH) ;	 // LED On

	if ( GPSflag )
	{
		gps_location(&data);	 // Read and parse the GPS info
	}

	switch((char)output[0])
	{
		case 'c':
			if (GPSflag)
			{
				printf("20%02d-%02d-%02d,",data.year, data.month, data.day);
				printf("%02d:%02d:%02d,",data.hour, data.minute, data.second);
				printf("%lf,",data.latitude);
				printf("%lf,",data.longitude);
				printf("%lf,",data.altitude);
			}
			printf("%6.2f,%6.2f,%6.2f,%6.2f,%6.2f",
				temp(1), temp(0),pressure(),wind_speed(),rain_fall() );
			for (c=0; c<8; c++)
			{
				printf(",%6.4f", read_adc_dev(c));
			}
			break;
		case 'h':
			printf("The current conditions are:\n\n");
			if ( GPSflag )
			{
				printf("      Date: 20%02d-%02d-%02d\n",data.year, data.month, data.day);
				printf("      Time: %02d:%02d:%02d\n",data.hour, data.minute, data.second);
				printf("  Latitude: %lf\n",data.latitude);
				printf(" Longitude: %lf\n",data.longitude);
				printf("  Altitude: %lf m\n",data.altitude);
			}
			value = temp(1);
			printf(" Tempeture: %lf c = %5.2f f\n", value, (value * 1.8) + 32);
			printf("  Humitity: %lf %\n", temp(0));
			value = pressure();
			printf("  Pressure: %lf hPa = % 4.2f inch of mercury\n", value, value * 0.02953);
			printf("Wind Speed: %lf mph\n Direction: %lf\n Rain Fall: %lf ipm\n",
				wind_speed(),
				rain_fall(),
				24.7
				);
			for (c=0; c<8; c++)
			{
				printf("  Device %d: %lf ohms\n",c , read_adc_dev(c));
			};
			break;
		case 's':
			break;
	}
	digitalWrite (LED, LOW) ;	 // LED Off
	printf("\n");

	return 0;
}
