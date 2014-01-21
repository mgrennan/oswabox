/*
//  airqual.c
//  Mark Grennan - 2014-01-14
//
//  Using the MCP3008 AD converter (ADC) read the Air Quality
//  insterment and print the values.
//
//  based on isr.c from the WiringPi library, authored by Gordon Henderson
//  https://github.com/WiringPi/WiringPi/blob/master/examples/isr.c
//
//  Compile as follows:
//
//      gcc -o airqual airqual.c -lwiringPi
//
//  Run as follows:
//
//      sudo ./airqual
*/
#include <stdio.h>
#include <stdint.h>
#include <wiringPi.h>
#include "wiringPiSPI.h"
#include <gertboard.h>

#define samples 50				 // Number of AD samples to take
#define wait 10					 // Time delay between ADC reads

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

	buff[1] += adcnum << 4 ;

	wiringPiSPIDataRW(0, buff, 3);

	//	adc = ((buff[1] & 3) << 8) + buff[2];
								 // 10 bits of data
	adc = ((buff[1] * 256 ) + buff[2]) & 0b1111111111 ;

	return adc;
}


int main(int argc, char *argv[])
{
	int i, j ;
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
	pin[8] =
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

	printf ("Read Air Quarilty\n") ; 		 // Let's get started

	if (wiringPiSPISetup (0, 1500000) < 0) 		 // initialize the WiringPi API channel and speed
		return -1 ;				 // the mcp3008 wants clock speed between 1.35 and 3.6Mz

	j = 0;						 // Start for pin 0

	for ( j=0; j<8; j++) 
	{

		ob = 0;
		tot = 0;
		for (i=0; i<samples; i++)// Read samples
		{
			ob =  readadc(j);
			tot = tot + (ob * 1.0) ;
			delay( wait ) ;
		}
		avg = tot / samples ;	 		 // calculate the average of the readings

		// See http://en.wikipedia.org/wiki/Voltage_divider

		volt = (pin[j].refvolt / 1023.0) * avg ; // calculate the average voltage
		// reference voltage / 10bit AD (1023) * reading

		if ( pin[j].pullup )	 		 // If this is a pullup resister
		{
			value = (( pin[j].resistance * pin[j].refvolt ) / volt ) - pin[j].resistance ;
		}
		else					 // this is a pulldown resister
		{
			value = pin[j].resistance / (( pin[j].refvolt / volt ) - 1 ) ;
		}

		// display the average value
		printf("Pin %d:%s  %09.2f = %06.4f volts = %010.2f Ohms\n", j, pin[j].name, avg, volt, value ) ;

	}

	return 0 ;
}
