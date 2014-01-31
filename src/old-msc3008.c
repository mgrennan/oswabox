/*
//  mcp3008.c
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
//      gcc -o mcp3008 mcp3008.c -lwiringPi
//
//  Run as follows:
//
//      mcp3008 1
//
//  This will read the analog input of ping 2.
//
//  You may Optionaly input the A/D converter you want
//  to read as 0 through 7.
//
//  Read the MCP3008 chip throughe the SPI port on the
//  RaspberryPi.
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <wiringPi.h>
#include "wiringPiSPI.h"
#include <gertboard.h>

// read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)

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

								 // 10 bits of data
	adc = ((buff[1] * 256 ) + buff[2]) & 0b1111111111 ;

	return adc;
}


/*
//
*/
int main(int argc, char *argv[])
{
	int i, chan;
	uint32_t x1, tot, avg ;
	float volt ;

	printf ("SPI test program\n") ;

	// initialize the WiringPi API
	if (wiringPiSPISetup (0, 1000000) < 0)
		return -1 ;

	// get the channel to read, default to 0
	if (argc>1)
		chan = atoi(argv[1]);
	else
		chan = 0;

	if ((chan > 7) || (chan < 0))
	{
		printf("Channels 0-7 only\n");
		exit( 1 ) ;
	}

	while (1)					 // run until killed with Ctrl-C
	{
		tot = 0;
		for (i=0; i<100; i++)	 // Read 100 samples
		{
			x1 = readadc(chan);	 // read data and add to total
			tot += x1;
			delay(10);
		}

		avg = tot / 100 ;

		// Reference voltage 3.3 volts
		volt = avg * 0.003223 ;	 // result * ( Ref voltage / 1024 )

		// Reference voltage 5.0 volts
		//  volt = avg * 0.004882 ; // result * ( Ref voltage / 1024 )

		// display the average value
		printf("channal %d:  % 4d = %4.2f volts \n", chan, avg, volt) ;

	}

	return 0 ;
}
