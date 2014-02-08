/*
//  windspeed.c
//  Mark Grennan - 2014-01-02
//
// based on isr.c from the WiringPi library, authored by Gordon Henderson
// https://github.com/WiringPi/WiringPi/blob/master/examples/isr.c
//
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>

#include <wiringPi.h>
#include "wiringPiSPI.h"
#include <gertboard.h>

// Use GPIO Pin 17 Wind Speed, which is Pin 0 in wiringPi
// Use GPIO Pin 18 Rain Guage, which is Pin 1 in wiringPi

#define WIND_PIN 0
#define WIND_DIR_PIN 5

long readadc(int);
float get_wind_direction(void);
void myInterrupt(void);

// the event counter
volatile int eventCounter = 0;


int main(void)
{
    // sets up the wiringPi library
    if (wiringPiSetup () < 0)
    {
        fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
        return 1;
    }

    // set Pin 17/0 generate an interrupt on high-to-low transitions
    // and attach myInterrupt() to the interrupt
    if ( wiringPiISR (WIND_PIN, INT_EDGE_FALLING, &myInterrupt) < 0 )
    {
        fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        return 1;
    }

    // display counter value every second.
    while ( 1 )
    {
        /* Wind is 1.492 mph per event */
        printf("     Wind Speed: % 4.2f\n", eventCounter * 1.492 );
        printf(" Wind Direction: % 4.2f\n", get_wind_direction() );
        eventCounter = 0;
        delay( 1000 );                                      // wait 1 second
    }

    return 0;
}

//
// read the wind direction sensor, return heading in degrees
//
float get_wind_direction() 
{
  unsigned int adc;

  adc = readadc(WIND_DIR_PIN); 			// get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order!  See Weather Meters datasheet for more information.

  if (adc < 380) return (112.5);
  if (adc < 393) return (67.5);
  if (adc < 414) return (90);
  if (adc < 456) return (157.5);
  if (adc < 508) return (135);
  if (adc < 551) return (202.5);
  if (adc < 615) return (180);
  if (adc < 680) return (22.5);
  if (adc < 746) return (45);
  if (adc < 801) return (247.5);
  if (adc < 833) return (225);
  if (adc < 878) return (337.5);
  if (adc < 913) return (0);
  if (adc < 940) return (292.5);
  if (adc < 967) return (315);
  if (adc < 990) return (270);
  return (-1); // error, disconnected?
}

/* This table is for 3.3V through a 10k resistor.

heading         resistance      volts           nominal         midpoint (<)
112.5		0.69	k	1.2	V	372	counts	380
67.5		0.89	k	1.26	V	389	counts	393
90		1	k	1.29	V	398	counts	414
157.5		1.41	k	1.39	V	430	counts	456
135		2.2	k	1.56	V	483	counts	508
202.5		3.14	k	1.72	V	534	counts	551
180		3.9	k	1.84	V	569	counts	615
22.5		6.57	k	2.13	V	661	counts	680
45		8.2	k	2.26	V	700	counts	746
247.5		14.12	k	2.55	V	792	counts	801
225		16	k	2.62	V	811	counts	833
337.5		21.88	k	2.76	V	855	counts	878
0		33	k	2.91	V	902	counts	913
292.5		42.12	k	2.98	V	925	counts	940
315		64.9	k	3.08	V	956	counts	967
270		98.6	k	3.15	V	978	counts	>967
*/

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

    //      adc = ((buff[1] & 3) << 8) + buff[2];
                                                            // 10 bits of data
    adc = ((buff[1] * 256 ) + buff[2]) & 0b1111111111 ;

    return adc;
}

// -------------------------------------------------------------------------
// myInterrupt:  called every time an event occurs
void myInterrupt(void)
{
    eventCounter++;
}


