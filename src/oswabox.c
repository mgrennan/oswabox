/*
//  oswabox.c
//  Mark Grennan - 2014-01-17
//
*/
#define Version 0.4

//#define DEBUG					// uncomment to do debuging

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <math.h>

#include <wiringPi.h>
#include "wiringPiSPI.h"
#include <gertboard.h>
#include <maxdetect.h>
#include "gps.h"
#include "bmp085.h"

//
// Function decrelations
//
extern int readCalibrationTable(int,BMP085 *);
extern void printCalibrationTable(BMP085 *);
extern void makeMeasurement(int, BMP085 *);

void print_usage(const char *);
void parse_opts(int argc, char *argv[]);
void WindInterrupt(void);
void RainInterrupt(void);
float wind_speed(void);
float wind_direction(void);
float rain_fall(void);
float temp(int);
float pressure(int);
long readadc(int);
float read_adc_dev(int);

//
// GPIO pin decrelations
//
#define LED 3					// The wiringPi pin for the LED
#define RHT03_PIN  7			 	// The wiringPi pin for the RHT03
#define WIND_PIN 0				// The wiringPi pin for the wind speed
#define RAIN_PIN 1				// The wiringPi pin for the rain guage

//
// Program definations
//
#define wind_avg 1				// Number of seconds to average for wind speed
#define rain_avg 1
#define samples 50				// Number of AD samples to take
#define wait 10					// Time delay between ADC reads

//
// Program variables
//
int GPSflag = 0;			// use GPS
int Bannerflag = 1;			// Print program Banner
char output[20] = "";			// h=HUMAN, c=CSV, s=SQL
char station[20] = "";			// Name the Station
char table[64] = "";			// Table name used in SQL

int main (int argc, char **argv)
{
	int c;
	float value;

	loc_t data ;				 // Create GPS structure

	opterr = 0;

	parse_opts(argc,argv);  

	if (output[0] == 0)			// make the defalt ouput type Human readable
		strcpy(output, "human");

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
		fprintf (stderr, "Unable to setup wiringPi.\n");
		return 1;
	}

	// set Pin 17/0 generate an interrupt on high-to-low transitions
	if ( wiringPiISR (WIND_PIN, INT_EDGE_FALLING, &WindInterrupt) < 0 )
	{
		fprintf (stderr, "Unable to setup wind interrupts\n");
		return 1;
	}

	//
	// Start reading data
	//
	digitalWrite (LED, HIGH) ;	 // Turn LED On

	if ( GPSflag )
	{
		gps_location(&data);	 // Read and parse the GPS info
	} else {

		// Read data time from system
	}

	switch((char)output[0])
	{

		//
		// Output the current Observation as a CSV text line.
		//
		case 'c':
			if (GPSflag)
			{
				printf("%s, 20%02d-%02d-%02dT", station, data.year, data.month, data.day);
				printf("%02d:%02d:%02d,",data.hour, data.minute, data.second);
				printf("%lf,",data.latitude);
				printf("%lf,",data.longitude);
				printf("%lf,",data.altitude);
			}
			printf("%6.2f,%6.2f,%6.2f,%6.2f,%6.2f",
				temp(1), temp(0),pressure(0),wind_speed(),rain_fall() );
			for (c=0; c<8; c++)
			{
				printf(",%6.4f", read_adc_dev(c));
			}
			break;

		//
		// Output the current observations in a human readable format.
		//
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
			printf("  Humitity: %lf %%\n", temp(0));
			value = pressure(0);
			printf("  Pressure: %lf hPa = % 4.2f inch of mercury\n", value, value * 0.02953);
			printf("Wind Speed: %lf mph\n Direction: %lf\n Rain Fall: %lf ipm\n",
				wind_speed(),
				wind_direction(),
				rain_fall()
				);
			for (c=0; c<8; c++)
			{
				printf("  Device %d: %lf ohms\n",c , read_adc_dev(c));
			};
			break;

		//
		// Ouput a text line with an SQL insert statment
		//
		case 's':
			printf("INSERT INTO %s ( Station, Datetime, Temperature, Pressure, ", table);
			printf("Relative_Humidity, Wind_Speed, Wind_Direction, Rain, ");
			printf("Device_1, Device_2, Device_3, Device_4, ");
			printf("Device_5, Device_6, Device_7, Device_8) values (");

			printf("\"%s\",", station);
			printf("\"20%02d-%02d-%02d ", data.year, data.month, data.day);
			printf("%02d:%02d:%02d\"", data.hour, data.minute, data.second);
			printf(",%6.4f", temp(1));		// Temperature
			printf(",%6.4f", pressure(0));		// Pressure
			printf(",%6.4f", temp(0));		// Relative_Humidity
			printf(",%6.4f", wind_speed()); 	// Wind Speed
			printf(",%6.4f", wind_direction());	// Wind Direction
			printf(",%6.4f", rain_fall());		// Rain Fall
			for (c=0; c<8; c++)
			{
				printf(",%6.4f", read_adc_dev(c));
			}

			printf(");");
			break;
	}
	digitalWrite (LED, LOW) ;	 // LED Off
	printf("\n");

	return 0;
}


void print_usage(const char *prog)
{

//	printf("Open Source Weather and Air quality Box (OSWABox)\n");
//	printf(" - Version %4.2f\n\n",Version);
        printf("Usage: %s [-BGost]\n", prog);
	puts( "  -b --Banner  - Turn off the banner\n"
              "  -g --GPS     - Turn on the GPS\n"
              "  -s --station - Station name\n"
              "  -t --table   - Table name used for SQL\n"
              "  -o --output  - [chq] output type\n"
              "      c - CSV output\n"
              "      h - Human readable output\n"
              "      s - SQL output\n");

//             "  -m --mode\t\t sets the measurement mode. Default value 1 = STANDARD. Allowed values:\n"
//             "                                            0 = ULTRA LOW POWER\n"
//             "                                            1 = STANDARD\n"
//             "                                            2 = HIGH RESOLUTION\n"
//             "                                            3 = ULTRA HIGH RESOLUTION\n");

        exit(1);
}


void parse_opts(int argc, char *argv[])
{
        while (1) {
                static const struct option lopts[] = {
                        { "Banner", no_argument, NULL, 'b' },
                        { "GPS",no_argument, NULL, 'g' },
                        { "ouput", required_argument, NULL, 'o' },
                        { "station", required_argument, NULL, 's' },
                        { "table", required_argument, NULL, 't' },
                        { NULL, 0, 0, 0 },
                };
                int c;

                c = getopt_long(argc, argv, "bgo:s:t:", lopts, NULL);

                if (c == -1)
                        break;

		switch (c)
                {
                        case 'b':
                                Bannerflag = 0;
                                break;
                        case 'g':
                                GPSflag = 1;
                                break;
                        case 'o':
                                optarg[20] = 0;         // Limit the length of the input
                                strcpy(output, optarg);
                                break;
                        case 's':
                                optarg[20] = 0;         // Limit the length of the input
                                strcpy(station, optarg);
                                break;
                        case 't':
                                optarg[64] = 0;         // Limit the length of the input
                                strcpy(table, optarg);
                                break;
                        default:
                                print_usage(argv[0]);
                                break;
                }

        }
}


//
// the event counter
//
volatile int WindCounter = 0;
volatile int RainCounter = 0;


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

// Return the current wind speed in mph
float wind_speed(void)
{
        WindCounter = 0;
        delay( wind_avg * 1000 );        // Take an average

        /* Wind is 1.492 mph per event */
        #ifdef DEBUG
        printf( "%4.2f\n", (WindCounter * 1.492) / wind_avg);
        #endif

        return ((WindCounter * 1.492) / wind_avg);

}

//
// Return the current wind direction in degrees
//
float wind_direction(void)
{
        return 0.0;
}

//
// Return the rain fall in inches for the last minute
//
float rain_fall(void)
{
        RainCounter = 0;

        delay( rain_avg * 1000 );        // Take an average

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
// Program variables
//
char i2cAddress = 0x77;
char *device = "/dev/i2c-0";
int mode = 1;

//
// Return air pressure in hPa
//
float pressure(int temp)
{
        int fileDescriptor;

        BMP085 *sensor;
        sensor = (BMP085 *) malloc(sizeof(BMP085));
        
        sensor->i2cAddress = i2cAddress;
        sensor->oss = 3;

        fileDescriptor = open(device, O_RDWR);
        if (fileDescriptor<0)
        {
                printf("Failed to open i2c device!\n");
                exit(1);
        }

        if (ioctl(fileDescriptor, I2C_SLAVE, sensor->i2cAddress)<0)
        {
                printf("Failed to select i2c device!\n");
                exit(1);
        }

        
        if ( ! readCalibrationTable(fileDescriptor,sensor))
        {
                printf("Failed to read calibration table!\n");
                exit(1);
        }

	makeMeasurement(fileDescriptor,sensor);

        free(sensor);

        close(fileDescriptor);

	if( temp )
		return sensor->temperature;
	else
		return sensor->pressure/100.0;
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
long readadc(int adcnum)
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

/*
// Read the Analog to Digital Converter (ADC) and return the current
//   resistence value. The value is calulated on the if the device is
//   in a pullup or pull down circut, the input reference voltage and
//   resistence devider value in ohms.
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
                int pullup ;                     // 1 = device pulls up the Pin
                float refvolt ;                  // reverence voltage
                float resistance ;               // resistence in ohms
                char *name ;                     // device name
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
                return -1 ;                              // the mcp3008 wants clock speed between 1.35 and 3.6Mz

        ob = 0;
        tot = 0;
        for (i=0; i<samples; i++)        // Read samples
        {
                ob =  readadc(pin);
                tot = tot + (ob * 1.0) ;
                delay( wait ) ;
        }
        avg = tot / samples ;            // calculate the average of the readings

        // See http://en.wikipedia.org/wiki/Voltage_divider

                                                                 // calculate the average voltage
        volt = (dev[pin].refvolt / 1023.0) * avg ;
        // reference voltage / 10bit AD (1023) * reading

        if ( dev[pin].pullup )           // If this is a pullup resister
        {
                value = (( dev[pin].resistance * dev[pin].refvolt ) / volt ) - dev[pin].resistance ;
        }
        else                                             // this is a pulldown resister
        {
                value = dev[pin].resistance / (( dev[pin].refvolt / volt ) - 1 ) ;
        }

        #ifdef DEBUG                             /* display the average value */
        printf("Pin %d:%s  %09.2f = %06.4f volts = %010.2f Ohms\n", pin, dev[pin].name, avg, volt, value ) ;
        #endif                                           /* DEBUG */

        return value ;
}


