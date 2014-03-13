/*
// oswaboxd - Open Source Weather and Air quality daemon
//    Copyright: Mark Grennan - 2014/03/12
//
//    This program run in the background as a daemon and collect weather
// and air quality information from the OSWABox hardware on connected
// to a RaspberryPi computer.
//
//    For more information on the OSWABox hardware and software goto
// http://www.oswabox.com/
//
//    LICENSE GPL Version 2.1
*/

// TODO:
//       Fix the average Wind Direction
//       Create function to turn AtoD reading into degrees +- ture north
//       Create array to calculate hourly moving averages, wind, rain...
//		http://server.gladstonefamily.net/pipermail/wxqc/2006-July/004319.html
//       Create network API to request fetch current condations in JSON format.
//       Create output to The Citizen Weather Observer Program (CWOP).

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <getopt.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <gps.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>

#include <wiringPi.h>
#include "wiringPiSPI.h"
#include <gertboard.h>
#include "bmp085.h"

#include "version.h"

extern int readCalibrationTable(int,BMP085 *);
extern void makeMeasurement(int, BMP085 *);

void print_usage(const char *);
void parse_opts(int argc, char *argv[]);
void become_daemon() ;
void RainInterrupt(void);
void WindInterrupt(void);
float pressure(int);
float read_adc_dev(int);
long readadc(int);
uint8_t sizecvt(int);
int read_dht22_dat();
float windAverage(void);
void signal_handler(int);
float ohms2lux(float);

//
// GPIO pin decrelations
//
#define LED_PIN  3                                          // The wiringPi pin for the LED
#define DHT_PIN  7                                          // The wiringPi pin for the RHT03
#define WIND_PIN 0                                          // The wiringPi pin for the wind speed
#define RAIN_PIN 1                                          // The wiringPi pin for the rain guage
#define TGS2600 0					    // ADA pins
#define MiSC2710 1
#define MiSC5525 2
#define Sound 3
#define WindRaw 4

//
// Global program variables
//
char GPShost[20] = "127.0.0.1";                             // Host IP for GPSd data
char GPSport[20] = "2947";                                  // Host PORT number for GPSd data
char *BMP085_device = "/dev/i2c-0";                         // Linux device for I2C buss
char BMP085_i2cAddress = 0x77;                              // Device number on I2C buss for BMP085
char *NamedPipe = "/tmp/OSWABoxPipe";                       // Named Pipe for CSV output
char *CSVFile = "/tmp/OSWABoxData";                         // Named Pipe for CSV output
char *WeeFile = "/tmp/oswadata";

int KeepAlive = 1;                                          // Loop while true - Signal can change this
unsigned long RainCount = 0;                                // Interupt counter for tipping bucket 0.011 inches per event
unsigned long WindCount = 0;                                // Interupt counter for wind speed s 1.492 mph per event
int GPSflag = 0;                                            // collect GPS data
int debugFlag = 0;                                          // Output debug inforamtion to syslog daemon file
int humidityFlag = 0;
int CollectionPeriod = 5;                                   // delay between observation collections in seconds
int ReportPeriod = 60;                                      // report results every X collectionPeriods 5*20=300 (5 min)
int BMP085_mode = 2;                                        // 0=LOW POWER 1=STANDARD 2=HIGH RESOLUTION 3=ULTRA HIGH RESOLUTION
int ADSamples = 10;                                         // number of read samples to average on the AD converter
int NPFlag = 0;                                             // write obs to named pipe
int CSVFlag = 0;                                            // write obs to file
int WeeFlag = 0;                                            // write obs to file read by WeeWx 
int pressureSet = 0;
struct gps_data_t gpsdata;
float dht_tempature;
float dht_humidity;
float seaLevelPressure = 1000.0;
uint8_t dht22_dat[5] = {0,0,0,0,0};
float WindDir[100];                                         // Array use to average wind direction for on period

struct statsRecord {                                        // Structure use to write statistics record
    time_t eventTime;
    float tempHigh;
    float tempLow;
    float highWind;                                         // High wind speed (gust) for reporting period
    float highWindDir;                                      // Direction of gust
    float hourlyPrecip;
    float dailyPrecip;
    float CurrentPressure;
} stats;

//
// Let the program begin
//
int main(int argc, char **argv)
{
    time_t currTime;                                        // System time in unix EPOC
    struct tm *localTime;
    FILE *fileHandle;                                       // Filesystem handle for open files
    char printBuffer[80];                                   // Array buffers
    int ReportLoop,ADLoop,DONE;
    float pressureAdjust = 0.0;                             // Difference between reading and standard 

    char CurrentTime[25]  = "";                             // The current time UTC

    float CurrentLatitude = 0;                              // The station's location
    float CurrentLongitude = 0;
    float CurrentAltitude = 0;

    float CurrentTemperature = 0;                           // Current temperature in Centragrade
    float CurrentHumidity = 0;                              // Current humidity

    float CurrentPressure = 0;                              // Current Air Pressure

    float WindAccumulation = 0;                             // Wind speed
    float WindDirection = 0;                                // Current wind direction
    float WindDirectionAccumulation = 0;                    // Wind direction average in degrees

    float RainAccumulation = 0;                             // Current rainfall accumulation

    float ADAccumulation[8];                                // Current values of AD convert 0-7

    parse_opts(argc,argv);                                  // check for command line arguments

    //
    // Initilize hardware and program settings
    //
    if (wiringPiSetup () < 0) {                             // Setup all Raspberry Pi Pins
        fprintf (stderr, "Unable to setup wiringPi.\n");
        exit(EXIT_FAILURE);
    }
    pinMode (LED_PIN, OUTPUT) ;                             // Set LED pin as an output
    digitalWrite (LED_PIN, LOW) ;                           // LED Off
    if ( wiringPiISR (RAIN_PIN, INT_EDGE_FALLING, &RainInterrupt) < 0 ) {
        fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        exit(EXIT_FAILURE);
    }
    if ( wiringPiISR (WIND_PIN, INT_EDGE_FALLING, &WindInterrupt) < 0 ) {
        fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        exit(EXIT_FAILURE);
    }
    CurrentTemperature = -9999.0;                            // Set imposible numbers to start
    CurrentHumidity = -9999.0;
    CurrentPressure = -9999.0;
    stats.tempHigh = -9999.0;
    stats.tempLow = 9999.0;
    stats.highWind = -1.0;
    stats.highWindDir = -1.0;
    stats.hourlyPrecip = -1.0;
    stats.dailyPrecip = -1.0;

    become_daemon();                                        // Disconnect from user and stay resident
    syslog (LOG_NOTICE, "Starting weather collection.");

    //
    // Read Daily Highs and Lows from the stats file
    //
    fileHandle = fopen("/tmp/oswaboxStats", "rb");          // Read stats data to file if exists
    if (fileHandle != NULL) {
        if (debugFlag > 1) {
            syslog (LOG_NOTICE, "DEBUG: opening /tmp/oswaboxStats");
        }
        fread(&stats, sizeof stats, 1, fileHandle);
        if (debugFlag > 2) {
            sprintf(printBuffer, "DEBUG: stats:time = %u", (unsigned int)stats.eventTime);
            syslog(LOG_NOTICE, printBuffer);
            sprintf(printBuffer, "DEBUG: stats:tempHigh = %f", stats.tempHigh);
            syslog(LOG_NOTICE, printBuffer);
            sprintf(printBuffer, "DEBUG: stats:tempLow = %f", stats.tempLow);
            syslog(LOG_NOTICE, printBuffer);
            sprintf(printBuffer, "DEBUG: stats:highWind = %f", stats.highWind);
            syslog(LOG_NOTICE, printBuffer);
            sprintf(printBuffer, "DEBUG: stats:highWindDir = %f", stats.highWindDir);
            syslog(LOG_NOTICE, printBuffer);
            sprintf(printBuffer, "DEBUG: stats:hourlyPrecip = %f", stats.hourlyPrecip);
            syslog(LOG_NOTICE, printBuffer);
            sprintf(printBuffer, "DEBUG: stats:dailyPrecip = %f", stats.dailyPrecip);
            syslog(LOG_NOTICE, printBuffer);
            sprintf(printBuffer, "DEBUG: stats:CurrentPressure = %f", stats.CurrentPressure);
            syslog(LOG_NOTICE, printBuffer);
        }
        fclose(fileHandle);
    }

    if ( time(NULL) >= stats.eventTime + 3600 ) {           // Reset stats if older than one hour
        stats.highWind = -1.0;
        stats.highWindDir = -1.0;
        stats.hourlyPrecip = -1.0;
    } else {
        pressureSet = 1;                                    // reset the pressure offset based on the last reading
    }
    if ( time(NULL) >= stats.eventTime + 90000 ) {          // Reset stats if older than one day
        stats.dailyPrecip = -1.0;
    }

//
// Main / Outer Loop
//
// Make reading every CollectionPeriod - Average these reading over the ReportPeriod
//
    while (KeepAlive) {
        for (ReportLoop=0; ReportLoop<ReportPeriod; ReportLoop++) {
            if ( ! KeepAlive )                              // Check if we should bugger off
                break;
            if (debugFlag > 1) {
                sprintf(printBuffer, "DEBUG: Collecting Observations %d", ReportLoop+1);
                syslog(LOG_NOTICE, printBuffer);
            }

            digitalWrite (LED_PIN, HIGH) ;                  // LED On

            //
            // Temperature & Humidity
            //
            if(humidityFlag) {				    // if -H set read temp from pressure censor
                CurrentTemperature = pressure(1);           // Read Temperature
            } else {
                if (debugFlag > 2) {
                    syslog (LOG_NOTICE, "DEBUG: Reading DHT");
                }
                if ( read_dht22_dat() ) {
                    CurrentTemperature = dht_tempature;     // Read Temp from Humidity censor
                    CurrentHumidity = dht_humidity;         // Read Humidity
                }
            }
                stats.tempLow = CurrentTemperature;

            //
            // Air Pressure
            //
            if ( pressureSet ) {                            // local/sea level pressure has been set
                CurrentPressure = seaLevelPressure;
                pressureAdjust = pressure(0) - seaLevelPressure;
                pressureSet = 0;
            } else {                                        // read air and adjust it
                CurrentPressure = pressure(0) - pressureAdjust ;             // Read Air Pressure
            }
            stats.CurrentPressure = CurrentPressure;        // save the current pressure in the stats record
            //
            // Wind
            //
            WindDirectionAccumulation = -95.0;              // wind direction +- degrees true north
            WindDir[ReportLoop] = WindDirectionAccumulation;
            WindDirection = windAverage();
            if (debugFlag > 2) {
                sprintf(printBuffer, "DEBUG: Wind Direction reading #%d=%6.2f  Average=%6.2f", ReportLoop+1,WindDirectionAccumulation,WindDirection);
                syslog(LOG_NOTICE, printBuffer);
            }

            WindAccumulation = WindCount * 1.492 ;          // Calculate the current wind speed
            WindCount = 0;                                  // Zero the Interupt counter
            if ( WindAccumulation > stats.highWind ) {      // Capture the daily high wind speed
                stats.highWind = WindAccumulation ;
                stats.highWindDir = WindDirectionAccumulation ;
            }

            //
            // Precipition
            //
            RainAccumulation += RainCount * 0.011 ;         // Calculate the current rainfall
            RainCount = 0;                                  // Zero the Interupt counter
            stats.hourlyPrecip += RainAccumulation;         // Accumulate the hourly rain total
            stats.dailyPrecip += RainAccumulation;          // Accumulate the daily rain total

            digitalWrite (LED_PIN, LOW) ;                   // LED Off

            //
            // Time
            //
            currTime = time(NULL);                      // Current Time from System
            localTime = localtime(&currTime);
            sprintf(CurrentTime,"20%02d:%02d:%02dT%02d:%02d:%02d.00Z",
                localTime->tm_year-100, localTime->tm_mon+1, localTime->tm_mday,
                localTime->tm_hour, localTime->tm_min, localTime->tm_sec);

            //
            // Air Quality
            //
            for (ADLoop=0; ADLoop<8; ADLoop++) {        // Read all the AD values
                ADAccumulation[ADLoop] = read_adc_dev(ADLoop);
            }
/*
 * Output for the WeeWx weather station software - See below
*/
            if ( WeeFlag ) {                                // Write data for the WeeWx web service

                fileHandle = fopen(WeeFile,"w");

                if (fileHandle == NULL) {
                    syslog (LOG_NOTICE, "ERROR: Failed to open output file.\n");
                    exit(EXIT_FAILURE);
                } else {
                    // These are keyword used by WeeWx for weather guage/station data
                    sprintf(printBuffer,"dateTime %lu\noutTemp %4.2f\nbarometer %4.2f\nwindSpeed %4.2f\n",
                       time(NULL), (9.0/5.0)*CurrentTemperature+32.0, CurrentPressure / 33.86, WindAccumulation );
                    fwrite(printBuffer, strlen(printBuffer),1,fileHandle);
                    sprintf(printBuffer,"windDir %4.2f\noutHumidity %4.2f\nrain %4.2f\nUV %10.2f\n",
                       WindDirection, CurrentHumidity, RainAccumulation, (readadc(TGS2600)*1.0)/64);
                    fwrite(printBuffer, strlen(printBuffer),1,fileHandle);
                    sprintf(printBuffer,"dateTime %lu\noutTemp %4.2f\nbarometer %4.2f\nwindSpeed %4.2f\n",
                       time(NULL), (9.0/5.0)*CurrentTemperature+32.0, CurrentPressure / 33.86, WindAccumulation );
                    fwrite(printBuffer, strlen(printBuffer),1,fileHandle);
                    sprintf(printBuffer,"windDir %4.2f\noutHumidity %4.2f\nrain %4.2f\nUV %10.2f\n",
                       WindDirection, CurrentHumidity, RainAccumulation, (readadc(TGS2600)*1.0)/64);
                    fwrite(printBuffer, strlen(printBuffer),1,fileHandle);
                    fclose(fileHandle);
                }
            }

//
// This Data is read only once per reporting period
//
            if ( ReportLoop+1 == ReportPeriod ) {           // Report Opbservations
                //
                // These readings only need to be collected once per reporting period
                //
                digitalWrite (LED_PIN, HIGH) ;              // LED On

                //
                // GPS
                //
                if (GPSflag) {                              // Get GPS information
                    if (gps_open(GPShost, GPSport, &gpsdata) != 0) {
                        syslog(LOG_NOTICE, "ERROR: connecting to gpsd");
                    } else {
                        gps_stream(&gpsdata, WATCH_ENABLE, NULL);
                        DONE = 0;
                        while( ! DONE) {
                            if ( !gps_waiting(&gpsdata, 500000) ) {
                                syslog(LOG_NOTICE,"ERROR: GPS Timmed out");
                                DONE = 1;
                            } else {
                                if (debugFlag > 2 )
                                    syslog(LOG_NOTICE, "Reading GPS information");
                                gps_read(&gpsdata);
                                if( gpsdata.fix.mode > STATUS_NO_FIX ) {
                                    unix_to_iso8601(gpsdata.fix.time, CurrentTime, sizeof(CurrentTime));
                                    CurrentLatitude = gpsdata.fix.latitude;
                                    CurrentLongitude = gpsdata.fix.longitude;
                                    CurrentAltitude = gpsdata.fix.altitude;
                                    DONE = 1;
                                }
                            }
                        }
                        gps_close(&gpsdata);
                    }
                }

                digitalWrite (LED_PIN, LOW) ;               // LED Off

                // Opbservation Collection is DONE

                if (debugFlag) {
                    int ReportMinutes = CollectionPeriod * ReportPeriod / 60;
                    int ReportSeconds = CollectionPeriod * ReportPeriod % 60;
                    sprintf(printBuffer, "DEBUG: Current time %s", CurrentTime);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Current Latitude %6.4f", CurrentLatitude);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Current Longitude %6.4f", CurrentLongitude);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Reporting Period (RP) %2d:%02d", ReportMinutes, ReportSeconds);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Current Altitude %6.2f", CurrentAltitude);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Current Temperature = %6.2f", CurrentTemperature);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Daily High Temperature = %6.2f", stats.tempHigh);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: RP Precipition = %6.2f", RainAccumulation);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Hourly Precipition = %6.2f", stats.hourlyPrecip);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Daily Precipition= %6.2f", stats.dailyPrecip);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Current Windspeed = %6.2f", WindAccumulation);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Current Wind Direction = %6.2f", WindDirection);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: RP High Wind speed = %6.2f", stats.highWind);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: RP High Wind Direction = %6.2f", stats.highWindDir);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: LUX Level = %6.2f", ohms2lux(ADAccumulation[0]));
                    syslog(LOG_NOTICE, printBuffer);
                    for (ADLoop=0; ADLoop<8; ADLoop++) {
                        sprintf(printBuffer, "DEBUG: AD %d = %6.2f", ADLoop, ADAccumulation[ADLoop]);
                        syslog(LOG_NOTICE, printBuffer);
                    }
                }
/*
 * Output for the WeeWx weather station software
*/
                if ( WeeFlag ) {                                // Write data for the WeeWx web service

                    fileHandle = fopen(WeeFile,"w");

                    if (fileHandle == NULL) {
                        syslog (LOG_NOTICE, "ERROR: Failed to open output file.\n");
                        exit(EXIT_FAILURE);
                    } else {
                        // These are keyword used by WeeWx for weather guage/station data
                        sprintf(printBuffer,"dateTime %lu\noutTemp %4.2f\nbarometer %4.2f\nwindSpeed %4.2f\n",
                            time(NULL), (9.0/5.0)*CurrentTemperature+32.0, CurrentPressure / 33.86, WindAccumulation);
                        fwrite(printBuffer, strlen(printBuffer),1,fileHandle);
                        sprintf(printBuffer,"windDir %4.2f\noutHumidity %4.2f\nrain %4.2f\naltitude %4.2f\nUV %10.2f\n",
                            WindDirection, CurrentHumidity, RainAccumulation, CurrentAltitude, (readadc(TGS2600)*1.0)/64);
                        fwrite(printBuffer, strlen(printBuffer),1,fileHandle);
                        fclose(fileHandle);
                    }
                }

                //
                // Write the observations to the named pipe or CSV file
                //
                if (NPFlag || CSVFlag) {
                    if ( NPFlag ) 
                        fileHandle = fopen(NamedPipe,"rw");

                    if ( CSVFlag )
                        fileHandle = fopen(CSVFile,"a+");

                    if (fileHandle == NULL) {
                        syslog (LOG_NOTICE, "ERROR: Failed to open output file.\n");
                        exit(EXIT_FAILURE);
                    } else {
/*
 * CSV output
*/
                        sprintf(printBuffer,"%s,%4.4f,%4.4f,%6.4f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f",
                            CurrentTime,CurrentLatitude,CurrentLongitude,CurrentAltitude,
                            CurrentTemperature,CurrentHumidity,CurrentPressure,WindAccumulation,WindDirection);
                        fwrite(printBuffer, strlen(printBuffer),1,fileHandle);

                        sprintf(printBuffer,",%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f\n",
                            stats.tempHigh,stats.tempLow,stats.highWind,stats.highWindDir,
                            ohms2lux(ADAccumulation[0]),ADAccumulation[1],ADAccumulation[2],ADAccumulation[3],
                            ADAccumulation[4],ADAccumulation[5],ADAccumulation[6],ADAccumulation[7]);
                        fwrite(printBuffer, strlen(printBuffer),1,fileHandle);

                        fclose(fileHandle);
                    }
                }
            }

            //
            // Save Current Stats
            //
            fileHandle = fopen("/tmp/oswaboxStats", "w+b"); // Write stats data to file
            if (fileHandle == NULL) {
                syslog (LOG_NOTICE, "ERROR: Failed to open /tmp/oswaboxStats\n");
                exit(EXIT_FAILURE);
            }
            fwrite(&stats, sizeof stats, 1, fileHandle);
            fclose(fileHandle);

            sleep ( CollectionPeriod );                     // Sleep until we need to collect observations again
        }

        //
        // Zero accumlations for ths reporting period
        //
        WindDirectionAccumulation = 0.0;
        RainAccumulation = 0.0;

    }

    //
    // Close file and exit
    //
    syslog (LOG_NOTICE, "Terminated.");
    closelog();

    exit(EXIT_SUCCESS);
}


//
// Return air pressure in hPa
//
float pressure(int temp)
{
    int fileDescriptor = open(BMP085_device, O_RDWR);
    if (fileDescriptor < 0) {
        syslog (LOG_NOTICE, "ERROR: Failed to open i2c device!\n");
        return -9999.0;
    }

    BMP085 *sensor;
    sensor = (BMP085 *) malloc(sizeof(BMP085));

    sensor->i2cAddress = BMP085_i2cAddress;
    sensor->oss = BMP085_mode;

    if (ioctl(fileDescriptor, I2C_SLAVE, sensor->i2cAddress) < 0) {
        syslog (LOG_NOTICE, "ERROR: Failed to select BMP085 i2c device!\n");
        exit(EXIT_FAILURE);
    }

    if ( ! readCalibrationTable(fileDescriptor,sensor)) {
        syslog (LOG_NOTICE, "ERROR: Failed to read BMP085 calibration table!\n");
        exit(EXIT_FAILURE);
    }

    if ( debugFlag > 2)
        syslog (LOG_NOTICE, "DEBUG: Reading BMP085 pressure sensor");

    makeMeasurement(fileDescriptor,sensor);

    free(sensor);

    close(fileDescriptor);

    if( temp )
        return sensor->temperature;
    else
        return sensor->pressure/100.0;
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
        int pullup ;                                        // 1 = device pulls up the Pin
        float refvolt ;                                     // reverence voltage
        float resistance ;                                  // resistence in ohms
        char *name ;                                        // device name
    }
    dev[8] = {
        { 0, 3.3,  10000.0, "LDR light sensor" },
        { 0, 3.3,    990.0, "Test Voltage    " }
    } ;

    if (wiringPiSPISetup (0, 1500000) < 0)                  // initialize the WiringPi API channel and speed
        return -1 ;                                         // the mcp3008 wants clock speed between 1.35 and 3.6Mz

    ob = 0;
    tot = 0;
    for (i=0; i<ADSamples; i++) {                           // Read samples
        ob =  readadc(pin);
        tot = tot + (ob * 1.0) ;
        delay( 10 ) ;                                       // wait 10 miliseconds before reading again
    }
    avg = tot / ADSamples ;                                 // calculate the average of the readings

    // See http://en.wikipedia.org/wiki/Voltage_divider

    volt = (dev[pin].refvolt / 1023.0) * avg ;              // calculate the average voltage
    // reference voltage / 10bit AD (1023) * reading

    if ( dev[pin].pullup ) {                                // If this is a pullup resister
        value = (( dev[pin].resistance * dev[pin].refvolt ) / volt ) - dev[pin].resistance ;
    }
    else {                                                  // this is a pulldown resister
        value = dev[pin].resistance / (( dev[pin].refvolt / volt ) - 1 ) ;
    }

    return value ;
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

    if ( debugFlag > 2)
        syslog (LOG_NOTICE, "DEBUG: Reading MCP3008 AD value");

    wiringPiSPIDataRW(0, buff, 3);

    adc = ((buff[1] * 256 ) + buff[2]) & 0b1111111111 ;

    return adc;
}


uint8_t sizecvt(const int read)
{
    /* digitalRead() and friends from wiringpi are defined as returning a value
    < 256. However, they are returned as int() types. This is a safety function */

    if (read > 255 || read < 0) {
        printf("Invalid data from wiringPi library\n");
        exit(EXIT_FAILURE);
    }
    return (uint8_t)read;
}


int read_dht22_dat()
{
    uint8_t laststate = HIGH;
    uint8_t counter = 0;
    uint8_t j , i;
    int tries = 1;
    char printBuffer[80];                                   // Array buffers

    while ( tries < 9 ) {                                   // try to read the device 8 times
        j = 0;
        dht22_dat[0] = dht22_dat[1] = dht22_dat[2] = dht22_dat[3] = dht22_dat[4] = 0;

        pinMode(DHT_PIN, OUTPUT);
        digitalWrite(DHT_PIN, HIGH);                            // start pin high for some time
        delayMicroseconds(40);
        digitalWrite(DHT_PIN, LOW);                             // pull pin down for 18 miliseconds
        delayMicroseconds(10);
        digitalWrite(DHT_PIN, HIGH);                            // then pull it up for 40 microseconds
        delayMicroseconds(30);

        pinMode(DHT_PIN, INPUT);                                // prepare to read the pin
        for ( i=0; i< 85; i++) {                                // detect change and read data
            counter = 0;
            while (sizecvt(digitalRead(DHT_PIN)) == laststate) {
                counter++;
                delayMicroseconds(1);
                if (counter == 255) {
                    break;
                }
            }
            laststate = sizecvt(digitalRead(DHT_PIN));

            if ((i >= 4) && (i%2 == 0)) {                  // ignore first 3 transitions
                dht22_dat[j/8] <<= 1;                      // move over the last bit set
                if (counter > 27)                          // if it took < 28us to see the transitition
                    dht22_dat[j/8] |= 1;		   // set the bit high
                j++;
            }
        }

        if (debugFlag > 2) {
            sprintf(printBuffer, "DEBUG: DHT try #%d returned %d:%d:%d:%d:%d", 
                tries,dht22_dat[0], dht22_dat[1], dht22_dat[2], dht22_dat[3], dht22_dat[4]);
            syslog (LOG_NOTICE, printBuffer);
        }

        // check we read 40 bits (8bit x 5 ) + verify checksum in the last byte
        if ((j >= 40) && (dht22_dat[4] == ((dht22_dat[0] + dht22_dat[1] + dht22_dat[2] + dht22_dat[3]) & 0xFF)) ) {

            dht_humidity = ((float)dht22_dat[0] * 256 + (float)dht22_dat[1]) / 10.0;
            dht_tempature = ((float)(dht22_dat[2] & 0x7F)* 256 + (float)dht22_dat[3]) / 10.0;

            if ((dht22_dat[2] & 0x80) != 0)  dht_tempature *= -1;

            return 1;
        }
        sleep(1);
        tries++;
    }
    return 0;
}


float windAverage(void)
{
    float WindDirection = 0;

    float totalX = 0.0;
    float totalY = 0.0;
    int count;

    for (count=0; count < ReportPeriod; count++ ) {
        totalX = (WindDir[count]*sin(WindDir[count])) + totalX;
        totalY = (WindDir[count]*cos(WindDir[count])) + totalY;

        if ( totalY == 0.0)
            WindDirection = 0.0;
        else
            WindDirection = atan(totalX/totalY);

        WindDirection = WindDirection / 0.01745311;         // 3.14156 / 180

        if (totalX*totalY < 0.0) {
            if (totalX < 0.0)
                WindDirection += 180.0;
            else
                WindDirection += 360.0;
        }
        else {
            if (totalX > 0.0) {
                WindDirection += 180.0;
            }
        }

    }
    return (WindDirection);
}


void parse_opts(int argc, char *argv[])
{
    while (1) {
        static const struct option lopts[] = {
            { "BMP085", required_argument, NULL, 'B' },
            { "csv", no_argument, NULL, 'c' },
            { "debug", required_argument, NULL, 'd' },
            { "gps",required_argument, NULL, 'g' },
            { "host",required_argument, NULL, 'h' },
            { "NoHmt",no_argument, NULL, 'H' },
            { "namedpipe", no_argument, NULL, 'n' },
            { "port", required_argument, NULL, 'P' },
            { "period", required_argument, NULL, 'p' },
            { "report", required_argument, NULL, 'r' },
            { "samples", required_argument, NULL, 's' },
            { "version", no_argument, NULL, 'v' },
            { "weewx", no_argument, NULL, 'w' },
            { "help", no_argument, NULL, '?' },
            { NULL, 0, 0, 0 },
        };
        int c;

        c = getopt_long(argc, argv, "B:cd:gh:HnP:p:r:s:S:vw?", lopts, NULL);

        if (c == -1)
            break;

        switch (c) {
            case 'B':
                BMP085_mode = atoi(optarg);
                break;
            case 'c':
                CSVFlag = 1;
                break;
            case 'd':
                debugFlag = atoi(optarg);
                break;
            case 'g':
                GPSflag = 1;
                break;
            case 'h':
                strcpy(GPShost, optarg);
                break;
            case 'H':
                humidityFlag = 1;
                break;
            case 'n':
                NPFlag = 1;
                break;
            case 'P':
                strcpy(GPSport, optarg);
                break;
            case 'p':
                CollectionPeriod = atoi(optarg);
                break;
            case 'r':
                ReportPeriod = atoi(optarg);
                break;
            case 's':
                ADSamples = atoi(optarg);
                break;
            case 'S':
                pressureSet = 1;
                seaLevelPressure = atof(optarg);
                break;
            case 'v':
                printf("OSWABox Version: %s\n",Version);
                printf("     Build Date: %s\n",BuildDate);
                exit(EXIT_FAILURE);
            case 'w':
		WeeFlag = 1;
                break;
            case '?':
                print_usage(argv[0]);
                break;
            default:
                print_usage(argv[0]);
                break;
        }

    }
}


void print_usage(const char *prog)
{
    printf("Open Source Weather and Air quality Box Daemon\n");
    printf("   Version: %s\n",Version);
    printf("Build Date: %s\n",BuildDate);
    printf("\nUsage: %s [-BdghHPprsTv?]\n", prog);
    puts(   "  -B --BMP085 - sets the pressure measurement mode.\n"
        "             0 = ULTRA LOW POWER\n"
        "             1 = STANDARD\n"
        "   Default = 2 = HIGH RESOLUTION\n"
        "             3 = ULTRA HIGH RESOLUTION\n"
        "  -c --csv        - write stats to CSV file in /tmp\n"
        "  -d --debug      - debug level 1=low 2=high 3=everything\n"
        "  -g --gps        - turn on the GPS\n"
        "  -h --host       - GPS host IP; Default 127.0.0.1\n"
        "  -H --NoHmt      - do not read the humidity and get temp from pressure censor\n"
        "  -? --help       - print this help message\n"
        "  -n --namedpipe  - write CSV data to named pipe /tmp/OSWABoxPipe\n"
        "  -P --port       - GPS port; Default 2947\n"
        "  -p --period     - number of seconds between observations; Default 20\n"
        "  -r --report     - number of observations before a report: Default 9\n"
        "  -s --samples    - number of samples to average the AD converter: Default 10\n"
        "  -S --sealevel   - local air pressure setting above sealevel\n"
        "  -v --version    - print the version information\n" 
        "  -w --weewx      - write data to /temp/oswadata for the WeeWx oswabox driver");
    exit(EXIT_FAILURE);
}


//
// Terminate and stay resident
//
void become_daemon(void)
{
    pid_t pid;

    pid = fork();                                           // Fork off the parent process

    if (pid < 0)                                            // An error occurred
        exit(EXIT_FAILURE);

    if (pid > 0)                                            // Success: Let the parent terminate
        exit(EXIT_SUCCESS);

    if (setsid() < 0)                                       // On success: The child process becomes session leader
        exit(EXIT_FAILURE);

    signal(SIGHUP, signal_handler);                         // Setup signal handling before we start
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
    signal(SIGQUIT, signal_handler);
    signal(SIGCHLD, SIG_IGN);                               // Catch, ignore and handle signals

    pid = fork();                                           // Fork off for the second time
    if (pid > 0)                                            // Success: Let the parent terminate
        exit(EXIT_SUCCESS);

    umask(0);                                               // Set new file permissions

    chdir("/");                                             // Change the working directory to the root directory
    // or another appropriated directory

    int x;                                                  // Close all open file descriptors
    for (x = sysconf(_SC_OPEN_MAX); x>0; x--) {
        close (x);
    }

    openlog ("oswaboxd", LOG_PID, LOG_DAEMON);
}


void signal_handler(int sig)
{

    switch(sig) {
        case SIGHUP:
            syslog(LOG_WARNING, "Received SIGHUP signal.");
            break;
        case SIGTERM:
            syslog(LOG_WARNING, "Received SIGTERM signal.");
            KeepAlive = 0;                                  // STOP, CLEANUP and DIE!
            break;
        default:
            syslog(LOG_WARNING, "Unhandled signal ");
            break;
    }
}

float ohms2lux(float ohms)
{
    return( 5e9 * pow(log10(ohms), -12.78) * sqrt(0.99) );
}

//
// Interrupt  called every time an event occurs
//
void RainInterrupt(void)
{
    WindCount++;
}


//
// Interrupt called every time an event occurs
//
void WindInterrupt(void)
{
    RainCount++;
}
