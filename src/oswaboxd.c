/*
// oswaboxd - Open Source Weather and Air quality daemon
//
//   This program run in the background as a daemon and collect weather
// and air quality information from the OSWABox hardware on connected
// to a RaspberryPi computer.
//
*/

#define DEBUG                                               // uncomment to do debuging

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <getopt.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>

#include <wiringPi.h>
#include "wiringPiSPI.h"
#include <gertboard.h>
#include "bmp085.h"

extern int readCalibrationTable(int,BMP085 *);
extern void makeMeasurement(int, BMP085 *);

void print_usage(const char *);
void parse_opts(int argc, char *argv[]);
static void become_daemon() ;
void RainInterrupt(void);
void WindInterrupt(void);
float pressure(int);

//
// GPIO pin decrelations
//
#define LED_PIN  3                                          // The wiringPi pin for the LED
#define DHT_PIN  7                                          // The wiringPi pin for the RHT03
#define WIND_PIN 0                                          // The wiringPi pin for the wind speed
#define RAIN_PIN 1                                          // The wiringPi pin for the rain guage

//
// Application Definations
//
#define BMP085_Mode  2

//
// Global program variables
//
int GPSflag = 0;                                            // collect GPS data
int debugFlag = 0;                                          // Output debug inforamtion to syslog daemon file
char gpstype[20] = "";                                      // The GPS is s=serial device, d=gps daemon device
int CollectionPeriod = 20 ;                                 // Default delay between observation collections in seconds
int ReportPeriod = 9 ;                                      // Default report results every X collectionPeriods
int NPFlag = 0;                                             // write obs to named pipe
char GPShost[20] = "127.0.0.1";                             // Host IP for GPSd data
char GPSport[20] = "2947";                                  // Host PORT number for GPSd data
volatile unsigned long RainCount = 0;                       // Counter for tipping bucket 0.011 inches per event
volatile unsigned long WindCount = 0;                       // Counter for wind speed s 1.492 mph per event
char *BMP085_device = "/dev/i2c-0";                         // Linux device for I2C buss
char BMP085_i2cAddress = 0x77;                              // Device number on I2C buss for BMP085
char *NamedPipe = "/tmp/OSWABoxPipe";                       // Named Pipe for CSV output

int main(int argc, char **argv)
{
    time_t currTime;
    struct tm *localTime;
    char CurrentTime[20]  = "";
    int NamedPipeHandle;
    char printBuffer[80];
    int ReportLoop;

    float RainAccumulation = 0;                             // Current rainfall accumulation
    float WindAccumulation = 0;
    float WindDirection = 0;
    float CurrentPressure = 0;
    float CurrentTemperature = 0;

    float HourlyRainTotal = 0;                              // Total rainfall over the last hour

    float DailyTempHigh = -9999.0;
    float DailyTempLow = 9999.0;
    float DailyRainTotal = 0;                               // Totlal rainfall for today
    float DailyWindHigh = -1;

    parse_opts(argc,argv);                                  // check for command line arguments

    //
    // Initilise hardware and program settings
    //
    if (wiringPiSetup () < 0)                               // Setup all Raspberry Pi Pins
    {
        fprintf (stderr, "Unable to setup wiringPi.\n");
        return 1;
    }
    pinMode (LED_PIN, OUTPUT) ;                             // Set LED pin as an output
    digitalWrite (LED_PIN, LOW) ;                           // LED Off
    if ( wiringPiISR (RAIN_PIN, INT_EDGE_FALLING, &RainInterrupt) < 0 )
    {
        fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        return 1;
    }
    if ( wiringPiISR (WIND_PIN, INT_EDGE_FALLING, &WindInterrupt) < 0 )
    {
        fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        return 1;
    }

    become_daemon();

    //
    // Mail Loop
    //
    syslog (LOG_NOTICE, "Starting weather collection.");
    for (ReportLoop=0; ReportLoop<ReportPeriod; ReportLoop++)
    {

        digitalWrite (LED_PIN, HIGH) ;                      // LED On

        currTime = time(NULL);                              // Current Time - from System or GPS
        localTime = localtime(&currTime);
        sprintf(CurrentTime,"20%02d:%02d:%02dT%02d:%02d:%02d",
            localTime->tm_year-100, localTime->tm_mon+1, localTime->tm_mday,
            localTime->tm_hour, localTime->tm_min, localTime->tm_sec);

        CurrentPressure = pressure(0) ;                     // Read Air Pressure
        CurrentTemperature = pressure(1);                   // Read Air Temperature
        if ( CurrentTemperature > DailyTempHigh )           // Capture Daily High Temp
            DailyTempHigh = CurrentTemperature;
        if ( CurrentTemperature < DailyTempLow )            // Capture Daily Low Temp
            DailyTempLow = CurrentTemperature;

        RainAccumulation = RainCount * 0.011 ;              // Calculate the current rainfall
        HourlyRainTotal += RainAccumulation;                // Accumulate the hourly rain total
        DailyRainTotal += RainAccumulation;                 // Accumulate the daily rain total

                                                            // Calculate the current wind speed
        WindAccumulation = WindCount * 1.492 / CollectionPeriod;
        if ( WindAccumulation > DailyWindHigh )             // Capture the daily high wind speed
            DailyWindHigh = WindAccumulation ;
        WindDirection = 0.0;                                // Wind Direction

        digitalWrite (LED_PIN, LOW) ;                       // LED Off

        if ( ReportLoop == ReportPeriod )                   // Report Opbservations
        {
            if (debugFlag)
            {
                sprintf(printBuffer, "DEBUG: Current time %s", CurrentTime);
                syslog(LOG_NOTICE, printBuffer);
                sprintf(printBuffer, "DEBUG: Current Temperature = %6.2f", CurrentTemperature);
                syslog(LOG_NOTICE, printBuffer);
                sprintf(printBuffer, "DEBUG: Daily Temp High = %6.2f", DailyTempHigh);
                syslog(LOG_NOTICE, printBuffer);
                sprintf(printBuffer, "DEBUG: Daily Temp Low = %6.2f", DailyTempLow);
                syslog(LOG_NOTICE, printBuffer);
                sprintf(printBuffer, "DEBUG: Current Pressure = %6.2f", CurrentPressure);
                syslog(LOG_NOTICE, printBuffer);
                sprintf(printBuffer, "DEBUG: Current Railfall = %6.2f", RainCount * 0.011);
                syslog(LOG_NOTICE, printBuffer);
                sprintf(printBuffer, "DEBUG: Railfall Accumulation = %6.2f", RainAccumulation);
                syslog(LOG_NOTICE, printBuffer);
                sprintf(printBuffer, "DEBUG: Railfall HourlyRainTotal = %6.2f", HourlyRainTotal);
                syslog(LOG_NOTICE, printBuffer);
                sprintf(printBuffer, "DEBUG: Railfall DailyRainTotal = %6.2f", DailyRainTotal);
                syslog(LOG_NOTICE, printBuffer);
                sprintf(printBuffer, "DEBUG: Windspeed = %6.2f", WindAccumulation);
                syslog(LOG_NOTICE, printBuffer);
                sprintf(printBuffer, "DEBUG: Wind Direction = %6.2f", WindDirection);
                syslog(LOG_NOTICE, printBuffer);
                sprintf(printBuffer, "DEBUG: Daily Wind High = %6.2f", DailyWindHigh);
                syslog(LOG_NOTICE, printBuffer);
            }

            if (NPFlag)
            {
                NamedPipeHandle = open(NamedPipe,O_WRONLY);
                if (NamedPipeHandle<0)
                {
                    syslog (LOG_NOTICE, "Failed to open named pipe /tmp/OSWABoxPipe\n");
                    exit(1);
                }

                sprintf(printBuffer,"%s,%4.2f,%4.2f,%4.2f,%3.2f\n",
                    CurrentTime,CurrentTemperature,CurrentPressure,WindAccumulation,WindDirection);
                write(NamedPipeHandle, printBuffer, strlen(printBuffer));
                close(NamedPipeHandle);
            }
        }
        // Sleep untill we need to collect observations again
        sleep ( CollectionPeriod );

        // #ifdef DEBUG				// Only run one cycle and quit if DEBUGing
        //       break;
        // #endif

    }
    //
    // Close file and exit
    //
    syslog (LOG_NOTICE, "Terminated.");
    closelog();

    return EXIT_SUCCESS;
}


void print_usage(const char *prog)
{

    printf("Open Source Weather and Air quality Box Daemon\n");
    printf(" - Version 0.0.0\n\n");
    printf("Usage: %s [-dghpv]\n", prog);
    puts(   "  -d --debug      - Write values to daemon syslog\n"
        "  -g --gps        - Turn on the GPS\n"
        "      s - Serial device\n"
        "      d - GPS device\n"
        "  -h --help       - Print this help message\n"
        "  -n --namedpipe  - write CSV data to named pipe /tmp/OSWABoxPipe\n"
        "  -p --period     - number of seconds between observations default 180\n"
        "  -v --version    - Print the version information\n" );
    exit(1);
}


void parse_opts(int argc, char *argv[])
{
    while (1)
    {
        static const struct option lopts[] =
        {
            { "debug", no_argument, NULL, 'd' },
            { "gps",required_argument, NULL, 'g' },
            { "help", no_argument, NULL, 'h' },
            { "namedpipe", no_argument, NULL, 'n' },
            { "period", required_argument, NULL, 'r' },
            { "version", no_argument, NULL, 'v' },
            { NULL, 0, 0, 0 },
        };
        int c;

        c = getopt_long(argc, argv, "dg:hnp:v", lopts, NULL);

        if (c == -1)
            break;

        switch (c)
        {
            case 'd':
                debugFlag = 1;
                break;
            case 'g':
                GPSflag = 1;
                strcpy(gpstype, optarg);
                break;
            case 'h':
                print_usage(argv[0]);
                break;
            case 'n':
                NPFlag = 1;
                break;
            case 'p':
                CollectionPeriod = atoi(optarg);
                break;
            case 'v':
                printf("OSWABox Version 0.0.0 Beta\n");
                break;
            default:
                print_usage(argv[0]);
                break;
        }

    }
}


/*
// Terminate and stay resident
*/
static void become_daemon(void)
{
    pid_t pid;

    /* Fork off the parent process */
    pid = fork();

    /* An error occurred */
    if (pid < 0)
        exit(EXIT_FAILURE);

    /* Success: Let the parent terminate */
    if (pid > 0)
        exit(EXIT_SUCCESS);

    /* On success: The child process becomes session leader */
    if (setsid() < 0)
        exit(EXIT_FAILURE);

    /* Catch, ignore and handle signals */
    //TODO: Implement a working signal handler */
    signal(SIGCHLD, SIG_IGN);
    signal(SIGHUP, SIG_IGN);

    /* Fork off for the second time*/
    pid = fork();

    /* An error occurred */
    if (pid < 0)
        exit(EXIT_FAILURE);

    /* Success: Let the parent terminate */
    if (pid > 0)
        exit(EXIT_SUCCESS);

    /* Set new file permissions */
    umask(0);

    /* Change the working directory to the root directory */
    /* or another appropriated directory */
    chdir("/");

    /* Close all open file descriptors */
    int x;
    for (x = sysconf(_SC_OPEN_MAX); x>0; x--)
    {
        close (x);
    }

    /* Open the log file */
    openlog ("oswaboxd", LOG_PID, LOG_DAEMON);
}


// Interrupt  called every time an event occurs
void RainInterrupt(void)
{
    WindCount++;
}


// Interrupt called every time an event occurs
void WindInterrupt(void)
{
    RainCount++;
}


//
// Return air pressure in hPa
//
float pressure(int temp)
{
    int fileDescriptor;

    BMP085 *sensor;
    sensor = (BMP085 *) malloc(sizeof(BMP085));

    sensor->i2cAddress = BMP085_i2cAddress;
    sensor->oss = BMP085_Mode;

    fileDescriptor = open(BMP085_device, O_RDWR);
    if (fileDescriptor<0)
    {
        syslog (LOG_NOTICE, "Failed to open i2c device!\n");
        exit(1);
    }

    if (ioctl(fileDescriptor, I2C_SLAVE, sensor->i2cAddress)<0)
    {
        syslog (LOG_NOTICE, "Failed to select BMP085 i2c device!\n");
        exit(1);
    }

    if ( ! readCalibrationTable(fileDescriptor,sensor))
    {
        syslog (LOG_NOTICE, "Failed to read BMP085 calibration table!\n");
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
