/*
// oswaboxd - Open Source Weather and Air quality daemon
//
//   This program run in the background as a daemon and collect weather
// and air quality information from the OSWABox hardware on connected
// to a RaspberryPi computer.
//
*/

#define DEBUG                           // uncomment to do debuging

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <getopt.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <syslog.h>

#include <wiringPi.h>
#include "wiringPiSPI.h"
#include <gertboard.h>

void print_usage(const char *);
void parse_opts(int argc, char *argv[]);
static void become_daemon() ;
void RainInterrupt(void);
void WindInterrupt(void);

//
// GPIO pin decrelations
//
#define LED_PIN  3                      // The wiringPi pin for the LED
#define DHT_PIN  7                      // The wiringPi pin for the RHT03
#define WIND_PIN 0                      // The wiringPi pin for the wind speed
#define RAIN_PIN 1                      // The wiringPi pin for the rain guage

//
// Application Definations
//
#define SecondsResluation 10 

//
// Global program variables
//
int GPSflag = 0;                        // collect GPS data
char gpstype[20] = "";                  // The GPS is s=serial device, d=gps daemon device
int collectionPeriod = 120 ;		// Default delay between observation collections in seconds 
volatile unsigned long RainCount = 0;   // Counter for tipping bucket 0.011 inches per event
volatile unsigned long WindCount = 0;	// Counter for wind speed s 1.492 mph per event

int main(int argc, char **argv)
{
    float RainAccumulation = 0;		// Current rainfall accumulation
    float WindAccumulation = 0;
    float HourlyRainTotal = 0;		// Total rainfall over the last hour
    float HourlyWindHigh = 0;
    float DailyTempHigh = 0;
    float DailyTempLow = 0;
    float DailyRainTotal = 0;		// Totlal rainfall for today
    float DailyWindHigh = 0;
    char printBuffer[80];

    parse_opts(argc,argv);		// check for command line arguments

//
// Initilise hardware and program settings
//
    if (wiringPiSetup () < 0)		// Setup all Raspberry Pi Pins
    {
        fprintf (stderr, "Unable to setup wiringPi.\n");
        return 1;
    }
    pinMode (LED_PIN, OUTPUT) ;		// Set LED pin as an output
    digitalWrite (LED_PIN, LOW) ;       // LED Off
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
    become_daemon();			// Make this program run in the background

//
// Mail Loop
//
    syslog (LOG_NOTICE, "Starting weather collection.");
    while (1)
    {

	digitalWrite (LED_PIN, HIGH) ;  // LED On

        RainAccumulation = RainCount * 0.011 ;
	HourlyRainTotal += RainAccumulation;
	DailyRainTotal += RainAccumulation;

#ifdef DEBUG
        sprintf(printBuffer, "DEBUG: Current Railfall = %6.2f", RainCount * 0.011);
	syslog(LOG_NOTICE, printBuffer);
        sprintf(printBuffer, "DEBUG: Railfall Accumulation = %6.2f", RainAccumulation);
	syslog(LOG_NOTICE, printBuffer);
        sprintf(printBuffer, "DEBUG: Railfall HourlyRainTotal = %6.2f", HourlyRainTotal);
	syslog(LOG_NOTICE, printBuffer);
        sprintf(printBuffer, "DEBUG: Railfall DailyRainTotal = %6.2f", DailyRainTotal);
	syslog(LOG_NOTICE, printBuffer);
#endif

        //TODO: Collect Weather information here.
        sleep (SecondsResluation) ;	// FAKE PRODESS FOR NOW !!!

	digitalWrite (LED_PIN, LOW) ;   // LED Off

        sleep ( collectionPeriod );	// Sleep untill we need to collect observations again

#ifdef DEBUG				// Only run one cycle and quit if DEBUGing
        break;
#endif

    }

    syslog (LOG_NOTICE, "Terminated.");
    closelog();

    return EXIT_SUCCESS;
}

void print_usage(const char *prog)
{

        printf("Open Source Weather and Air quality Box Daemon\n");
        printf(" - Version 0.0.0\n\n");
        printf("Usage: %s [-ghv]\n", prog);
        puts(   "  -g --GPS     - Turn on the GPS\n"
                "      s - Serial device\n"
                "      d - gps device\n"
		"  -h --help    - Print this help message"
		"  -v --version - Print the version information" );
        exit(1);
}

void parse_opts(int argc, char *argv[])
{
        while (1)
        {
                static const struct option lopts[] =
                {
                        { "GPS",required_argument, NULL, 'g' },
                        { "help", no_argument, NULL, 'h' },
                        { NULL, 0, 0, 0 },
                };
                int c;

                c = getopt_long(argc, argv, "g:hv", lopts, NULL);

                if (c == -1)
                        break;

                switch (c)
                {
                        case 'g':
                                GPSflag = 1;
                                strcpy(gpstype, optarg);
                                break;
                        case 'h':
                                print_usage(argv[0]);
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
//
*/
static void become_daemon()
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

