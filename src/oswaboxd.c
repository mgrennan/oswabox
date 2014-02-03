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

//
// GPIO pin decrelations
//
#define LED 3                           // The wiringPi pin for the LED
#define DHT_PIN  7                      // The wiringPi pin for the RHT03
#define WIND_PIN 0                      // The wiringPi pin for the wind speed
#define RAIN_PIN 1                      // The wiringPi pin for the rain guage

//
// Global program variables
//
int GPSflag = 0;                        // collect GPS data
char gpstype[20] = "";                  // The GPS is s=serial device, d=gps daemon device
int collectionPeriod = 120 ;		// Default delay between observation collections in seconds 

int main(int argc, char **argv)
{

    parse_opts(argc,argv);		// check for command line arguments

    if (wiringPiSetup () < 0)
    {
        fprintf (stderr, "Unable to setup wiringPi.\n");
        return 1;
    }
    pinMode (LED, OUTPUT) ;		// The the LED IO pin as Output
    digitalWrite (LED, LOW) ;           // Turn LED Off

    become_daemon();			// Make this program run in the background

    syslog (LOG_NOTICE, "Starting weather collection.");
    while (1)
    {

	digitalWrite (LED, HIGH) ;       // LED On

        //TODO: Collect Weather information here.
        sleep (2) ;			// FAKE PRODESS FOR NOW !!!

	digitalWrite (LED, LOW) ;       // LED Off

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

