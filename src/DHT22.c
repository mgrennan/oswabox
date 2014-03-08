/*
//  temperature.c
//
//    Read temperature and humity form DHT03 via 1-Wire interface.
//
//  NOTE: This program requires use of the WiringPi-2 libraries found
//  in the WiringPi2-Python code.
//
//  SEE: https://github.com/Gadgetoid/WiringPi2-Python
//
*/

#define DEBUG

#include <maxdetect.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <getopt.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h>
#include <sys/time.h>

#include <wiringPi.h>

void print_usage(const char *);
void parse_opts(int argc, char *argv[]);
int read_RHT (int, float *, float *);
long long current_timestamp();

#define MAXTIMINGS 85
static int DHTPIN = 7;

int count = 1;

int main(int argc, char *argv[])
{
    float myTemp = 0 ;
    float T = 0 ;
    float myRelHumidity  = 0 ;
    float H = 0 ;
    int attempts ;
    int goodReading ;
    int i  ;

    parse_opts(argc,argv);

    if (wiringPiSetup () == -1)
    {
        printf("\n\nERR: setup for wiringPi failed\n\n");
        exit (1);
    }

    for ( i=1; i<=count; i++)
    {
        goodReading = 0;
        attempts = 10;
        while (!goodReading && (attempts-- > 0))
        {
            if ( read_RHT (DHTPIN, &myTemp, &myRelHumidity))
                goodReading = 1; // stop now that it worked
        }

        T = myTemp / 10.0 ;
        H = myRelHumidity / 10.0 ;
        printf("Temperature: % 11.4f C\t= % 5.2f F\n", T, (T * 1.8) + 32) ;
        printf("   Humidity: % 11.4f %%\n", H);
        if ( count > 1 )
            sleep(2);
    }

    return 0;
}


void print_usage(const char *prog)
{
    puts("\n\ndtemperature - Read and print temperature and humidity from DHT22 via one-wire interface\n\n");
    printf("Usage: %s [-chp]\n", prog);
    puts("  -c --count\t number of time to data temp");
    puts("  -h --help\t print this help message");
    puts("  -p --pin\t\t wireinPi pin number. Default 7;");
    exit(1);
}


void parse_opts(int argc, char *argv[])
{
    while (1)
    {
        static const struct option lopts[] =
        {
	    { "count", required_argument, NULL, 'c' },
            { "help", no_argument, NULL, 'h' },
            { "pin", required_argument, NULL, 'p' },
            { NULL, 0, 0, 0 },
        };
        int c;

        c = getopt_long(argc, argv, "c:hp:", lopts, NULL);

        if (c == -1)
            break;

        switch (c)
        {
	    case 'c':
		count = atoi(optarg);
		break;
            case 'h':
                print_usage(argv[0]);
                break;
            case 'p':
            {
                DHTPIN = atoi(optarg);
            }
            break;
            default:
                print_usage(argv[0]);
                break;
        }
    }
}


/*
 * read_RHT:
 *      Read the Temperature & Humidity from an RHT03 sensor
 *********************************************************************************
 */

int read_RHT (const int pin, float *temp, float *rh)
{
    static long long nextTime   = 0 ;
    long long now ;
    static int lastTemp   = 0 ;
    static int lastRh     = 0 ;
    static int lastResult = 0 ;

    unsigned char buffer [4] ;

    now = current_timestamp();
    if ( now < nextTime)         // if < second sience last call
    {
        *temp = lastTemp ;       // return last reading
        *rh   = lastRh ;
        return 1 ;
    }

    #ifdef DEBUG
    printf("Reading NEW data\n");
    #endif
    lastResult = maxDetectRead (pin, buffer) ;

    if (lastResult)
    {
        *rh        = lastRh     =  (buffer [0] * 256 + buffer [1]) ;
        *temp      = lastTemp   =  (buffer [2] * 256 + buffer [3]) ;
                                 // next call no sooner then now + 1 second
        nextTime   = current_timestamp() + 2000 ;
        return 1 ;
    }
    else
    {
        return 0 ;
    }
}


long long current_timestamp()
{
    struct timeval te;
    gettimeofday(&te, NULL);     // get current time
                                 // caculate milliseconds
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000;
    // printf("milliseconds: %lld\n", milliseconds);
    return milliseconds;
}
