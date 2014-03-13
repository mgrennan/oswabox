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
#include "wiringPiSPI.h"

void print_usage(const char *);
void parse_opts(int argc, char *argv[]);
int read_rht (float *, float *);
long long current_timestamp();

static int DHTPIN = 7;

int count = 1;
uint8_t dht_data [4] ;

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

    if (wiringPiSetup () == -1) {
        printf("\n\nERR: setup for wiringPi failed\n\n");
        exit (1);
    }

    for ( i=1; i<=count; i++) {
        goodReading = 0;
        attempts = 10;
        while (!goodReading && (attempts-- > 0)) {
            if ( read_rht (&myTemp, &myRelHumidity))
                goodReading = 1;                            // stop now that it worked
            sleep(1);
        }

        T = myTemp / 10.0 ;
        H = myRelHumidity / 10.0 ;
        printf("Temperature: % 11.4f C\t= % 5.2f F\n", T, (T * 1.8) + 32) ;
        printf("   Humidity: % 11.4f %%\n", H);
    }
    if ( count > 1 )
        sleep(5);

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
    while (1) {
        static const struct option lopts[] = {
            { "count", required_argument, NULL, 'c' },
            { "help", no_argument, NULL, 'h' },
            { "pin", required_argument, NULL, 'p' },
            { NULL, 0, 0, 0 },
        };
        int c;

        c = getopt_long(argc, argv, "c:hp:", lopts, NULL);

        if (c == -1)
            break;

        switch (c) {
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
 * read_rht:
 *      Read the Temperature & Humidity from an RHT03 sensor
 *********************************************************************************
 */

int read_rht (float *temp, float *rh)
{
    static unsigned long now ;
    unsigned long nextTime   = 0 ;
    static int lastTemp   = 0 ;
    static int lastRh     = 0 ;
    static int newResult = 0 ;

    now = time(NULL);
    if ( now < nextTime) {                                  // if < second sience last call
        printf("Returning old results\n");
        *temp = lastTemp ;                                  // return last reading
        *rh   = lastRh ;
        return 1 ;
    }

    printf("Reading dht\n");
    newResult = read_dht_data() ;

    if (newResult) {
        printf("Returning new results\n");
        *rh        = lastRh     =  (dht_data [0] * 256 + dht_data [1]) ;
        *temp      = lastTemp   =  (dht_data [2] * 256 + dht_data [3]) ;
        nextTime   = time(NULL) + 2;                        // next call no sooner then now + 2 second
        return 1 ;
    } else
    return 0 ;
}


int read_dht_data(void)
{
    uint8_t laststate = HIGH;
    uint8_t counter = 0;
    uint8_t j = 0, i;
    unsigned long currenttime;

    pinMode(DHTPIN, OUTPUT);                                // pull the pin high and wait 250 milliseconds
    digitalWrite(DHTPIN, HIGH);
    delay(250);

    dht_data[0] = dht_data[1] = dht_data[2] = dht_data[3] = dht_data[4] = 0;

    digitalWrite(DHTPIN, LOW);                              // now pull it low for ~20 milliseconds
    delay(20);
    digitalWrite(DHTPIN, HIGH);
    delay(20);
    pinMode(DHTPIN, INPUT);

    for ( i=0; i< 1000 ; i++) {                             // read in timings
        counter = 0;
        while (digitalRead(DHTPIN) == laststate) {
            counter++;
            delay(1);
            if (counter == 255) {
                printf("Never got a state chage!\n");
                break;
            }
        }
        if (counter == 255) break;

        laststate = digitalRead(DHTPIN);
        printf("%d",laststate);

        if ((i >= 4) && (i%2 == 0)) {                       // ignore first 3 transitions
            dht_data[j/8] <<= 1;                            // shove each bit into the storage bytes
            if (counter > 16)
                dht_data[j/8] |= 1;
            j++;
        }

    }

    printf("\ndht_data = %d:%d:%d:%d:%d\n", dht_data[0], dht_data[1], dht_data[2], dht_data[3], dht_data[4]);
    pinMode(DHTPIN, OUTPUT);
    digitalWrite(DHTPIN, LOW);
    // check we read 40 bits and that the checksum matches
    if ((j >= 40) &&
    (dht_data[4] == ((dht_data[0] + dht_data[1] + dht_data[2] + dht_data[3]) & 0xFF)) ) {
        return 1;
    }

    return 0;

}
