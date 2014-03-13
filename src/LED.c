/*
//  blink.c
//  Mark Grennan - 2014-02-01
//
//  Using the MCP3008 AD converter (ADC) read the Air Quality
//  insterment and print the values.
//
//  based on isr.c from the WiringPi library, authored by Gordon Henderson
//  https://github.com/WiringPi/WiringPi/blob/master/examples/isr.c
//
//  Compile as follows:
//
//      gcc -o blink blink.c -lwiringPi
//
//  Run as follows:
//
//      sudo ./blink
*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <getopt.h>
#include <math.h>

#include <wiringPi.h>

void print_usage(const char *);
void parse_opts(int argc, char *argv[]);

int LED_PIN = 3;
int count = 4;

int main (int argc, char *argv[])
{
    int i;

    parse_opts(argc,argv);

    wiringPiSetup () ;
    pinMode (LED_PIN, OUTPUT) ;

    for( i=0; i< count; i++) {
        digitalWrite (LED_PIN, HIGH) ;                      // On
        //		printf("On\n");
        delay (500) ;                                       // mS
        digitalWrite (LED_PIN, LOW) ;                       // Off
        //		printf("Off\n");
        delay (500) ;
    }
    return 0 ;
}


void print_usage(const char *prog)
{
    puts("\nblink - Blink the LED\n");
    printf("Usage: %s [-chp]\n", prog);
    puts("  -c --count\t number of times to blink - Default 4");
    puts("  -h --help\t print this help message");
    puts("  -p --pin\t the GPIO pin connected to the LED - Default 3");
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
            {
                count = atoi(optarg);
            }
            break;
            case 'h':
                print_usage(argv[0]);
                break;
            case 'p':
            {
                LED_PIN = atoi(optarg);
            }
            break;
            default:
                print_usage(argv[0]);
                break;
        }
    }
}
