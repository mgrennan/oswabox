/*
//  gps-serial
//  Mark Grennan - 2014-01-17
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdlib.h>
#include <getopt.h>
#include <math.h>
#include "gps.h"

void print_usage(const char *);
void parse_opts(int argc, char *argv[]);

int main (int argc, char *argv[])
{

    parse_opts(argc,argv);

    gps_init();                                             // Open serial port

    loc_t data;                                             // Create structure

    gps_location(&data);                                    // Read and parse the GPS info

    printf("       Date: 20%02d-%02d-%02d %02d:%02d:%02d\n", data.year, data.month, data.day, data.hour, data.minute, data.second);
    printf("  Longatude: % 11.4f\n", data.latitude);
    printf("   Latitude: % 11.4f\n", data.longitude);
    printf("   Altitude: % 11.4f\n", data.altitude);

    return EXIT_SUCCESS;
}


void print_usage(const char *prog)
{
    puts("\ngps-serial - read GPS data from a serial port\n");
    printf("Usage: %s [-h]\n", prog);
    puts("  -h --help\t print this help message");
    exit(1);
}


void parse_opts(int argc, char *argv[])
{
    while (1) {
        static const struct option lopts[] = {
            { "help", no_argument, NULL, 'h' },
            { NULL, 0, 0, 0 },
        };
        int c;

        c = getopt_long(argc, argv, "h", lopts, NULL);

        if (c == -1)
            break;

        switch (c) {
            break;
            case 'h':
                print_usage(argv[0]);
                break;
            default:
                print_usage(argv[0]);
                break;
        }
    }
}
