/*
//  gps-gpsd.c
//    Mark Grennan - 2014-01-28
//
//    Print GPS information from the gpsd deamon.
//
//    If you realy want a program to dispya this use cgps.
*/
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <gps.h>

char host[20] = "127.0.0.1";
char port[20] = "2947";

void print_usage(const char *);
void parse_opts(int argc, char *argv[]);

#define COUNT 10

static struct gps_data_t gpsdata;

int main (int argc, char *argv[])
{
    char time_str[32];
    int done = 0;
    float avgLong[COUNT];
    float avgLat[COUNT];
    float avgAlt[COUNT];
    float average = 0.0;

    parse_opts(argc,argv);

    if (gps_open(host, port, &gpsdata) != 0) {
        fprintf(stderr,
            "gps-gpsd: no gpsd running or network error: %s\n", gps_errstr(errno));
        exit(2);
    }

    gps_stream(&gpsdata, WATCH_ENABLE | WATCH_NMEA, NULL);

    int i = COUNT-1;
    while ( i ) {

        while( !done ) {

            if (!gps_waiting(&gpsdata, 5000000)) {
                fprintf(stderr, "gps-gpsd: GPS Timeout\n");
                exit(1);
            }
            else {
                if (gps_read(&gpsdata) == -1) {
                    fprintf(stderr, "gps-gpsd: IP socket error\n");
                    exit(1);
                }
                else {
                    unix_to_iso8601(gpsdata.fix.time, time_str, sizeof(time_str));
                    printf("       Date: %s\n  Longitude: % 11.4f\n   Latitude: % 11.4f\n   Altitude: % 11.4f\n",
                        time_str,
                        gpsdata.fix.longitude,
                        gpsdata.fix.latitude,
                        gpsdata.fix.altitude
                        );
                    done = 1;
                }
            }

        }
        done = 0;
        avgLong[i] = gpsdata.fix.longitude;
        avgLat[i] = gpsdata.fix.latitude;
        avgAlt[i] = gpsdata.fix.altitude;
        i--;
        sleep(5);
    }
    for (i=0; i<COUNT; i++)
        average += avgLong[i];
    average /= COUNT;
    printf(" Averaged Longatude: %11.4f\n", average);
    average = 0.0;
    for (i=0; i<COUNT; i++)
        average += avgLat[i];
    average /= COUNT;
    printf("           Latitude: %11.4f\n", average);
    average = 0.0;
    for (i=0; i<COUNT; i++)
        average += avgAlt[i];
    average /= COUNT;
    printf("           Altitude: %11.4f\n", average);

    //	gps_stream(&gps_data, WATCH_DISABLE, NULL);

    gps_close(&gpsdata);
    return EXIT_SUCCESS;
}


void print_usage(const char *prog)
{
    puts("\ngps-gpsd - read GPS data form the gpsd device\n");
    printf("Usage: %s [-hHp]\n", prog);
    puts("  -h --help\t print this help message");
    puts("  -H --host\t input the host IP - Deafult 127.0.0.1");
    puts("  -p --post\t input the port - Default 2947");
    exit(1);
}


void parse_opts(int argc, char *argv[])
{
    while (1) {
        static const struct option lopts[] = {
            { "help", no_argument, NULL, 'h' },
            { "host", required_argument, NULL, 'H' },
            { "port", required_argument, NULL, 'p' },
            { NULL, 0, 0, 0 },
        };
        int c;

        c = getopt_long(argc, argv, "hH:p:", lopts, NULL);

        if (c == -1)
            break;

        switch (c) {
            case 'h':
                print_usage(argv[0]);
                break;
            case 'H':
                break;
            case 'p':
                break;
            default:
                print_usage(argv[0]);
                break;
        }
    }
}
