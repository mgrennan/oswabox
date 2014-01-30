/*
//  gps-gpsd.c
//    Mark Grennan - 2014-01-28
//
//    Print GPS information from the gpsd deamon.
*/
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <gps.h>

#define host "127.0.0.1"
#define port "2947"

struct gps_data_t gpsdata;

int main(void)
{
	char time_str[32];
	printf("Display the GPS information from gpsd.\n");

	if (gps_open(host, port, &gpsdata) != 0)
	{
		(void)fprintf(stderr, "gps-gpsd - no gpsd running or network error: %d - %s\n",errno,gps_errstr(errno));
		exit(1);
	}

	gps_stream(&gpsdata, WATCH_ENABLE, NULL);

	for (;;)					 /* heart of the client */
	{
		if ( !gps_waiting(&gpsdata, 5000000) )
		{
			printf("GPS Timmed out\n");
			exit(EXIT_FAILURE);
		}
		else
		{

			gps_read(&gpsdata);
			if( gpsdata.fix.mode > STATUS_NO_FIX )
			{
				unix_to_iso8601(gpsdata.fix.time, time_str, sizeof(time_str));
				printf("Date: %s Lat: %lf lon: %lf Atl: %lf\n",
					time_str,
					gpsdata.fix.latitude,
					gpsdata.fix.longitude,
					gpsdata.fix.altitude
				);
			}
		}
	}

	//	gps_stream(&gps_data, WATCH_DISABLE, NULL);

	gps_close(&gpsdata);
	return EXIT_SUCCESS;
}
