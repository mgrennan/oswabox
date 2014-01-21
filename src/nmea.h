#ifndef _NMEA_H_
#define _NMEA_H_

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#define _EMPTY 0x00
#define NMEA_GPRMC 0x01
#define NMEA_GPRMC_STR "$GPRMC"
#define NMEA_GPGGA 0x02
#define NMEA_GPGGA_STR "$GPGGA"
#define NMEA_UNKNOWN 0x00
#define _COMPLETED 0x03

#define NMEA_CHECKSUM_ERR 0x80
#define NMEA_MESSAGE_ERR 0xC0

struct gpgga {
    double latitude;	// Latitude eg: 4124.8963 (XXYY.ZZKK.. DEG, MIN, SEC.SS)
    char lat;		// Latitude eg: N
    double longitude;	// Longitude eg: 08151.6838 (XXXYY.ZZKK.. DEG, MIN, SEC.SS)
    char lon;		// Longitude eg: W
    uint8_t quality;	// Quality 0, 1, 2
    uint8_t satellites;	// Number of satellites: 1,2,3,4,5...
    double altitude;	// Altitude eg: 280.2 (Meters above mean sea level)
    int hour;		// Hour 0-23
    int minute;		// Minute 0-59
    int second;		// Second 0-59
};
typedef struct gpgga gpgga_t;

struct gprmc {
    double latitude;
    char lat;
    double longitude;
    char lon;
    double speed;
    double course;
    int hour;
    int minute;
    int second;
    int day;
    int month;
    int year;
};
typedef struct gprmc gprmc_t;

uint8_t nmea_get_message_type(const char *);
uint8_t nmea_valid_checksum(const char *);
void nmea_parse_gpgga(char *, gpgga_t *);
void nmea_parse_gprmc(char *, gprmc_t *);

#endif

