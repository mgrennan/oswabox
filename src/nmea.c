#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>

#include "nmea.h"

void nmea_parse_gpgga(char *nmea, gpgga_t *loc)
{
    char *p = nmea;
    char s[3];

    // SAMPLE: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    //    printf("%s\n",p);

    p = strchr(p, ',')+1;                                   // Read hour, minute, second UTC
    s[0] = p[0];
    s[1] = p[1];
    s[2] = 0;
    loc->hour = atoi(s);
    s[0] = p[2];
    s[1] = p[3];
    s[2] = 0;
    loc->minute = atoi(s);
    s[0] = p[4];
    s[1] = p[5];
    s[2] = 0;
    loc->second = atoi(s);

    p = strchr(p, ',')+1;
    loc->latitude = atof(p);

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'N':
            loc->lat = 'N';
            break;
        case 'S':
            loc->lat = 'S';
            break;
        case ',':
            loc->lat = '\0';
            break;
    }

    p = strchr(p, ',')+1;
    loc->longitude = atof(p);

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'W':
            loc->lon = 'W';
            break;
        case 'E':
            loc->lon = 'E';
            break;
        case ',':
            loc->lon = '\0';
            break;
    }

    p = strchr(p, ',')+1;
    loc->quality = (uint8_t)atoi(p);

    p = strchr(p, ',')+1;
    loc->satellites = (uint8_t)atoi(p);

    p = strchr(p, ',')+1;

    p = strchr(p, ',')+1;
    loc->altitude = atof(p);
}


void nmea_parse_gprmc(char *nmea, gprmc_t *loc)
{
    char *p = nmea;
    char s[3];

    // SAMPLE: $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
    //    printf("%s\n",p);

    p = strchr(p, ',')+1;                                   // Read hour, minute, second UTC
    s[0] = p[0];
    s[1] = p[1];
    s[2] = 0;
    loc->hour = atoi(s);
    s[0] = p[2];
    s[1] = p[3];
    s[2] = 0;
    loc->minute = atoi(s);
    s[0] = p[4];
    s[1] = p[5];
    s[2] = 0;
    loc->second = atoi(s);

    p = strchr(p, ',')+1;                                   // skip status

    p = strchr(p, ',')+1;                                   // read latitude
    loc->latitude = atof(p);

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'N':
            loc->lat = 'N';
            break;
        case 'S':
            loc->lat = 'S';
            break;
        case ',':
            loc->lat = '\0';
            break;
    }

    p = strchr(p, ',')+1;                                   // read longitude
    loc->longitude = atof(p);

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'W':
            loc->lon = 'W';
            break;
        case 'E':
            loc->lon = 'E';
            break;
        case ',':
            loc->lon = '\0';
            break;
    }

    p = strchr(p, ',')+1;                                   // read ground speed in knots
    loc->speed = atof(p);

    p = strchr(p, ',')+1;                                   // read course in degrees true
    loc->course = atof(p);

    p = strchr(p, ',')+1;                                   // read date
    s[0] = p[0];
    s[1] = p[1];
    s[2] = 0;
    loc->day = atoi(s);
    s[0] = p[2];
    s[1] = p[3];
    s[2] = 0;
    loc->month = atoi(s);
    s[0] = p[4];
    s[1] = p[5];
    s[2] = 0;
    loc->year = atoi(s);
}


/**
 * Get the message type (GPGGA, GPRMC, etc..)
 *
 * This function filters out also wrong packages (invalid checksum)
 *
 * @param message The NMEA message
 * @return The type of message if it is valid
 */
uint8_t nmea_get_message_type(const char *message)
{
    uint8_t checksum = 0;
    if ((checksum = nmea_valid_checksum(message)) != _EMPTY) {
        return checksum;
    }

    if (strstr(message, NMEA_GPGGA_STR) != NULL) {
        return NMEA_GPGGA;
    }

    if (strstr(message, NMEA_GPRMC_STR) != NULL) {
        return NMEA_GPRMC;
    }

    return NMEA_UNKNOWN;
}


uint8_t nmea_valid_checksum(const char *message)
{
    uint8_t checksum= (uint8_t)strtol(strchr(message, '*')+1, NULL, 16);

    char p;
    uint8_t sum = 0;
    ++message;
    while ((p = *message++) != '*') {
        sum ^= p;
    }

    if (sum != checksum) {
        return NMEA_CHECKSUM_ERR;
    }

    return _EMPTY;
}
