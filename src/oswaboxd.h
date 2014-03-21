#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <getopt.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <gps.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>

#include <wiringPi.h>
#include "wiringPiSPI.h"
#include <gertboard.h>
#include "bmp085.h"

extern int readCalibrationTable(int,BMP085 *);
extern void makeMeasurement(int, BMP085 *);

void    print_usage(const char *);
void    parse_opts(int argc, char *argv[]);
void    become_daemon() ;
void    RainInterrupt(void);
void    WindInterrupt(void);
double  read_adc_ohms(int);
long    readadc(int);
uint8_t sizecvt(int);
double  read_bmp085(int);
int     read_dht22_dat();
double  windDirGuage(void);
void    windAverage(void);
void    signal_handler(int);
double  ohms2lux(double);
double  dewPoint(double, double);
double  windChill(double, double);
double  heatIndex(double, double);

//
// GPIO pin decrelations
//
#define LED_PIN  3                                          // The wiringPi pin for the LED
#define DHT_PIN  7                                          // The wiringPi pin for the RHT03
#define WIND_PIN 0                                          // The wiringPi pin for the wind speed
#define RAIN_PIN 1                                          // The wiringPi pin for the rain guage
#define TGS2600 0                                           // ADA pins
#define MiSC2710 1
#define MiSC5525 2
#define Sound 3
#define WindRaw 4

//
// Global program variables
//
char GPShost[20] = "127.0.0.1";                             // Host IP for GPSd data
char GPSport[20] = "2947";                                  // Host PORT number for GPSd data
char *BMP085_device = "/dev/i2c-1";                         // Linux device for I2C buss
char BMP085_i2cAddress = 0x77;                              // Device number on I2C buss for BMP085
char *NamedPipe = "/tmp/OSWABoxPipe";                       // Named Pipe for CSV output
char *CSVFile = "/tmp/oswadata.csv";                        // Named Pipe for CSV output
char *WeeFile = "/tmp/oswadata";

