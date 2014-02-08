/*
// oswaboxd - Open Source Weather and Air quality daemon
//    Copyright: Mark Grennan - 2014/02/06
//
//    This program run in the background as a daemon and collect weather
// and air quality information from the OSWABox hardware on connected
// to a RaspberryPi computer.
//
//    For more information on the OSWABox hardware and software goto
// http://www.oswabox.com/
//
//    LICENSE GPL Version 2.1 
*/

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

void print_usage(const char *);
void parse_opts(int argc, char *argv[]);
static void become_daemon() ;
void RainInterrupt(void);
void WindInterrupt(void);
float pressure(int);
float read_adc_dev(int);
long readadc(int);

//
// GPIO pin decrelations
//
#define LED_PIN  3                                          // The wiringPi pin for the LED
#define DHT_PIN  7                                          // The wiringPi pin for the RHT03
#define WIND_PIN 0                                          // The wiringPi pin for the wind speed
#define RAIN_PIN 1                                          // The wiringPi pin for the rain guage

//
// Application Definations
//

//
// Global program variables
//
int GPSflag = 0;                                            // collect GPS data
int debugFlag = 0;                                          // Output debug inforamtion to syslog daemon file
char gpstype[20] = "";                                      // The GPS is s=serial device, d=gps daemon device
int CollectionPeriod = 20 ;                                 // delay between observation collections in seconds
int ReportPeriod = 9 ;                                      // report results every X collectionPeriods 20*9=180 (3 min)
int BMP085_mode = 2;                                        // 0=ULTRA LOW POWER 1=STANDARD 2=HIGH RESOLUTION 3=ULTRA HIGH RESOLUTION
int ADSamples = 10;                                         // number of read samples to average on the AD converter
int NPFlag = 0;                                             // write obs to named pipe
char GPShost[20] = "127.0.0.1";                             // Host IP for GPSd data
char GPSport[20] = "2947";                                  // Host PORT number for GPSd data
volatile unsigned long RainCount = 0;                       // Counter for tipping bucket 0.011 inches per event
volatile unsigned long WindCount = 0;                       // Counter for wind speed s 1.492 mph per event
char *BMP085_device = "/dev/i2c-0";                         // Linux device for I2C buss
char BMP085_i2cAddress = 0x77;                              // Device number on I2C buss for BMP085
char *NamedPipe = "/tmp/OSWABoxPipe";                       // Named Pipe for CSV output

//
// Let the program begin
//
int main(int argc, char **argv)
{
    time_t currTime;
    struct tm *localTime;
    char CurrentTime[20]  = "";
    int NamedPipeHandle;
    char printBuffer[80];
    int ReportLoop;
    int ADLoop;

    float RainAccumulation = 0;                             // Current rainfall accumulation
    float WindAccumulation = 0;
    float WindDirectionAccumulation = 0;
    float WindDirection = 0;
    float CurrentPressure = 0;
    float CurrentTemperature = 0;

    float HourlyRainTotal = 0;                              // Total rainfall over the last hour

    float DailyTempHigh = -9999.0;
    float DailyTempLow = 9999.0;
    float DailyRainTotal = 0;                               // Total rainfall for today
    float DailyWindHigh = -1;

    float ADAccumulation[8];                                // Current values of AD convert 0-7 

    parse_opts(argc,argv);                                  // check for command line arguments

    //
    // Initilise hardware and program settings
    //
    if (wiringPiSetup () < 0)                               // Setup all Raspberry Pi Pins
    {
        fprintf (stderr, "Unable to setup wiringPi.\n");
        return 1;
    }
    pinMode (LED_PIN, OUTPUT) ;                             // Set LED pin as an output
    digitalWrite (LED_PIN, LOW) ;                           // LED Off
    if ( wiringPiISR (RAIN_PIN, INT_EDGE_FALLING, &RainInterrupt) < 0 )
    {
        fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        return 1;
    }
    if ( wiringPiISR (WIND_PIN, INT_EDGE_FALLING, &WindInterrupt) < 0 )
    {
        fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        return 1;
    }

    become_daemon();
    syslog (LOG_NOTICE, "Starting weather collection.");

    //
    // Mail Loop
    //
    while (1)
    {
        for (ReportLoop=0; ReportLoop<ReportPeriod; ReportLoop++)
        {
            if (debugFlag)
            {
                sprintf(printBuffer, "DEBUG: Collecting Observations %d", ReportLoop+1);
                syslog(LOG_NOTICE, printBuffer);
            }

            //
            // Average these reading over the reporting period
            //
            digitalWrite (LED_PIN, HIGH) ;                  // LED On

            CurrentTemperature = pressure(1);               // Read Air Temperature
            if ( CurrentTemperature > DailyTempHigh )       // Capture Daily High Temp
                DailyTempHigh = CurrentTemperature;
            if ( CurrentTemperature < DailyTempLow )        // Capture Daily Low Temp
                DailyTempLow = CurrentTemperature;

            RainAccumulation = RainCount * 0.011 ;          // Calculate the current rainfall
            HourlyRainTotal += RainAccumulation;            // Accumulate the hourly rain total
            DailyRainTotal += RainAccumulation;             // Accumulate the daily rain total
            RainCount = 0.0;                                // Empty the counter

                                                            // Calculate the current wind speed
            WindAccumulation = WindCount * 1.492 / CollectionPeriod;
            if ( WindAccumulation > DailyWindHigh )         // Capture the daily high wind speed
                DailyWindHigh = WindAccumulation ;
            WindCount = 0.0;

// TODO: Create function to turn AD reading into degrees +- ture north
            WindDirectionAccumulation += -95.0;             // wind direction +- degrees true north
            if (debugFlag > 1)
            {
                sprintf(printBuffer, "DEBUG: Windspeed Accumulation %d=%6.2f", ReportLoop+1,WindDirectionAccumulation);
                syslog(LOG_NOTICE, printBuffer);
            }
            WindDirection = WindDirectionAccumulation / ReportPeriod;
            WindDirection = fmod(WindDirection + 360.0, 360.0); 


            digitalWrite (LED_PIN, LOW) ;                   // LED Off

            if ( ReportLoop+1 == ReportPeriod )             // Report Opbservations
            {
                //
                // These readings only need to be collected once per reporting period
                //
                digitalWrite (LED_PIN, HIGH) ;              // LED On
                currTime = time(NULL);                      // Current Time - from System or GPS
                localTime = localtime(&currTime);
                sprintf(CurrentTime,"20%02d:%02d:%02dT%02d:%02d:%02d",
                    localTime->tm_year-100, localTime->tm_mon+1, localTime->tm_mday,
                    localTime->tm_hour, localTime->tm_min, localTime->tm_sec);

                CurrentPressure = pressure(0) ;             // Read Air Pressure

                for (ADLoop=0; ADLoop<8; ADLoop++)          // Read all the AD values
                {
                    ADAccumulation[ADLoop] = read_adc_dev(ADLoop);
                }
                digitalWrite (LED_PIN, LOW) ;               // LED Off

                if (debugFlag)
                {
                    sprintf(printBuffer, "DEBUG: Current time %s", CurrentTime);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Current Temperature = %6.2f", CurrentTemperature);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Daily Temp High = %6.2f", DailyTempHigh);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Daily Temp Low = %6.2f", DailyTempLow);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Current Pressure = %6.2f", CurrentPressure);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Current Railfall = %6.2f", RainCount * 0.011);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Railfall Accumulation = %6.2f", RainAccumulation);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Railfall HourlyRainTotal = %6.2f", HourlyRainTotal);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Railfall DailyRainTotal = %6.2f", DailyRainTotal);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Windspeed = %6.2f", WindAccumulation);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Wind Direction = %6.2f", WindDirection);
                    syslog(LOG_NOTICE, printBuffer);
                    sprintf(printBuffer, "DEBUG: Daily Wind High = %6.2f", DailyWindHigh);
                    syslog(LOG_NOTICE, printBuffer);
                    for (ADLoop=0; ADLoop<8; ADLoop++)
                    {
                        sprintf(printBuffer, "DEBUG: AD %d = %6.2f", ADLoop, ADAccumulation[ADLoop]);
                        syslog(LOG_NOTICE, printBuffer);
                    }
                }

                if (NPFlag)
                {
                    NamedPipeHandle = open(NamedPipe,O_WRONLY);
                    if (NamedPipeHandle<0)
                    {
                        syslog (LOG_NOTICE, "Failed to open named pipe /tmp/OSWABoxPipe\n");
                        exit(1);
                    }

                    sprintf(printBuffer,"%s,%4.2f,%4.2f,%4.2f,%3.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f\n",
                        CurrentTime,CurrentTemperature,CurrentPressure,WindAccumulation,WindDirection,
                        ADAccumulation[0],ADAccumulation[1],ADAccumulation[2],ADAccumulation[3],
                        ADAccumulation[4],ADAccumulation[5],ADAccumulation[6],ADAccumulation[7]);
                    write(NamedPipeHandle, printBuffer, strlen(printBuffer));
                    close(NamedPipeHandle);
                }
            }

            sleep ( CollectionPeriod );                       // Sleep until we need to collect observations again
        }
        //
        // Zero accumlations for ths reporting period
        //
        WindDirectionAccumulation = 0.0;


    }

    //
    // Close file and exit
    //
    syslog (LOG_NOTICE, "Terminated.");
    closelog();

    return EXIT_SUCCESS;
}


void print_usage(const char *prog)
{
    printf("Open Source Weather and Air quality Box Daemon\n");
    printf(" - Version 0.0.0\n\n");
    printf("Usage: %s [-BdgHhPprsv]\n", prog);
    puts(   "  -B --BMP085 - sets the pressure measurement mode.\n"
        "             0 = ULTRA LOW POWER\n"
        "             1 = STANDARD\n"
        "   Default = 2 = HIGH RESOLUTION\n"
        "             3 = ULTRA HIGH RESOLUTION\n"
        "  -d --debug      - debug level 1=low 2=high 3=everything\n"
        "  -g --gps        - Turn on the GPS\n"
        "             s - Serial device\n"
        "             d - GPS device; Also see host and port\n"
        "  -H --host       - GPS host IP; Default 127.0.0.1\n"
        "  -h --help       - Print this help message\n"
        "  -n --namedpipe  - write CSV data to named pipe /tmp/OSWABoxPipe\n"
        "  -P --port       - GPS port; Default 2947\n"
        "  -p --period     - number of seconds between observations; Default 20\n"
        "  -r --report     - number of observations before a report: Default 9\n"
        "  -s --samples    - number of samples to average the AD converter: Default 10\n"
        "  -v --version    - Print the version information\n" );
    exit(1);
}


void parse_opts(int argc, char *argv[])
{
    while (1)
    {
        static const struct option lopts[] =
        {
            { "BMP085", required_argument, NULL, 'B' },
            { "debug", required_argument, NULL, 'd' },
            { "gps",required_argument, NULL, 'g' },
            { "host",required_argument, NULL, 'H' },
            { "help", no_argument, NULL, 'h' },
            { "namedpipe", no_argument, NULL, 'n' },
            { "port", required_argument, NULL, 'P' },
            { "period", required_argument, NULL, 'p' },
            { "report", required_argument, NULL, 'r' },
            { "samples", required_argument, NULL, 's' },
            { "version", no_argument, NULL, 'v' },
            { NULL, 0, 0, 0 },
        };
        int c;

        c = getopt_long(argc, argv, "B:d:g:H:hnP:p:r:s:v", lopts, NULL);

        if (c == -1)
            break;

        switch (c)
        {
            case 'B':
                BMP085_mode = atoi(optarg);
                break;
            case 'd':
                debugFlag = atoi(optarg);
                break;
            case 'g':
                GPSflag = 1;
                strcpy(gpstype, optarg);
                break;
            case 'H':
		strcpy(GPShost, optarg);
		break;
            case 'h':
                print_usage(argv[0]);
                break;
            case 'n':
                NPFlag = 1;
                break;
            case 'P':
                strcpy(GPSport, optarg);
                break;
            case 'p':
                CollectionPeriod = atoi(optarg);
                break;
            case 'r':
                ReportPeriod = atoi(optarg);
                break;
            case 's':
                ADSamples = atoi(optarg);
		break;
            case 'v':
                printf("OSWABox Version 0.0.0 Beta\n");
                exit(1);
            default:
                print_usage(argv[0]);
                break;
        }

    }
}


/*
// Terminate and stay resident
*/
static void become_daemon(void)
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


// Interrupt  called every time an event occurs
void RainInterrupt(void)
{
    WindCount++;
}


// Interrupt called every time an event occurs
void WindInterrupt(void)
{
    RainCount++;
}


//
// Return air pressure in hPa
//
float pressure(int temp)
{
    int fileDescriptor;

    BMP085 *sensor;
    sensor = (BMP085 *) malloc(sizeof(BMP085));

    sensor->i2cAddress = BMP085_i2cAddress;
    sensor->oss = BMP085_mode;

    fileDescriptor = open(BMP085_device, O_RDWR);
    if (fileDescriptor<0)
    {
        syslog (LOG_NOTICE, "Failed to open i2c device!\n");
        exit(1);
    }

    if (ioctl(fileDescriptor, I2C_SLAVE, sensor->i2cAddress)<0)
    {
        syslog (LOG_NOTICE, "Failed to select BMP085 i2c device!\n");
        exit(1);
    }

    if ( ! readCalibrationTable(fileDescriptor,sensor))
    {
        syslog (LOG_NOTICE, "Failed to read BMP085 calibration table!\n");
        exit(1);
    }

    if ( debugFlag > 1) 
        syslog (LOG_NOTICE, "Reading BMP085 pressure sensor");

    makeMeasurement(fileDescriptor,sensor);

    free(sensor);

    close(fileDescriptor);

    if( temp )
        return sensor->temperature;
    else
        return sensor->pressure/100.0;
}

/*
// Read the Analog to Digital Converter (ADC) and return the current
//   resistence value. The value is calulated on the if the device is
//   in a pullup or pull down circut, the input reference voltage and
//   resistence devider value in ohms.
*/
float read_adc_dev(int pin)
{
    int i ;
    uint16_t ob ;
    float tot, avg, volt, value ;
    /*
    //   vcc ----R1--+--R2---- GND   If device is R1 its a Pull-Up
    //               |               If device is R2 its a Pull-Down
    //              ADC
    */
    struct device
    {
        int pullup ;                                        // 1 = device pulls up the Pin
        float refvolt ;                                     // reverence voltage
        float resistance ;                                  // resistence in ohms
        char *name ;                                        // device name
    }
    dev[8] =
    {
        { 0, 3.3,  10000.0, "LDR light sensor" },
        { 0, 3.3,  22000.0, "TGS2600         " },
        { 0, 3.3,  10000.0, "MiSC-2710       " },
        { 0, 3.3, 100000.0, "MiCS=5525       " },
        { 0, 3.3,  01000.0, "Sound           " },
        { 0, 3.3,  10000.0, "Wind Direction  " },
        { 0, 3.3,  10000.0, "Open            " },
        { 0, 3.3,    990.0, "Test Voltage    " }
    } ;

    if (wiringPiSPISetup (0, 1500000) < 0)                  // initialize the WiringPi API channel and speed
        return -1 ;                                         // the mcp3008 wants clock speed between 1.35 and 3.6Mz

    ob = 0;
    tot = 0;
    for (i=0; i<ADSamples; i++)                             // Read samples
    {
        ob =  readadc(pin);
        tot = tot + (ob * 1.0) ;
        delay( 10 ) ;                                       // wait 10 miliseconds before reading again
    }
    avg = tot / ADSamples ;                                 // calculate the average of the readings

    // See http://en.wikipedia.org/wiki/Voltage_divider

    volt = (dev[pin].refvolt / 1023.0) * avg ;              // calculate the average voltage
    // reference voltage / 10bit AD (1023) * reading

    if ( dev[pin].pullup )                                  // If this is a pullup resister
    {
        value = (( dev[pin].resistance * dev[pin].refvolt ) / volt ) - dev[pin].resistance ;
    }
    else                                                    // this is a pulldown resister
    {
        value = dev[pin].resistance / (( dev[pin].refvolt / volt ) - 1 ) ;
    }

    return value ;
}


/*
// Read the ADC data fromt the SPI buss
// for Details see:
//    https://projects.drogon.net/understanding-spi-on-the-raspberry-pi/
//    http://ww1.microchip.com/downloads/en/DeviceDoc/21295d.pdf - Page 21
//
//      Start Bit Sel/Diff bit
//              V V
//      000000001 1xxx0000 000000000
//                 ^^^
//                 ADC Address
//
//  The bottem 10 bits are space for the returning data.
*/
long readadc(int adcnum)
{
    uint8_t buff[3] = { 0b00000001, 0b10000000, 0b00000000 }
    ;
    long adc;

    buff[1] += adcnum << 4 ;

    if ( debugFlag > 2) 
        syslog (LOG_NOTICE, "Reading MCP3008 AD value");

    wiringPiSPIDataRW(0, buff, 3);

    //      adc = ((buff[1] & 3) << 8) + buff[2];
                                                            // 10 bits of data
    adc = ((buff[1] * 256 ) + buff[2]) & 0b1111111111 ;

    return adc;
}


