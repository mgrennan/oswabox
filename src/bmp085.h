#define createUword(MSB,LSB) (unsigned short)((MSB << 8) | LSB)
#define createSword(MSB,LSB) (short)((MSB << 8) | LSB)

//Define addresses of important registers and commands
#define STARTTEMPMEAS 0x2EF4
#define CALTABLEADDR 0xAA
#define TEMPADDR 0xF6

typedef enum oversampling
{
        ultraLowPower = 0,
        standard,
        highResolution,
        ultraHighResolution
} overSampling;

typedef struct bmp085
{
        char i2cAddress;
        unsigned char calibrationCoeficients[22];
        float temperature;
        long pressure;
        overSampling oss;
} BMP085;
