/*
 * Copyright (c) 2013  Goce Boshkovski
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 */


#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include "bmp085.h"

const long pressureConversionTime[]={ 5000000L, 8000000L, 14000000L, 26000000L };

int readCalibrationTable(int fd, BMP085 *sensor)
{
        unsigned char cTableAddr = 0xAA;

	if (write(fd,&cTableAddr,1)!=1)
	{
		//printf("Error writing to the sensor.\n");
		return 0;
	}
        if (read(fd,sensor->calibrationCoeficients,22)!=22)
	{
		//printf("Error reading clibration table of the sensor.\n");
		return 0;
	}

	return 1;
}

void makeMeasurement(int fd, BMP085 *sensor)
{
        unsigned short startTempMeas = STARTTEMPMEAS;
        unsigned char tempMSBAddr = TEMPADDR, tempLSBAddr = TEMPADDR + 1;
        unsigned char rawTempMSB,rawTempLSB;
        long rawTemperature,temperature = 0;

        unsigned short STARTPRESMEAS = createUword((0x34+((sensor->oss)<<6)),0xF4);
        unsigned char pMSBAddress = TEMPADDR, pLSBAddress = 0xF7, pXLSBAddress = 0xF8;
        unsigned char pMSB, pLSB, pXLSB;
        long rawPressure = 0, pressure = 0;

        short ac1 = createSword(sensor->calibrationCoeficients[0],sensor->calibrationCoeficients[1]);
        short ac2 = createSword(sensor->calibrationCoeficients[2],sensor->calibrationCoeficients[3]);
        short ac3 = createSword(sensor->calibrationCoeficients[4],sensor->calibrationCoeficients[5]);
        unsigned short ac4 = createUword(sensor->calibrationCoeficients[6],sensor->calibrationCoeficients[7]);
        unsigned short ac5 = createUword(sensor->calibrationCoeficients[8],sensor->calibrationCoeficients[9]);
        unsigned short ac6 = createUword(sensor->calibrationCoeficients[10],sensor->calibrationCoeficients[11]);
        short b1 = createSword(sensor->calibrationCoeficients[12],sensor->calibrationCoeficients[13]);
        short b2 = createSword(sensor->calibrationCoeficients[14],sensor->calibrationCoeficients[15]);
        short mc = createSword(sensor->calibrationCoeficients[18],sensor->calibrationCoeficients[19]);
        short md = createSword(sensor->calibrationCoeficients[20],sensor->calibrationCoeficients[21]);

        struct timespec timer = {
                .tv_sec = 0,
                //.tv_nsec = 5000000L,
		.tv_nsec = pressureConversionTime[sensor->oss],
        };

	long x1, x2, x3, b3, b5, b6 = 0;
	unsigned long b4, b7 =0;

        //Send the command to start the temperature measurement
        write(fd,&startTempMeas,2);

        // wait 4.5ms and then read the temperature raw value
        nanosleep(&timer, NULL);

        //read the temperature raw value
        write(fd,&tempMSBAddr,1);
        read(fd,&rawTempMSB,1);
        write(fd,&tempLSBAddr,1);
        read(fd,&rawTempLSB,1);
        rawTemperature = (long)((rawTempMSB<<8)+rawTempLSB);

	//Send the command to start the pressure measurement
	write(fd,&STARTPRESMEAS,2);

	//wait 4.5ms and then read the pressure raw value
	nanosleep(&timer,NULL);

	//read the pressure raw value
	write(fd,&pMSBAddress,1);
	read(fd,&pMSB,1);
	write(fd,&pLSBAddress,1);
        read(fd,&pLSB,1);
        write(fd,&pXLSBAddress,1);
        read(fd,&pXLSB,1);
	rawPressure = ((pMSB<<16) + (pLSB<<8) + pXLSB)>>(8-sensor->oss);

	//calculate the real temperature value
        x1 = ((rawTemperature - ac6)*ac5)>>15;
        x2 = (mc << 11)/(x1 + md);
        b5 = x1 + x2;
        temperature = (b5 + 8)>>4;
        sensor->temperature = temperature * 0.1;

	//calculate the real pressure value in Pa
	b6 = b5 - 4000;
	x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
	x2 = (ac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = ((( ac1 * 4 + x3 ) << sensor->oss) + 2 ) / 4;
	x1 = (ac3 * b6) >> 13;
	x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1+x2)+2) >> 2;
	b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;
	b7 = ((unsigned long)rawPressure-b3)*(50000>>sensor->oss);
	if (b7<0x80000000)
		pressure=(b7*2)/b4;
	else
		pressure=(b7/b4)*2;
	x1=(pressure >> 8)* (pressure >> 8);
	x1=(x1*3038)>>16;
	x2=(-7357*pressure)>>16;
	pressure+=((x1+x2+3791)>>4);
	sensor->pressure = pressure;

}


void printCalibrationTable(BMP085 *sensor)
{
	const char *calCoefName[] = {"AC1","AC2","AC3","AC4","AC5","AC6","B1","B2","MB","MC","MD"};
	unsigned char i;

	for(i=0;i<11;i++)
	{
		switch (i)
		{
			case 3:
			case 4:
			case 5:
				printf("%s = \t%d\n",calCoefName[i],createUword(sensor->calibrationCoeficients[i*2],sensor->calibrationCoeficients[i*2+1]));
				break;
			default:
				printf("%s = \t%d\n",calCoefName[i],createSword(sensor->calibrationCoeficients[i*2],sensor->calibrationCoeficients[i*2+1]));
		}
	}
	printf("\n");

}
