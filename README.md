oswabox - Version 0.3 - ALPHA
=======

Open Source Weather and Air quality station for the Raspberry Pi

This code is developted to read weather data from the Open Source
Weather and Air Box (OSWABox) based on the AirPi (AirPi.es).

The orginal code for the AirPi was written in Python.  This code uses
the wiringPi library (wiringPi.com) and is written in C. Many of these 
program are based on wiringPi Examples

Example programs are:

    blinkgreen.c - Blink the Green LED

    temp.c - Read the tempiture and humity from the RHT03

    pressure.c - Read the air pressure, temp and altitude from the BMP085

    mcp3008.c - Read the voltage on one pin of the mcp3008

    airqual.c - Read the resistence of each of the devices on the mcp3008

    winc.c - Read the wind speed and direction fromt the wind sensors

    rain.c - Read the rain sensor

    position.c - Read the data from the gps and display the time, location and altitude

    oswabox.c - This program brings all the guages into one programs and outputs
	the data into a number of formats.

I want to thank Walter Dal Mut for the gps library used here. https://github.com/wdalmut/libgps

=================

To compile all the examples and put them in ~/bin type

    cd src
    make

To compile an individual example, just type

    make exampleName

The resulting appliation are written to the build directory.

