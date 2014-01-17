echo This program uses the gpio program build with wiringPi
echo to display the values of the mcp3008.
gpio -x mcp3004:100:0 readall
