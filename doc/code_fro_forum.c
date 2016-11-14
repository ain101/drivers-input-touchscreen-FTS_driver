// http://www.newhavendisplay.com/NHD_forum/index.php?topic=82.0

//---------------------------------------------------------
/*
CTP_Register_Test_mega.ino
Program for writing/reading to/from Newhaven Display Capacitive Touch Panel with FocalTech FT5x06 controller
Please refer to the below URLs for the touch panel controller documentation:
http://www.newhavendisplay.com/app_notes/FT5x06.pdf
http://www.newhavendisplay.com/app_notes/FT5x06_registers.pdf

(c)2014 Mike LaVine - Newhaven Display International, LLC.

        This program is free software; you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation; either version 2 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.
*/
//---------------------------------------------------------
#include <Wire.h>

int WAKE = 14;    //WAKE signal is connected to digital pin 14 on the Arduino Mega2560
int INT = 15;     //INT signal is connected to digital pin 15 on the Arduino Mega2560

//SDA line is connected to digital pin 20 (SDA) on the Arduino Mega2560
//SCL line is connected to digital pin 21 (SCL) on the Arduino Mega2560

unsigned char dummy, gesture, numpoints, t1xh, t1xl, t1yh, t1yl;
unsigned char x, read1, read2, read3, read4;
unsigned int t1x, t1y;
												
const char slave = 0x38;    //slave address of 0x70 shifted over 1 bit
const char slave_read = 0x39;

void init_CTP()
{
	digitalWrite(WAKE, HIGH);
        delay(100);
	digitalWrite(INT, HIGH);
	delay(100);
        digitalWrite(WAKE, LOW);
        delay(200);
        digitalWrite(WAKE, HIGH);
        delay(200);
}
     
void i2c_read(char addr, unsigned char reg)                  //function for reading a specified register
{
  unsigned char x;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, 1);
  while(Wire.available())
  {
    x = Wire.read();
    Serial.println(x, DEC);
  }
}

void i2c_read4times(char addr, unsigned char reg)           //function for reading 4 registers in a row, starting from specified register
{
  unsigned char x;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, 4);
  while(Wire.available())
  {
    read1 = Wire.read();
    read2 = Wire.read();
    read3 = Wire.read();
    read4 = Wire.read();
  }
}
/*****************************************************/
/*****************************************************/
void setup()
{
        pinMode(WAKE, OUTPUT);
        pinMode(INT, INPUT);
        init_CTP();
        Wire.begin();
        Serial.begin(9600);       
}
void loop()
{

   
    while((digitalRead(INT)) == LOW)
    {
        i2c_read4times(slave, 0x00);
        dummy = read1;                               //dummy read
        gesture = read2;                             //which gesture register read
        numpoints = read3;                           //how many touch points register read
        t1xh = read4;                                //upper 8 bits of X-axis touch location
        i2c_read4times(slave, 0x04);
        t1xl = read1;                                //lower 8 bits of X-axis touch location
        t1yh = read2;                                //upper 8 bits of Y-axis touch location
        t1yl = read3;                                //lower 8 bits of X-axis touch location
        dummy = read4;                               //dummy read
        t1x = t1xl | (t1xh << 8);                    //get the 16 bit X-axis touch location
        t1y = t1yl | (t1yh << 8);                    //get the 16 bit Y-axis touch location*/
       
       
       
        i2c_read4times(slave, 0x80);
        dummy = read1;
       
    //****************************************************************************//
    //  UNCOMMENT THE LINE BELOW THAT YOU WANT TO BE SHOWN ON THE SERIAL MONITOR  //
    //****************************************************************************//
       
        //Serial.println(gesture, HEX);delay(25);    //show gesture value
        //Serial.println(numpoints, HEX);delay(25);  //show how many touch points
        //Serial.println(t1x, HEX);delay(25);        //show 16 bit X-axis touch location
        //Serial.println(t1y, HEX);delay(25);        //show 16 bit Y-axis touch location
        Serial.println(dummy, DEC);delay(25);       
       
        //delays are only used here to slow down scrolling in the serial monitor
        //********************************************************************//
    }   
}