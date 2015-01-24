/************************************************************************/
/* 
  Copyright (c) 2013 by mindsensors.com                                
Email: info (<at>) mindsensors (<dot>) com                          

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 ************************************************************************/

#include <Wire.h>
#include <NXShield.h>
#include <EV3Gyro.h>


NXShield    nxshield;
EV3Gyro ev3gyro (0x32);

int inByte = 0;

void setup()
{
    char            str[80];

    Serial.begin(115200);       // start serial for output
    delay(500);                // wait, allowing time to
    // activate the serial monitor

    //
    // NXShield supports multiple transports
    // Initialize the transport for i2c devices.
    //
    nxshield.init(SH_HardwareI2C);

    //
    // Wait until user presses GO button to continue the program
    //
    Serial.println ("Press GO button to continue");
    Serial.println ("then Press LEFT to change mode to Rate");
    Serial.println ("     Press RIGHT to change mode to Angle");
    nxshield.waitForButtonPress(BTN_GO);
    ev3gyro.init( &nxshield, SH_BAS1 );
    ev3gyro.setMode(MODE_Gyro_Angle);

}

void printData(char *token, byte *buf, int len, char *out)
{
    char temp[10];
    int i;
    strcpy(out, token);
    for (i=0; i < len; i++) {
        if ( buf[i] >= ' ' && buf[i] <= '~' ) {
            sprintf (temp, "'%c' ", buf[i]);
        } else {
            sprintf (temp, "x%02x ", buf[i]);
        }
        strcat(out, temp);
    }

}

void loop()
{
    char            str[256];
    byte buf[64];
    int value;

    boolean left_btn = nxshield.getButtonState(BTN_LEFT);
    if ( left_btn == true ) {
        Serial.println ( "changing mode to Rate" );
        ev3gyro.setMode(MODE_Gyro_Rate);
    }

    boolean right_btn = nxshield.getButtonState(BTN_RIGHT);
    if ( right_btn == true ) {
        Serial.println ( "changing mode to Angle" );
        ev3gyro.setMode(MODE_Gyro_Angle);
    }

    buf[0] = '\0';
    value = ev3gyro.readValue();
    sprintf (str, "value: %d", value);
    Serial.println ( str );


    delay(300);

}

