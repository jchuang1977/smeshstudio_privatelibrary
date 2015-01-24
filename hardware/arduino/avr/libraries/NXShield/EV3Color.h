//EV3SensorAdapter.h
//
/*
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
*/
/*
 * History
 * ------------------------------------------------
 * Author     Date      Comments
 * Deepak     11/21/13  Initial Authoring.
 * M. Giles   01/28/14  Added I2C address and fully tested 
 */


#ifndef	EV3_COLOR_H
#define	EV3_COLOR_H

#define	ESA_Command	0x41

#include	"NXShieldI2C.h"
#include	"EV3SensorAdapter.h"

/**
 * \enum MODE_Color, Modes supported by EV3 Color Sensor
 */
typedef enum {
  MODE_Color_ReflectedLight     = 0x00,   /*!< Choose for measuring reflected light from sensor */
  MODE_Color_AmbientLight     = 0x01,   /*!< Choose for measuring ambient light around the sensor */
  MODE_Color_MeasureColor     = 0x02,   /*!< Choose to measure color */
} MODE_Color;
	/** @brief This class implements EV3 Color Sensor using EV3SensorAdapter.
      */
class EV3Color : public EV3SensorAdapter
{
public:
       
    EV3Color(uint8_t i2c_address = 0x32);
	 /** Class constructor for EV3Color; custom i2c address is an optional parameter */	
    uint8_t setMode(MODE_Color);
	 /** Color sensor's values will be specific for Reflected Light, Ambient Light, and Color modes. */ 
    
};

#endif
