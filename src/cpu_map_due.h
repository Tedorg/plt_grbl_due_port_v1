/*
  cpu_map_atmega328p.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* Grbl officially supports the Arduino Uno, but the other supplied pin mappings are
   supplied by users, so your results may vary. This cpu_map file serves as a central
   pin mapping settings file for AVR 328p used on the Arduino Uno.  */


#include "sam.h"

#ifdef GRBL_PLATFORM
#error "cpu_map already defined: GRBL_PLATFORM=" GRBL_PLATFORM
#endif

/*
DUE
	Os X
		Step C8 Digital Pin 40
		Dir A15  D24
		Enable D1  D26 
	Os Y
		Step D3  D28 
		Dir D9   D30
		Enable D10   D32
		
	Os Z
		Step C2 D34
		Dir C4   D36
		Enable C6   D38



*/

#define GRBL_PLATFORM "MaslowDUE"

#define X_STEP_BIT      1  // Due Digital = C.1 pin 33
#define Y_STEP_BIT      2  // Due Digital = C.2 pin 34
#define Z_STEP_BIT      3  // Due Digital = C.3 pin 35
#define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

#define X_DIRECTION_BIT   4  // Due Digital = C.4 pin 36
#define Y_DIRECTION_BIT   5  // Due Digital = C.5 pin 37
#define Z_DIRECTION_BIT   6  // Due Digital = C.6 pin 38
#define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

#define STEPPERS_DISABLE_BIT    7  // Due Pin = C.7 pin 39
#define STEPPERS_DISABLE_MASK   (1<<STEPPERS_DISABLE_BIT)

#ifdef CW_BOARD

  #define X_STEP  40      // 40	PC8	Digital Pin 40 
  #define X_ENABLE 26     // 26	PD1	Digital pin 26
  #define X_DIRECTION 24  // 24	PA15	Digital Pin 24
  #define X_CS 20  // 24	PA15	Digital Pin 24
  #define X_LIMIT_PIN 23   // 24	PA15	Digital Pin 24
 
  #define Y_STEP  28      // 28	PD3	Digital Pin 28
  #define Y_ENABLE 32     // 32	PD10	Digital Pin 32
  #define Y_DIRECTION 30  // 30	PD9	Digital Pin 30
  #define Y_CS 21// 24	PA15	Digital Pin 24
  #define Y_LIMIT_PIN  25 // 24	PA15	Digital Pin 24

  #define Z_ENABLE 38     //38	PC6	Digital Pin 38
  #define Z_STEP  34      // 34	PC2	Digital Pin 34
  #define Z_DIRECTION 36   //36	PC4	Digital Pin 36
  #define Z_CS  22// 24	PA15	Digital Pin 24
  #define Z_LIMIT_PIN 51  // 24	PA15	Digital Pin 24
   
 
#endif

#define X_LIMIT_BIT      14  // Due pin 23 - A.14
#define Y_LIMIT_BIT      0 // Due pin 25 - D.0
#define Z_LIMIT_BIT      12  // Due pin 51 - C.12



#define SCLpin  SCL1    /* EEPROM i2c signals */
#define SDApin  SDA1


/*Stepper Timers */
#define ST_MAIN_TIMER Timer4
#define ST_RESET_TIMER Timer3
#define ST_SYNCH_TIMER Timer5

/*Serial Handler Timer*/
#define SERIAL_TIMER Timer8


#define SPINDLE_TIMER Timer1
#define Spindle_PWM 16      /* output pin for Spindle PWM */
#define Spindle_PERIOD 500 /* 500 hz */

#ifdef CW_BOARD
  #define Encoder_XA 13 /* X encoder phases A & B */
  #define Encoder_XB 2
  #define Encoder_YA 4 /* Y encoder phases A & B */
  #define Encoder_YB 5
  #define Encoder_ZA 10 /* Z encoder phases A & B */
  #define Encoder_ZB 11
 
#endif







#define	X_STEP_PORT	PIOC
#define	X_STEP_PIN	(1 << 8)
#define	Y_STEP_PORT	PIOD
#define	Y_STEP_PIN	(1 << 3)
#define	Z_STEP_PORT	PIOC
#define	Z_STEP_PIN	(1 << 2)

#define	X_DIR_PORT	PIOA
#define	X_DIR_PIN	(1 << 15)
#define	Y_DIR_PORT	PIOD
#define	Y_DIR_PIN	(1 << 9)
#define	Z_DIR_PORT	PIOC
#define	Z_DIR_PIN	(1 << 4)

#define	X_ENA_PORT	PIOD
#define	X_ENA_PIN	(1 << 1)
#define	Y_ENA_PORT	PIOD
#define	Y_ENA_PIN	(1 << 10)
#define	Z_ENA_PORT	PIOC
#define	Z_ENA_PIN	(1 << 6)

#define MOTORX_RIGHT	X_DIR_PORT->PIO_SODR = X_DIR_PIN
#define MOTORX_LEFT		X_DIR_PORT->PIO_CODR = X_DIR_PIN
#define MOTORY_RIGHT	Y_DIR_PORT->PIO_SODR = Y_DIR_PIN
#define MOTORY_LEFT		Y_DIR_PORT->PIO_CODR = Y_DIR_PIN
#define MOTORZ_RIGHT	Z_DIR_PORT->PIO_SODR = Z_DIR_PIN
#define MOTORZ_LEFT		Z_DIR_PORT->PIO_CODR = Z_DIR_PIN

#define X_STEP_SET		X_STEP_PORT->PIO_SODR = X_STEP_PIN
#define X_STEP_CLEAR	X_STEP_PORT->PIO_CODR = X_STEP_PIN
#define Y_STEP_SET		Y_STEP_PORT->PIO_SODR = Y_STEP_PIN
#define Y_STEP_CLEAR	Y_STEP_PORT->PIO_CODR = Y_STEP_PIN
#define Z_STEP_SET		Z_STEP_PORT->PIO_SODR = Z_STEP_PIN
#define Z_STEP_CLEAR	Z_STEP_PORT->PIO_CODR = Z_STEP_PIN

#define MSI_Enable 0

//Ethernet Shield 
#define SD_CS_PIN 17   // due Pin D20
#define CSPIN 27 // Sd Card SS Pin

// Define homing/hard limit switch input pins and limit interrupt vectors. 
// NOTE: All limit bit pins must be on the same port, but not on a port with other input pins (CONTROL).



//  
//// Define probe switch input pin.

#define PROBE_BIT       5  // Uno Analog Pin 5
#define PROBE_MASK      (1<<PROBE_BIT)
//
//// Start of PWM & Stepper Enabled Spindle
#ifdef VARIABLE_SPINDLE
//  // Advanced Configuration Below You should not need to touch these variables
  #define PWM_MAX_VALUE    255.0

#endif // End of VARIABLE_SPINDLE
