/* This file is part of the Maslow Due Control Software.
    The Maslow Due Control Software is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    Maslow Due Control Software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with the Maslow Control Software.  If not, see <http://www.gnu.org/licenses/>.

    Created by:  Larry D O'Cull  Feb 10, 2019
    
    Some portions of this package directly or indirectly pull from from the Maslow CNC 
    firmware for Aduino Mega.   Those parts are Copyright 2014-2017 Bar Smith <https://www.PLT_V2.com/>
    
    */
// This is the main maslow include file

#ifndef maslow_h
#define maslow_h

//
// -- SHIELD SELECTION
//
#define CW_BOARD   /* Uncomment for V2 MakerMade CNC (1.0) Shield */
//#define MakerMadeCNC_V2   /* Uncomment for V2 MakerMade CNC (1.0) Shield */
//#define MakerMadeCNC_V1   /* Uncomment for V1 MakerMade CNC (1.0) Shield */
//#define DRIVER_L298P_12    /* Uncomment this for a L298P version 1.2 Shield */
//#define DRIVER_L298P_11    /* Uncomment this for a L298P version 1.1 Shield */
//#define DRIVER_L298P_10    /* Uncomment this for a L298P version 1.0 Shield */
//#define DRIVER_TLE5206       /* Uncomment this for a TLE5206 version Shield */

// uncomment this to work on PID settings and such using a terminal window
//#define TUNING_MODE 1

// HARDWARE PIN MAPPING
#define HeartBeatLED 13


#define bufferLength 64          // serial buffer length



#define AXES_DISABLE 39

#define MAX_PWM_LEVEL 255
#define MIN_PWM_LEVEL 5



struct PID_MOTION 
{
  long int Error;
  long int Integral;
  long int axis_Position;
  long int last_Position;
  long int target;
  long int target_PS;
  long int last;
  long int Speed;
  long int totalSpeed;
  long int DiffTerm;
  long int iterm;
  long int stepSize;
  int P_PWM;
  int M_PWM;
  int ENABLE;
};

extern struct PID_MOTION x_axis, y_axis, z_axis;
extern long int xSpeed, ySpeed, zSpeed;  // current speed each axis
extern int healthLEDcounter;
extern int stepTestEnable;
extern int posEnabled;

void motorsEnabled(void);
void motorsDisabled(void);

#endif
