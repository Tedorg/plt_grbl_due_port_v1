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

   This file (module) specifically contains the step-direction control used to drive the Maslow gear motors
   using the Maslow CNC shield board. Note: 10nf and 3.9K (parallel) must be added from each encoder phase
   lead to ground on the shield PCB in order for it to be compatible with the 3.3V high-speed inputs of the Arduino Due.
  
   The orignal Maslow firmware does not drive the motor in a 3-state (braking mode) or in a position loop, but
   is structured more as a precise velocity-loop based system. The behavior of this new drive setup will sound
   and act a bit different than what may have been experienced with a previous stock-Maslow setup. Generally,
   the GRBL driven system will seem faster overall due to the splining of vectors as the machine moves. The
   top speed will still be limited by the use of the original gear motors which will only go to about 20RPM.
  
   Also note that the chain configuration of this supplied software is for an 'under-sprocket' chain to a sled-ring
   system. The sled-ring simplifies the math to a triangular system and that makes it easier to compensate out any error.
 
 */
#include "MaslowDue.h"

#include "grbl.h"
#include "DueTimer.h"
#include <SPI.h>
#include <SD.h>

int incomingByte = 0;
//int healthLEDcounter = 0;



#ifdef PLT_V2_ENCODER
static volatile uint8_t pid_busy;
#endif

// Declare system global variable structure
system_t sys;

int32_t sys_probe_position[N_AXIS];              // Last probe position in machine coordinates and steps.
volatile uint8_t sys_probe_state;                // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile uint8_t sys_rt_exec_state;              // Global realtime executor bitflag variable for state management. See EXEC bitmasks.

volatile uint8_t sys_rt_exec_alarm;              // Global realtime executor bitflag variable for setting various alarms.
volatile uint8_t sys_rt_exec_motion_override;    // Global realtime executor bitflag variable for motion-based overrides.
volatile uint8_t sys_rt_exec_accessory_override; // Global realtime executor bitflag variable for spindle/coolant overrides.

volatile uint8_t status_ok = 1;
#ifdef DEBUG
volatile uint8_t sys_rt_exec_debug;
#endif
#ifdef PLT_V2
volatile uint8_t sys_rt_exec_tool_state;              // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
#endif

void serial_init(void);
void settings_init(void);            // Load Grbl settings from EEPROM
void stepper_init(void);             // Configure stepper pins and interrupt timers
void system_init(void);              // Configure pinout pins and pin-change interrupt
void serial_reset_read_buffer(void); // Clear serial read buffer
void serialScanner_handler(void);
void gc_init(void); // Set g-code parser to default state
void spindle_init(void);
void coolant_init(void);
void limits_init(void);
void probe_init(void);
void plan_reset(void); // Clear block buffer and planner variables
void st_reset(void);   // Clear stepper subsystem variables.
void plan_sync_position(void);
void gc_sync_position(void);
int protocol_main_loop(void);
void protocol_init(void);
void tuningLoop(void);







void MotorPID_Timer_handler(void) // PID interrupt service routine
{

//   bool x_had_step = 0,y_had_step = 0,z_had_step =0;
//   static bool xdir=0,ydir=0,zdir=0;
//   static bool old_xdir=0,old_ydir=0,old_zdir=0;
//   // printf("x: %li \n",REG_TC2_CV0);
//   //serialScanner_handler();
//   //report_util_encoder_values(REG_TC2_CV0);
//   x_axis.axis_Position = sys_real_position[X_AXIS] = REG_TC0_CV0; // Todo -1 quick fix just for now
//   y_axis.axis_Position = sys_real_position[Y_AXIS] = REG_TC2_CV0;      // Value of Encoder 2
//   long dummy = REG_TC0_SR0;                                            // vital - reading this clears the interrupt flag
//   //int pos_Z = REG_TC2_CV0;  // Value of Encoder 3
// #ifdef PLT_V2_ENCODER
//   if(pid_busy) return;

// pid_busy = 1;
//   x_axis.Error = error(sys.position[X_AXIS],x_axis.axis_Position); // current position error
//   y_axis.Error =error(sys.position[Y_AXIS],y_axis.axis_Position); // current position error
//   z_axis.Error = 0;//z_axis.target - z_axis.axis_Position; // current position error
//                                                        // z_axis.Error = z_axis.target - z_axis.axis_Position; // current position error

//   // if( (x_axis.Error == 0)){
//    //printf("%d  %d   %d  \n", x_axis.Error,y_axis.Error,z_axis.Error );
//   // }
//   x_axis.Error > 0 ? xdir = 1 : xdir = 0;
//   y_axis.Error > 0 ? ydir = 1 : ydir = 0;
//   z_axis.Error > 0 ? zdir = 1 : zdir = 0;
// if ( old_xdir != xdir || old_ydir != ydir || old_zdir != zdir){
//  if(xdir){
//     MOTORX_RIGHT;
//     }
//     else{ 
//     MOTORX_LEFT;
//     }
//     if(ydir){
//     MOTORY_RIGHT;
//       }
//     else{ 
//     MOTORY_LEFT;
//     }
//     if(zdir){
//     MOTORZ_RIGHT;
//       }
//     else{ 
//     MOTORZ_LEFT;
//     }}
  
//   if(abs(x_axis.Error) > 2){	
//     X_STEP_SET;
//     x_had_step = 1;
//     }
//   if(abs(y_axis.Error) > 2){
//     Y_STEP_SET;
//     y_had_step = 1;
//     }
//   if(abs(z_axis.Error) > 2) {
//     	Z_STEP_SET;
//       z_had_step = 1;
//       }
  
//  delay_us(1);
//  	if(x_had_step)X_STEP_CLEAR;
//   if(y_had_step)Y_STEP_CLEAR;
//   if(z_had_step)Z_STEP_CLEAR;
//     //if(x_had_step||y_had_step||z_had_step)printf("%d  %d   %d  \n", x_axis.Error,y_axis.Error,z_axis.Error );


// old_xdir = xdir; 
// old_ydir = ydir; 
// old_zdir = zdir;


//   pid_busy = 0;
// //delay_us(10);
// #endif
//   if(x_had_step||y_had_step)digitalWrite(HeartBeatLED, 1);
 // else digitalWrite(HeartBeatLED, 0);
}

// void TC0_Handler()
// {
//   sys_real_position[X_AXIS] = REG_TC0_CV0; // Value of Encoder 1
//   z_Total = REG_TC0_CV1;
//   long dummy = REG_TC0_SR1; // vital - reading this clears the interrupt flag
//                             //controllerStatus.theta = 255;
// }

void setup()
{
  //int microseconds_per_millisecond = 1000;

  noInterrupts(); // disable all interrupts

  pinMode(HeartBeatLED, OUTPUT);
  digitalWrite(HeartBeatLED, LOW);

  
  // pinMode(Encoder_ZA, INPUT_PULLUP);
  // pinMode(Encoder_ZB, INPUT_PULLUP);

  pinMode(Spindle_PWM, OUTPUT);
  digitalWrite(Spindle_PWM, 0);

  motorsDisabled();
  //
  // initialize hard-time MotorPID_Timer for servos (10ms loop)
  

  // hook up encoders
  //attachInterrupt(digitalPinToInterrupt(Encoder_XA), update_Encoder_XA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(Encoder_XB), update_Encoder_XB, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(Encoder_YA), update_Encoder_YA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(Encoder_YB), update_Encoder_YB, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(Encoder_ZA), update_Encoder_ZA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(Encoder_ZB), update_Encoder_ZB, CHANGE);
//spindle_init();

  system_init();  // Configure pinout pins and pin-change interrupt
  
  enc_sync_position();
  //sd_setup();
  //extern system_t sys;
  memset(&sys, 0, sizeof(system_t));                       // Clear all system variables
  //memset(sys.position, 0, sizeof(sys.position));           // Clear machine position.
 // memset(sys_real_position, 0, sizeof(sys_real_position)); // Clear machine position.
  sys.abort = true;                                        // Set abort to complete initialization

  interrupts(); // enable all interrupts

#if (DEBUG_COM_PORT != MACHINE_COM_PORT)
  DEBUG_COM_PORT.begin(BAUD_RATE); // setup serial for tuning mode only
  DEBUG_COM_PORT.print("DEBUG\n");
#endif

#ifndef TUNING_MODE
// Check for power-up and set system alarm if homing is enabled to force homing cycle
// by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
// startup scripts, but allows access to settings and internal commands. Only a homing
// cycle '$H' or kill alarm locks '$X' will disable the alarm.
// NOTE: The startup script will run after successful completion of the homing cycle, but
// not after disabling the alarm locks. Prevents motion startup blocks from crashing into
// things uncontrollably. Very bad.
#ifdef HOMING_INIT_LOCK
  if (bit_istrue(settings.flags, BITFLAG_HOMING_ENABLE))
  {
    sys.state = STATE_ALARM;
  }
#endif

// Force Grbl into an ALARM state upon a power-cycle or hard reset.
#ifdef FORCE_INITIALIZATION_ALARM
  sys.state = STATE_ALARM;
#endif

  // Start Grbl main loop. Processes program inputs and executes them.
  protocol_init();
#else
  motorsEnabled(); // for tuning loop
#endif
}

void loop()
{
  // digitalWrite(HeartBeatLED, healthLEDcounter++ & 0x40);
#ifndef TUNING_MODE
  // Arduinos love for this loop to run free -- so no blocking!
  if (protocol_main_loop() != 0) // in exeptions, holds, etc.. reinitialise!
    protocol_init();
#else
  tuningLoop();
#endif
 
}
