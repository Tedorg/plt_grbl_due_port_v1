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
int healthLEDcounter = 0;

//Arduino due Quad Decoder init
const int quadY_A = 2;  //TIOA0
const int quadY_B = 13; //TIOB0
const unsigned int mask_quadY_A = digitalPinToBitMask(quadY_A);
const unsigned int mask_quadY_B = digitalPinToBitMask(quadY_B);

const int quadX_A = 5; //TIOA6
const int quadX_B = 4; //TIOB6
const unsigned int mask_quadX_A = digitalPinToBitMask(quadX_A);
const unsigned int mask_quadX_B = digitalPinToBitMask(quadX_B);

const int quadZ_A = 11; //TIOA8
const int quadZ_B = 10; //TIOB7
const unsigned int mask_quadZ_A = digitalPinToBitMask(quadZ_A);
const unsigned int mask_quadZ_B = digitalPinToBitMask(quadZ_B);

//

uint8_t pid_step_outbits; // The next stepping-bits to be output
uint8_t pid_dir_outbits;  // The next direction-bits to be output

struct PID_MOTION *selected_axis;

struct PID_MOTION x_axis = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
struct PID_MOTION y_axis = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
struct PID_MOTION z_axis = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#ifdef PLT_V2_ENCODER
static volatile uint8_t pid_busy;
#endif

// Declare system global variable structure
system_t sys;
int32_t sys_position[N_AXIS];                    // Real-time machine (aka home) position vector in steps.
int32_t sys_real_position[N_AXIS];      // Real-time machine (aka home) Encoder  Position
int32_t sys_probe_position[N_AXIS];              // Last probe position in machine coordinates and steps.
volatile uint8_t sys_probe_state;                // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile uint8_t sys_rt_exec_state;              // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm;              // Global realtime executor bitflag variable for setting various alarms.
volatile uint8_t sys_rt_exec_motion_override;    // Global realtime executor bitflag variable for motion-based overrides.
volatile uint8_t sys_rt_exec_accessory_override; // Global realtime executor bitflag variable for spindle/coolant overrides.


#ifdef DEBUG
volatile uint8_t sys_rt_exec_debug;
#endif

int fault_was_low = 0;
int Motors_Disabled = 0;
bool p_busy = 0;
//helper


int lastXAstate, lastXBstate, lastYAstate, lastYBstate, lastZAstate, lastZBstate;

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
void initEncoder(void);



void motorsEnabled(void)
{
  Motors_Disabled = 0;

  digitalWrite(X_ENABLE, 0); // disable the motor driver
  digitalWrite(Y_ENABLE, 0);
  digitalWrite(Z_ENABLE, 0);
}

void motorsDisabled(void)
{
  Motors_Disabled = 1;

  digitalWrite(X_ENABLE, 1); // Enable the motor driver
  digitalWrite(Y_ENABLE, 1);
  digitalWrite(Z_ENABLE, 1);

  //  DEBUG_COM_PORT.print("MOTORS ON\n");
}
static uint32_t error(int32_t ist, int32_t soll){
  static float ratio = 1.25; //settings.steps_per_mm/settings.enc_steps_per_mm;
  float float_error = (float)(ist*ratio) - soll;
  int error = floor(float_error);
  

  return error;
}

void MotorPID_Timer_handler(void) // PID interrupt service routine
{

  bool x_had_step = 0,y_had_step = 0,z_had_step =0;
  static bool xdir=0,ydir=0,zdir=0;
  static bool old_xdir=0,old_ydir=0,old_zdir=0;
  // printf("x: %li \n",REG_TC2_CV0);
  //serialScanner_handler();
  //report_util_encoder_values(REG_TC2_CV0);
  x_axis.axis_Position = sys_real_position[X_AXIS] = REG_TC0_CV0; // Todo -1 quick fix just for now
  y_axis.axis_Position = sys_real_position[Y_AXIS] = REG_TC2_CV0;      // Value of Encoder 2
  long dummy = REG_TC0_SR0;                                            // vital - reading this clears the interrupt flag
  //int pos_Z = REG_TC2_CV0;  // Value of Encoder 3
#ifdef PLT_V2_ENCODER
  if(pid_busy) return;

pid_busy = 1;
  x_axis.Error = error(sys_position[X_AXIS],x_axis.axis_Position); // current position error
  y_axis.Error =error(sys_position[Y_AXIS],y_axis.axis_Position); // current position error
  z_axis.Error = 0;//z_axis.target - z_axis.axis_Position; // current position error
                                                       // z_axis.Error = z_axis.target - z_axis.axis_Position; // current position error

  // if( (x_axis.Error == 0)){
   //printf("%d  %d   %d  \n", x_axis.Error,y_axis.Error,z_axis.Error );
  // }
  x_axis.Error > 0 ? xdir = 1 : xdir = 0;
  y_axis.Error > 0 ? ydir = 1 : ydir = 0;
  z_axis.Error > 0 ? zdir = 1 : zdir = 0;
if ( old_xdir != xdir || old_ydir != ydir || old_zdir != zdir){
 if(xdir){
    MOTORX_RIGHT;
    }
    else{ 
    MOTORX_LEFT;
    }
    if(ydir){
    MOTORY_RIGHT;
      }
    else{ 
    MOTORY_LEFT;
    }
    if(zdir){
    MOTORZ_RIGHT;
      }
    else{ 
    MOTORZ_LEFT;
    }}
  
  if(abs(x_axis.Error) > 2){	
    X_STEP_SET;
    x_had_step = 1;
    }
  if(abs(y_axis.Error) > 2){
    Y_STEP_SET;
    y_had_step = 1;
    }
  if(abs(z_axis.Error) > 2) {
    	Z_STEP_SET;
      z_had_step = 1;
      }
  
 delay_us(1);
 	if(x_had_step)X_STEP_CLEAR;
  if(y_had_step)Y_STEP_CLEAR;
  if(z_had_step)Z_STEP_CLEAR;
    //if(x_had_step||y_had_step||z_had_step)printf("%d  %d   %d  \n", x_axis.Error,y_axis.Error,z_axis.Error );


old_xdir = xdir; 
old_ydir = ydir; 
old_zdir = zdir;


  pid_busy = 0;
//delay_us(10);
#endif
  if(x_had_step||y_had_step)digitalWrite(HeartBeatLED, 1);
  else digitalWrite(HeartBeatLED, 0);
}

void activateCNT_TC0() // X Axis
{
  // activate clock for TC0
  REG_PMC_PCER0 = (1 << 27);
  // select XC0 as clock source and set capture mode
  REG_TC0_CMR0 = 5;
  // activate quadrature encoder and position measure mode, no filters
  REG_TC0_BMR = (1 << 9) | (1 << 8) | (1 << 12) | (60 << 20);
  // enable the clock (CLKEN=1) and reset the counter (SWTRG=1)
  // SWTRG = 1 necessary to start the clock!!
  REG_TC0_CCR0 = 5;
  REG_TC0_IER1 = 0b10000000; // enable overflow interrupt TC0
  REG_TC0_IDR1 = 0b01111111; // disable other interrupts TC0
  NVIC_EnableIRQ(TC0_IRQn);  // enable TC0 interrupts
}

void activateCNT_TC2() // Y Axis
{
  REG_PMC_PCER0 = PMC_PCER0_PID27 | PMC_PCER0_PID28 | PMC_PCER0_PID29 | PMC_PCER0_PID30 | PMC_PCER0_PID31;
  REG_PMC_PCER1 = PMC_PCER1_PID32 | PMC_PCER1_PID33 | PMC_PCER1_PID34 | PMC_PCER1_PID35;
  // select XC0 as clock source and set capture mode
  REG_TC2_CMR0 = 5;
  // activate quadrature encoder and position measure mode, no filters
  REG_TC2_BMR = (1 << 9) | (1 << 8) | (1 << 12) | (1 << 13);
  // enable the clock (CLKEN=1) and reset the counter (SWTRG=1)
  // SWTRG = 1 necessary to start the clock!!
  REG_TC2_CCR0 = 5;
  REG_TC2_IER1 = 0b10000000; // enable overflow interrupt TC2
  REG_TC2_IDR1 = 0b01111111; // disable other interrupts TC2
  NVIC_EnableIRQ(TC2_IRQn);  // enable TC2 interrupts
}
void initEncoder()
{
  // activate peripheral functions for quad pins Encoder-X
  REG_PIOB_PDR = mask_quadX_A;   // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quadX_A; // choose peripheral option B
  REG_PIOB_PDR = mask_quadX_B;   // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quadX_B; // choose peripheral option B

  // activate peripheral functions for quad pins Encoder-Y
  REG_PIOB_PDR = mask_quadY_A;   // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quadY_A; // choose peripheral option B
  REG_PIOB_PDR = mask_quadY_B;   // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quadY_B; // choose peripheral option B

  // activate peripheral functions for quad pins Encoder-Z
  // REG_PIOB_PDR = mask_quadZ_A;     // activate peripheral function (disables all PIO functionality)
  // REG_PIOB_ABSR |= mask_quadZ_A;   // choose peripheral option B
  // REG_PIOB_PDR = mask_quadZ_B;     // activate peripheral function (disables all PIO functionality)
  // REG_PIOB_ABSR |= mask_quadZ_B;   // choose peripheral option B

  // activate peripheral functions for quad pins first decoder
  PIO_Configure(PIOC, PIO_PERIPH_B, PIO_PB25B_TIOA0, PIO_DEFAULT);
  PIO_Configure(PIOC, PIO_PERIPH_B, PIO_PB27B_TIOB0, PIO_DEFAULT);

  // activate peripheral functions for quad pins second decoder
  PIO_Configure(PIOC, PIO_PERIPH_B, PIO_PC25B_TIOA6, PIO_DEFAULT);
  PIO_Configure(PIOC, PIO_PERIPH_B, PIO_PC26B_TIOB6, PIO_DEFAULT);

  // activate clock for TC0
  activateCNT_TC0();

  // activate clock for TC2
  activateCNT_TC2();
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

  pinMode(X_STEP, OUTPUT);
  pinMode(X_ENABLE, OUTPUT);
  pinMode(X_DIRECTION, OUTPUT);
  //pinMode(Encoder_XA, INPUT_PULLUP);
  // pinMode(Encoder_XB, INPUT_PULLUP);

  pinMode(Y_STEP, OUTPUT);
  pinMode(Y_ENABLE, OUTPUT);
  pinMode(Y_DIRECTION, OUTPUT);
  // pinMode(Encoder_YA, INPUT_PULLUP);
  // pinMode(Encoder_YB, INPUT_PULLUP);

  pinMode(Z_STEP, OUTPUT);
  pinMode(Z_ENABLE, OUTPUT);
  pinMode(Z_DIRECTION, OUTPUT);
  // pinMode(Encoder_ZA, INPUT_PULLUP);
  // pinMode(Encoder_ZB, INPUT_PULLUP);

  pinMode(Spindle_PWM, OUTPUT);
  digitalWrite(Spindle_PWM, 0);

  motorsDisabled();
  //
  // initialize hard-time MotorPID_Timer for servos (10ms loop)
  Timer5.attachInterrupt(MotorPID_Timer_handler).setPeriod(1100);
  Timer8.attachInterrupt(serialScanner_handler).setPeriod(500).start();

  // hook up encoders
  //attachInterrupt(digitalPinToInterrupt(Encoder_XA), update_Encoder_XA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(Encoder_XB), update_Encoder_XB, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(Encoder_YA), update_Encoder_YA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(Encoder_YB), update_Encoder_YB, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(Encoder_ZA), update_Encoder_ZA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(Encoder_ZB), update_Encoder_ZB, CHANGE);
//spindle_init();
  serial_init(); // Setup serial baud rate and interrupts for machine port

  settings_init(); // Load Grbl settings from EEPROM

  stepper_init(); // Configure stepper pins and interrupt timers
  system_init();  // Configure pinout pins and pin-change interrupt
  initEncoder();
  //sd_setup();
  extern system_t sys;
  memset(&sys, 0, sizeof(system_t));                       // Clear all system variables
  memset(sys_position, 0, sizeof(sys_position));           // Clear machine position.
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
#ifndef TUNING_MODE
  // Arduinos love for this loop to run free -- so no blocking!
  if (protocol_main_loop() != 0) // in exeptions, holds, etc.. reinitialise!
    protocol_init();
#else
  tuningLoop();
#endif
}
