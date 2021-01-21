/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2017 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

    reworked for Maslow-Due (Arduino Due) by Larry D O'Cull  Mar 22, 2019
*/

#include "grbl.h"

#ifdef PLT_V2
  #include "MaslowDue.h"
  #include "DueTimer.h"

  uint16_t current_pwm;
  int spindle_running = 0;

  void serialScanner_handler(void);
  void Spindle_SPINDLE_TIMER_handler(void);

#endif

static float pwm_gradient; // Precalulated value to speed up rpm to PWM conversions.


void spindle_init()
{    
 //Timer8.attachInterrupt(Spindle_SPINDLE_TIMER_handler).setPeriod(Spindle_PERIOD).start();
  //no spindel on a plotter
}

#ifdef PLT_V2

void Spindle_SPINDLE_TIMER_handler(void)
{
   //no spindel on a plotter
 //printf("test");
 //serialScanner_handler(); // borrow this timer for serial preprocessing if available
}
#endif

uint8_t spindle_get_state()
{
 
    return  spindle_running;
 
}


// Disables the spindle and sets PWM output to zero when PWM variable spindle speed is enabled.
// Called by various main program and ISR routines. Keep routine small, fast, and efficient.
// Called by spindle_init(), spindle_set_speed(), spindle_set_state(), and mc_reset().
void spindle_stop()
{
 //no spindel on a plotter
}


// Sets spindle speed PWM output and enable pin, if configured. Called by spindle_set_state()
// and stepper ISR. Keep routine small and efficient.
void spindle_set_speed(uint16_t pwm_value)
{
 //no spindel on a plotter
}



  // Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
  uint16_t spindle_compute_pwm_value(float rpm) // Mega2560 PWM register is 16-bit.
  {

	return 0;
  }



// Immediately sets spindle running state with direction and spindle rpm via PWM, if enabled.
// Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
// sleep, and spindle stop override.
void spindle_set_state(uint8_t state, float rpm)
{
 
  
  sys.report_ovr_counter = 0; // Set to report change immediately
}


// G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails 
// if an abort or check-mode is active.
void spindle_sync(uint8_t state, float rpm)
{
  //no spindel on a plotter
}
