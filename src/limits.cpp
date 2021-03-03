/*
  limits.c - code pertaining to limit-switches and performing the homing cycle
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
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

      reworked for Maslow-Due (Arduino Due) by Larry D O'Cull  Mar 2019
*/
  
#include "grbl.h"
#include "MaslowDue.h"

// Homing axis search distance multiplier. Computed by this value times the cycle travel.
#ifndef HOMING_AXIS_SEARCH_SCALAR
  #define HOMING_AXIS_SEARCH_SCALAR  1.5 // Must be > 1 to ensure limit switch will be engaged.
#endif
#ifndef HOMING_AXIS_LOCATE_SCALAR
  #define HOMING_AXIS_LOCATE_SCALAR  5.0 // Must be > 1 to ensure limit switch is cleared.
#endif

void limits_init()
{
  #ifndef PLT_V2

    #ifdef DEFAULTS_RAMPS_BOARD
      // Set as input pins
      MIN_LIMIT_DDR(0) &= ~(1<<MIN_LIMIT_BIT(0));
      MIN_LIMIT_DDR(1) &= ~(1<<MIN_LIMIT_BIT(1));
      MIN_LIMIT_DDR(2) &= ~(1<<MIN_LIMIT_BIT(2));
      MAX_LIMIT_DDR(0) &= ~(1<<MAX_LIMIT_BIT(0));
      MAX_LIMIT_DDR(1) &= ~(1<<MAX_LIMIT_BIT(1));
      MAX_LIMIT_DDR(2) &= ~(1<<MAX_LIMIT_BIT(2));

      #ifdef DISABLE_LIMIT_PIN_PULL_UP
        MIN_LIMIT_PORT(0) &= ~(1<<MIN_LIMIT_BIT(0)); // Normal low operation. Requires external pull-down.
        MIN_LIMIT_PORT(1) &= ~(1<<MIN_LIMIT_BIT(1)); // Normal low operation. Requires external pull-down.
        MIN_LIMIT_PORT(2) &= ~(1<<MIN_LIMIT_BIT(2)); // Normal low operation. Requires external pull-down.
        MAX_LIMIT_PORT(0) &= ~(1<<MAX_LIMIT_BIT(0)); // Normal low operation. Requires external pull-down.
        MAX_LIMIT_PORT(1) &= ~(1<<MAX_LIMIT_BIT(1)); // Normal low operation. Requires external pull-down.
        MAX_LIMIT_PORT(2) &= ~(1<<MAX_LIMIT_BIT(2)); // Normal low operation. Requires external pull-down.
      #else
        MIN_LIMIT_PORT(0) |= (1<<MIN_LIMIT_BIT(0));  // Enable internal pull-up resistors. Normal high operation.
        MIN_LIMIT_PORT(1) |= (1<<MIN_LIMIT_BIT(1));  // Enable internal pull-up resistors. Normal high operation.
        MIN_LIMIT_PORT(2) |= (1<<MIN_LIMIT_BIT(2));  // Enable internal pull-up resistors. Normal high operation.
        MAX_LIMIT_PORT(0) |= (1<<MAX_LIMIT_BIT(0));  // Enable internal pull-up resistors. Normal high operation.
        MAX_LIMIT_PORT(1) |= (1<<MAX_LIMIT_BIT(1));  // Enable internal pull-up resistors. Normal high operation.
        MAX_LIMIT_PORT(2) |= (1<<MAX_LIMIT_BIT(2));  // Enable internal pull-up resistors. Normal high operation.
      #endif
      #ifndef DISABLE_HW_LIMITS
        if (bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE)) {
          LIMIT_PCMSK |= LIMIT_MASK; // Enable specific pins of the Pin Change Interrupt
          PCICR |= (1 << LIMIT_INT); // Enable Pin Change Interrupt
        } else {
          limits_disable();
        }
        #ifdef ENABLE_SOFTWARE_DEBOUNCE
          MCUSR &= ~(1<<WDRF);
          WDTCSR |= (1<<WDCE) | (1<<WDE);
          WDTCSR = (1<<WDP0); // Set time-out at ~32msec.
        #endif
      #endif // DISABLE_HW_LIMITS
    #else
      LIMIT_DDR &= ~(LIMIT_MASK); // Set as input pins

      #ifdef DISABLE_LIMIT_PIN_PULL_UP
        LIMIT_PORT &= ~(LIMIT_MASK); // Normal low operation. Requires external pull-down.
      #else
        LIMIT_PORT |= (LIMIT_MASK);  // Enable internal pull-up resistors. Normal high operation.
      #endif

      if (bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE)) {
        LIMIT_PCMSK |= LIMIT_MASK; // Enable specific pins of the Pin Change Interrupt
        PCICR |= (1 << LIMIT_INT); // Enable Pin Change Interrupt
      } else {
        limits_disable();
      }
    
      #ifdef ENABLE_SOFTWARE_DEBOUNCE
        MCUSR &= ~(1<<WDRF);
        WDTCSR |= (1<<WDCE) | (1<<WDE);
        WDTCSR = (1<<WDP0); // Set time-out at ~32msec.
      #endif
    #endif // DEFAULTS_RAMPS_BOARD
  #endif
}

// Disables hard limits.
void limits_disable()
{
  #ifndef PLT_V2
    #ifdef DEFAULTS_RAMPS_BOARD
      #ifndef DISABLE_HW_LIMITS
      LIMIT_PCMSK &= ~LIMIT_MASK;  // Disable specific pins of the Pin Change Interrupt
      PCICR &= ~(1 << LIMIT_INT);  // Disable Pin Change Interrupt
      #endif
    #else
      LIMIT_PCMSK &= ~LIMIT_MASK;  // Disable specific pins of the Pin Change Interrupt
      PCICR &= ~(1 << LIMIT_INT);  // Disable Pin Change Interrupt
    #endif // DEFAULTS_RAMPS_BOARD
  #endif
}

#ifdef DEFAULTS_RAMPS_BOARD  
  static volatile uint8_t * const max_limit_pins[N_AXIS] = {&MAX_LIMIT_PIN(0), &MAX_LIMIT_PIN(1), &MAX_LIMIT_PIN(2)};
  static volatile uint8_t * const min_limit_pins[N_AXIS] = {&MIN_LIMIT_PIN(0), &MIN_LIMIT_PIN(1), &MIN_LIMIT_PIN(2)};
  static const uint8_t max_limit_bits[N_AXIS] = {MAX_LIMIT_BIT(0), MAX_LIMIT_BIT(1), MAX_LIMIT_BIT(2)};
  static const uint8_t min_limit_bits[N_AXIS] = {MIN_LIMIT_BIT(0), MIN_LIMIT_BIT(1), MIN_LIMIT_BIT(2)};
#endif // DEFAULTS_RAMPS_BOARD

// Returns limit state as a bit-wise uint8 variable. Each bit indicates an axis limit, where 
// triggered is 1 and not triggered is 0. Invert mask is applied. Axes are defined by their
// number in bit position, i.e. Z_AXIS is (1<<2) or bit 2, and Y_AXIS is (1<<1) or bit 1.
uint8_t limits_get_state()
{
  uint8_t limit_state = 0;

  #ifndef PLT_V2
    
    #ifdef DEFAULTS_RAMPS_BOARD
      uint8_t pin;
      uint8_t idx;
      #ifdef INVERT_LIMIT_PIN_MASK
        #error "INVERT_LIMIT_PIN_MASK is not implemented"
      #endif
      for (idx=0; idx<N_AXIS; idx++) {
        pin = *max_limit_pins[idx] & (1<<max_limit_bits[idx]);
        pin = !!pin;
        if (bit_isfalse(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { pin = !pin; }
        #ifdef INVERT_MAX_LIMIT_PIN_MASK
          if (bit_istrue(INVERT_MAX_LIMIT_PIN_MASK, bit(idx))) { pin = !pin; }
        #endif
        if (pin)
          limit_state |= (1 << idx);
        pin = *min_limit_pins[idx] & (1<<min_limit_bits[idx]);
        pin = !!pin;
        if (bit_isfalse(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { pin = !pin; }
        #ifdef INVERT_MIN_LIMIT_PIN_MASK
          if (bit_istrue(INVERT_MIN_LIMIT_PIN_MASK, bit(idx))) { pin = !pin; }
        #endif
        if (pin)
          limit_state |= (1 << idx);
      } 
      return(limit_state);
    #else
      uint8_t pin = (LIMIT_PIN & LIMIT_MASK);
      #ifdef INVERT_LIMIT_PIN_MASK
        pin ^= INVERT_LIMIT_PIN_MASK;
      #endif
      if (bit_isfalse(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { pin ^= LIMIT_MASK; }
      if (pin) {  
        uint8_t idx;
        for (idx=0; idx<N_AXIS; idx++) {
          if (pin & get_limit_pin_mask(idx)) { limit_state |= (1 << idx); }
        }
      }
      return(limit_state);
    #endif //DEFAULTS_RAMPS_BOARD
  #endif
}

#ifndef PLT_V2
  #ifdef DEFAULTS_RAMPS_BOARD
    #ifndef DISABLE_HW_LIMITS
      #error "HW limits are not implemented"
    #endif
  #else
  // This is the Limit Pin Change Interrupt, which handles the hard limit feature. A bouncing 
  // limit switch can cause a lot of problems, like false readings and multiple interrupt calls.
  // If a switch is triggered at all, something bad has happened and treat it as such, regardless
  // if a limit switch is being disengaged. It's impossible to reliably tell the state of a
  // bouncing pin because the Arduino microcontroller does not retain any state information when
  // detecting a pin change. If we poll the pins in the ISR, you can miss the correct reading if the 
  // switch is bouncing.
  // NOTE: Do not attach an e-stop to the limit pins, because this interrupt is disabled during
  // homing cycles and will not respond correctly. Upon user request or need, there may be a
  // special pinout for an e-stop, but it is generally recommended to just directly connect
  // your e-stop switch to the Arduino reset pin, since it is the most correct way to do this.
    #ifndef ENABLE_SOFTWARE_DEBOUNCE
      ISR(LIMIT_INT_vect) // DEFAULT: Limit pin change interrupt process. 
      {
        // Ignore limit switches if already in an alarm state or in-process of executing an alarm.
        // When in the alarm state, Grbl should have been reset or will force a reset, so any pending 
        // moves in the planner and serial buffers are all cleared and newly sent blocks will be 
        // locked out until a homing cycle or a kill lock command. Allows the user to disable the hard
        // limit setting if their limits are constantly triggering after a reset and move their axes.
        if (sys.state != STATE_ALARM) { 
          if (!(sys_rt_exec_alarm)) {
            #ifdef HARD_LIMIT_FORCE_STATE_CHECK
              // Check limit pin state. 
              if (limits_get_state()) {
                mc_reset(); // Initiate system kill.
                system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); // Indicate hard limit critical event
              }
            #else
              mc_reset(); // Initiate system kill.
              system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); // Indicate hard limit critical event
            #endif
          }
        }
      }  
    #else // OPTIONAL: Software debounce limit pin routine.
      // Upon limit pin change, enable watchdog timer to create a short delay. 
      ISR(LIMIT_INT_vect) { if (!(WDTCSR & (1<<WDIE))) { WDTCSR |= (1<<WDIE); } }
      ISR(WDT_vect) // Watchdog timer ISR
      {
        WDTCSR &= ~(1<<WDIE); // Disable watchdog timer. 
        if (sys.state != STATE_ALARM) {  // Ignore if already in alarm state. 
          if (!(sys_rt_exec_alarm)) {
            // Check limit pin state. 
            if (limits_get_state()) {
              mc_reset(); // Initiate system kill.
              system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); // Indicate hard limit critical event
            }
          }  
        }
      }
    #endif
  #endif // DEFAULTS_RAMPS_BOARD

  #ifdef DEFAULTS_RAMPS_BOARD
    static uint8_t axislock_active(uint8_t *axislock)
    {
      uint8_t res = 0;
      uint8_t idx;
  
      for (idx = 0; idx < N_AXIS; idx++)
        if (axislock[idx]) {
          res = 1;
          break;
        }
  
      return res;
    }
  #endif // DEFAULTS_RAMPS_BOARD
#endif

 
// Homes the specified cycle axes, sets the machine position, and performs a pull-off motion after
// completing. Homing is a special motion case, which involves rapid uncontrolled stops to locate
// the trigger point of the limit switches. The rapid stops are handled by a system level axis lock
// mask, which prevents the stepper algorithm from executing step pulses. Homing motions typically
// circumvent the processes for executing motions in normal operation.
// NOTE: Only the abort realtime command can interrupt this process.
// TODO: Move limit pin-specific calls to a general function for portability.
void limits_go_home(uint8_t cycle_mask)
{
  if (sys.abort) { return; } // Block if system reset has been issued.

  uint8_t n_cycle = (2*N_HOMING_LOCATE_CYCLE+1);
  uint8_t step_pin[N_AXIS];
  float target[N_AXIS];
  float max_travel = 0.0;
  uint8_t idx;


  for (idx=0; idx<N_AXIS; idx++) {  
    // Initialize step pin masks
    step_pin[idx] = get_step_pin_mask(idx);
    #ifdef COREXY    
      if ((idx==A_MOTOR)||(idx==B_MOTOR)) { step_pin[idx] = (get_step_pin_mask(X_AXIS)|get_step_pin_mask(Y_AXIS)); } 
    #endif

    if (bit_istrue(cycle_mask,bit(idx))) { 
      // Set target based on max_travel setting. Ensure homing switches engaged with search scalar.
      // NOTE: settings.max_travel[] is stored as a negative value.
      max_travel = max(max_travel,(-HOMING_AXIS_SEARCH_SCALAR)*settings.max_travel[idx]);
      
    }
  }
  //Serial.println(max_travel);
  // The active cycle axes should now be homed and machine limits have been located. By
  // default, Grbl defines machine space as all negative, as do most CNCs. Since limit switches
  // can be on either side of an axes, check and set axes machine zero appropriately. Also,
  // set up pull-off maneuver from axes limit switches that have been homed. This provides
  // some initial clearance off the switches and should also help prevent them from falsely
  // triggering when hard limits are enabled or when more than one axes shares a limit pin.
  int32_t set_axis_position =0,off_axis_position=0;
  // Set machine positions for homed limit switches. Don't update non-homed axes.
  for (idx=0; idx<N_AXIS; idx++) {
    // NOTE: settings.max_travel[] is stored as a negative value.

     
      #ifdef PLT_V2
        if (idx==X_AXIS) {

          
          //int32_t off_axis_position = system_convert_corexy_to_y_axis_steps(sys.position);
          sys.position[A_MOTOR] = set_axis_position + off_axis_position;
          sys.position[B_MOTOR] = set_axis_position - off_axis_position;
        } else if (idx==Y_AXIS) {
          //int32_t off_axis_position = system_convert_corexy_to_x_axis_steps(sys.position);
          sys.position[A_MOTOR] = off_axis_position + set_axis_position;
          sys.position[B_MOTOR] = off_axis_position - set_axis_position;
        } else {
          sys.position[idx] = set_axis_position;
        }
   //probe_get_state()
#endif
//Serial.println(sys.position[idx]);
    }
  
  sys.step_control = STEP_CONTROL_NORMAL_OP; // Return step control to normal operation.
}


// Performs a soft limit check. Called from mc_line() only. Assumes the machine has been homed,
// the workspace volume is in all negative space, and the system is in normal operation.
// NOTE: Used by jogging to limit travel within soft-limit volume.
void limits_soft_check(float *target)
{
  if (system_check_travel_limits(target)) {
    sys.soft_limit = true;
    // Force feed hold if cycle is active. All buffered blocks are guaranteed to be within
    // workspace volume so just come to a controlled stop so position is not lost. When complete
    // enter alarm mode.
    if (sys.state == STATE_CYCLE) {
      system_set_exec_state_flag(EXEC_FEED_HOLD);
      do {
        protocol_execute_realtime();
        if (sys.abort) { return; }
      } while ( sys.state != STATE_IDLE );
    }
    mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown.
    system_set_exec_alarm(EXEC_ALARM_SOFT_LIMIT); // Indicate soft limit critical event
    protocol_execute_realtime(); // Execute to enter critical event loop and system abort
    return;
  }
}
