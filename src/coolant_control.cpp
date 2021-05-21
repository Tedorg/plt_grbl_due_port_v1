/*
  coolant_control.c - coolant control methods
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

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
#include "DueTimer.h"

static uint32_t ink_state_buffer[INK_STATE_BUFFER_SIZE]; // A ring buffer for motion instructions
static uint8_t ink_state_buffer_index = 0;               // Index of the block to process now
static uint32_t ink_state_buffer_sum;                    // Index of the next block to be pushed
uint16_t ink_threshold;                                  // Index of the optimally planned block
uint8_t healthLEDcounter = 0;

//const unsigned int mask_ink_state_pin = digitalPinToBitMask(INK_STATE_PIN);

void coolant_init()
{
#ifdef INK_STATE_CONTROL
  coolant_set_threshold();
  //attachInterrupt(digitalPinToInterrupt(INK_STATE_PIN), INK_STATE_TIMER_handler, CHANGE);
  sys_rt_exec_tool_state = 0;
  // if(bit_istrue())
  ink_state_buffer_init();
  sys.ink_state = ink_state_buffer_handler();
  INK_STATE_TIMER.attachInterrupt(INK_STATE_TIMER_handler).setFrequency(10);
  INK_STATE_TIMER.start();

#else
  COOLANT_FLOOD_DDR |= (1 << COOLANT_FLOOD_BIT); // Configure as output pin.
  COOLANT_MIST_DDR |= (1 << COOLANT_MIST_BIT);   // Configure as output pin.
  coolant_stop();
#endif
}

void coolant_set_threshold(void)
{
  ink_threshold = settings.ink_threshold;
}

void INK_STATE_TIMER_handler(void)
{
  digitalWrite(HeartBeatLED, healthLEDcounter++ & 0x40);
  digitalWrite(LOW_INK_INDICATIOR_LED, 0);

  coolant_get_state();
}

uint32_t ink_state_buffer_handler()
{

  ink_state_buffer_sum -= ink_state_buffer[ink_state_buffer_index];
  ink_state_buffer[ink_state_buffer_index] = analogRead(INK_STATE_PIN);
  ink_state_buffer_sum += ink_state_buffer[ink_state_buffer_index];

  uint32_t mean = (uint32_t)ink_state_buffer_sum / INK_STATE_BUFFER_SIZE;
  if (++ink_state_buffer_index >= INK_STATE_BUFFER_SIZE)
    ink_state_buffer_index = 0;
  return mean;
}

void ink_state_buffer_init()
{
  int i;
  ink_state_buffer_sum = 0;
  ink_state_buffer_index = 0;
  for (i = 0; i < INK_STATE_BUFFER_SIZE; i++)
  {
    ink_state_buffer[i] = 500;
    ink_state_buffer_sum += ink_state_buffer[i];
  }
}



// Returns current coolant output state. Overrides may alter it from programmed state.
uint8_t coolant_get_state()
{
#ifdef PLT_V2
  uint8_t cl_state = 0;
  uint8_t lim_pin_state = limits_get_state();

  sys.ink_state = ink_state_buffer_handler();
  if (bit_istrue(sys_rt_exec_tool_state, EXEC_TOOL_TIMER_ACTIVE))
  {
    if ((sys.ink_state < ink_threshold))
    {
      digitalWrite(LOW_INK_INDICATIOR_LED, 1);
      system_set_exec_tool_state_flag(EXEC_TOOL_CHANGE_M0); // Use feed hold for program pause.
      system_control_get_state();
      //protocol_execute_realtime();                          // Execute suspend.}
      // Execute suspend.
      cl_state |= EXEC_TOOL_CHANGE_M0;
    }
  }

  if (bit_istrue(sys_rt_exec_tool_state, EXEC_TOOL_COLLISION_MODE) && (sys.state == STATE_CYCLE))
  {
    if ((sys_real_position[Z_AXIS] <= -5) && bit_isfalse(lim_pin_state, bit(Z_AXIS)))
      bit_true(sys_rt_exec_tool_state, EXEC_TOOL_COLLISION_ERROR);
    if ((sys_real_position[Z_AXIS] > 1) && bit_istrue(lim_pin_state, bit(Z_AXIS)))
      bit_true(sys_rt_exec_tool_state, EXEC_TOOL_COLLISION_ERROR);
    digitalWrite(LOW_INK_INDICATIOR_LED, 1);
    cl_state |= EXEC_TOOL_COLLISION_ERROR;
  }
  return (cl_state);
#else
#ifdef INVERT_COOLANT_FLOOD_PIN
  if (bit_isfalse(COOLANT_FLOOD_PORT, (1 << COOLANT_FLOOD_BIT)))
  {
#else
  if (bit_istrue(COOLANT_FLOOD_PORT, (1 << COOLANT_FLOOD_BIT)))
  {
#endif
    cl_state |= COOLANT_STATE_FLOOD;
  }
#ifdef INVERT_COOLANT_MIST_PIN
  if (bit_isfalse(COOLANT_MIST_PORT, (1 << COOLANT_MIST_BIT)))
  {
#else
  if (bit_istrue(COOLANT_MIST_PORT, (1 << COOLANT_MIST_BIT)))
  {
#endif
    cl_state |= COOLANT_STATE_MIST;
    return COOLANT_STATE_DISABLE;
  }

#endif
}

// Directly called by coolant_init(), coolant_set_state(), and mc_reset(), which can be at
// an interrupt-level. No report flag set, but only called by routines that don't need it.
void coolant_stop()
{
#ifndef PLT_V2
#ifdef INVERT_COOLANT_FLOOD_PIN
  COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);
#else
  COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
#endif
#ifdef INVERT_COOLANT_MIST_PIN
  COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
#else
  COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
#endif
#endif
}

void coolant_calibrate()
{
}
// Main program only. Immediately sets flood coolant running state and also mist coolant,
// if enabled. Also sets a flag to report an update to a coolant state.
// Called by coolant toggle override, parking restore, parking retract, sleep mode, g-code
// parser program end, and g-code parser coolant_sync().
void coolant_set_state(uint8_t mode)
{
#ifdef PLT_V2
  if (sys.abort)
  {
    return;
  } // Block during abort.
  //Serial.print(bit_istrue(mode, COOLANT_DISABLE));
  if (bit_istrue(mode, COOLANT_MIST_ENABLE))
  {

    if (bit_isfalse(sys_rt_exec_tool_state, EXEC_TOOL_TIMER_ACTIVE))
      system_set_exec_tool_state_flag(EXEC_TOOL_TIMER_ACTIVE);
  }
  if (bit_istrue(mode, COOLANT_FLOOD_ENABLE))
  {
    if (bit_isfalse(sys_rt_exec_tool_state, EXEC_TOOL_COLLISION_MODE))
      system_set_exec_tool_state_flag(EXEC_TOOL_COLLISION_MODE);
  }

  if (mode == 0)
  {

    //ink_timer_stop();
    if (sys_rt_exec_tool_state)
      sys_rt_exec_tool_state = 0;
    //ink_timer_stop();
    if (bit_istrue(sys_rt_exec_state, EXEC_SAFETY_DOOR))
      bit_false(sys_rt_exec_state, EXEC_SAFETY_DOOR);
  };

#else
  if (mode & COOLANT_FLOOD_ENABLE)
  {
#ifdef INVERT_COOLANT_FLOOD_PIN
    COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
#else
    COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);
#endif
  }
  else
  {
#ifdef INVERT_COOLANT_FLOOD_PIN
    COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);
#else
    COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
#endif
  }

  if (mode & COOLANT_MIST_ENABLE)
  {
#ifdef INVERT_COOLANT_MIST_PIN
    COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
#else
    COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
#endif
  }
  else
  {
#ifdef INVERT_COOLANT_MIST_PIN
    COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
#else
    COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
#endif
  }
#endif
  sys.report_ovr_counter = 0; // Set to report change immediately
}

// G-code parser entry-point for setting coolant state. Forces a planner buffer sync and bails
// if an abort or check-mode is active.
void coolant_sync(uint8_t mode)
{

  if (sys.state == STATE_CHECK_MODE)
  {
    return;
  }

  protocol_buffer_synchronize(); // Ensure coolant turns on when specified in program.
  coolant_set_state(mode);
}
