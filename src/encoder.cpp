
#include "grbl.h"
#include "DueTimer.h"
void read_enc_position(void);
int32_t sys_real_position[N_AXIS]; // Real-time machine (aka home) Encoder  Position
int lastZAstate,lastZBstate;
//Arduino due Quad Decoder init
const int quadY_A = Encoder_XA; //TIOA0
const int quadY_B = Encoder_XB; //TIOB0
const unsigned int mask_quadY_A = digitalPinToBitMask(quadY_A);
const unsigned int mask_quadY_B = digitalPinToBitMask(quadY_B);

const int quadX_A = Encoder_YB; //TIOA6
const int quadX_B = Encoder_YA; //TIOB6
const unsigned int mask_quadX_A = digitalPinToBitMask(quadX_A);
const unsigned int mask_quadX_B = digitalPinToBitMask(quadX_B);

const int quadZ_A = Encoder_ZA; //TIOA8
const int quadZ_B = Encoder_ZB; //TIOB7
const unsigned int mask_quadZ_A = digitalPinToBitMask(quadZ_A);
const unsigned int mask_quadZ_B = digitalPinToBitMask(quadZ_B);
void update_Encoder_ZA(void)
{
  volatile int encState  = 0;
  delayMicroseconds(2);
  encState = (digitalRead(Encoder_ZA) << 1) + digitalRead(Encoder_ZB);
  if(lastZAstate == encState) return; // noise reject
  switch(encState)
  {
    case 0:
    case 3:
      sys_real_position[Z_AXIS]++;
      lastZAstate = encState;  
      break;
      
    case 1:
    case 2:
      sys_real_position[Z_AXIS]--;
      lastZAstate = encState;  
      break;
  } 
}

void update_Encoder_ZB(void)
{
  volatile int encState  = 0;
  delayMicroseconds(2);
  encState = (digitalRead(Encoder_ZA) << 1) + digitalRead(Encoder_ZB);
  if(lastZBstate == encState) return; // noise reject
  switch(encState)
  {
    case 0:
    case 3:
      sys_real_position[Z_AXIS]--;
      lastZBstate = encState;  
      break;
      
    case 1:
    case 2:
      sys_real_position[Z_AXIS]++;
      lastZBstate = encState;  
      break;
  } 
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
  // enable the clock (CLKEN=1) and reset the counter (SWTRG=1)
   REG_TC0_CCR0 = 5;
   REG_TC0_CCR1 = 5;
  // REG_TC0_IER1 = 0b10000000; // enable overflow interrupt TC0
  // REG_TC0_IDR1 = 0b01111111; // disable other interrupts TC0
  NVIC_EnableIRQ(TC0_IRQn); // enable TC0 interrupts
}

void activateCNT_TC2() // Y Axis
{
  REG_PMC_PCER0 = PMC_PCER0_PID27 | PMC_PCER0_PID28 | PMC_PCER0_PID29 | PMC_PCER0_PID30 | PMC_PCER0_PID31;
  REG_PMC_PCER1 = PMC_PCER1_PID32 | PMC_PCER1_PID33 | PMC_PCER1_PID34 | PMC_PCER1_PID35;
  // select XC0 as clock source and set capture mode
  REG_TC2_CMR0 = 5;
  // activate quadrature encoder and position measure mode, no filters
  REG_TC2_BMR = (1 << 9) | (1 << 8) | (1 << 12) | (60 << 20);
  // enable the clock (CLKEN=1) and reset the counter (SWTRG=1)
  // SWTRG = 1 necessary to start the clock!!
  REG_TC2_CCR0 = 5;
  REG_TC2_CCR1 = 5;
  //   REG_TC2_IER1 = 0b10000000; // enable overflow interrupt TC2
  // REG_TC2_IDR1 = 0b01111111; // disable other interrupts TC2
  NVIC_EnableIRQ(TC2_IRQn); // enable TC2 interrupts
}
void initEncoder(void)
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



  attachInterrupt(digitalPinToInterrupt(Encoder_ZA), update_Encoder_ZA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_ZB), update_Encoder_ZB, CHANGE);
  ST_SYNCH_TIMER.attachInterrupt(read_enc_position).setPeriod(1100).start();
}


void enc_sync_position()
{
  initEncoder();
#ifdef DEBUG2
  Serial.print("encA: ");
  Serial.println(REG_TC0_CV0, DEC);
  Serial.print("encB: ");
  Serial.println(REG_TC2_CV0, DEC);
#endif
  for (int i = 0; i < N_AXIS; i++)
  {
    sys_real_position[i] = sys.position[i];
  }
}

void read_enc_position(void)
{

  sys_real_position[X_AXIS] = REG_TC0_CV0; // Todo -1 quick fix just for now
  sys_real_position[Y_AXIS] = REG_TC2_CV0; // Value of Encoder 2
  
  long dummy = REG_TC0_SR0;                // vital - reading this clears the interrupt flag
  //   //int pos_Z = REG_TC2_CV0;  // Value of Encoder 3
}

static uint32_t error(int32_t ist, int32_t soll)
{
  static float ratio = 1.25; //settings.steps_per_mm/settings.enc_steps_per_mm;
  float float_error = (float)(ist * ratio) - soll;
  int error = floor(float_error);

  return error;
}