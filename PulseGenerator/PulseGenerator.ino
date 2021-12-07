/*
  * PulseGenerator.ino
  * Leonardo Lisa
  * 07/12/2021
  * leonardo.lisa@studenti.unimi.it
  * 
  * For Atmega328PB (Arduino UNO)
  *
  */

#define F_CPU 16000000
#define PIN_1 0b00000100 // GPIO 2
#define PIN_2 0b10000000 // GPIO 7

#define LOOPS_PER_MS (((F_CPU/1000)-(1+1+2+2))/(2+2)) // accounting for the overhead of 6 (1+1+2+2) cycles per ms and the 4 (2+2) cycles per inner loop iteration

static inline void ms_spin(unsigned short ms);

void setup() {
  // disable interrupt
  cli();
  DDRD |= (PIN_1) | (PIN_2);
}

void loop() {
  PORTD |= PIN_1;

  // delay i uS
  for (uint32_t i = 0; i < 10; i++)
  {
    // delay 1uS
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
  }
  
  // delay 62.5 uS
  asm("nop");
  // delay 62.5 uS
  asm("nop");
  
  PORTD |= PIN_2;
  ms_spin(500);
  PORTD &= ~PIN_1 & ~PIN_2;
  ms_spin(1500);
}

static inline void ms_spin(unsigned short ms) {
    if (ms) {
        unsigned short dummy;
        __asm__ __volatile__ (
            "ms_spin_outer_loop_%=:                \n"

            "    ldi %A[loopcnt], lo8(%[loops])    \n"
            "    ldi %B[loopcnt], hi8(%[loops])    \n"

            "ms_spin_inner_loop_%=:                \n"

            "    sbiw %A[loopcnt], 1               \n"
            "    brne ms_spin_inner_loop_%=        \n"

            "    sbiw %A[ms], 1                    \n"
            "    brne ms_spin_outer_loop_%=        \n"

            :  [ms] "+w" (ms),
               [loopcnt] "=&w" (dummy)
            :  [loops] "i" (LOOPS_PER_MS)
            :  // none.
        );
    }
}
