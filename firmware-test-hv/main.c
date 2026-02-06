// 100kHz square wave on PA0 using TCA0
// Target: tinyAVR 1-series (ATtiny816, ATtiny1616, etc.)

#define F_CPU 20000000UL

#include <avr/io.h>

int main(void) {
    // Set PA0 as output (TCA0 WO0)
    PORTA.DIRSET = PIN0_bm;

    // Configure TCA0 for 100kHz square wave
    // Frequency mode: toggles output at CMP0 match
    // 20MHz / 200 = 100kHz (CMP0 = 99 means period of 200 cycles)
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_FRQ_gc | TCA_SINGLE_CMP0EN_bm;
    TCA0.SINGLE.CMP0 = 99;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;

    while (1) { }
}
