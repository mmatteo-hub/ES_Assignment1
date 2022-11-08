/*
 * File:   main.c
 * Author: ettor
 *
 * Created on 27 settembre 2022, 11.16
 */

// DSPIC30F4011 Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FPR = XT                 // Primary Oscillator Mode (XT)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#include "my_timer_lib.h"
#include "my_print_lib.h"
#include "my_circular_buffer_lib.h"
#include <stdio.h>

// Definition of timers.
#define TIMER1 1 
#define TIMER2 2
#define TIMER3 3 
#define TIMER4 4 
#define TIMER5 5

int seconds = 0;

circular_buffer * buffer_ptr;

void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt() {
    // UART receive interrupt, triggered when buffer is 3/4 full

    IFS1bits.U2RXIF = 0; // reset interrupt flag
    
    while(U2STAbits.URXDA == 1) // there is something to read
        cb_push_back(buffer_ptr, U2RXREG); // put data in the circular buffer
}

void algorithm(){
    tmr_wait_ms(TIMER2, 7);
}

int main(void) {
    char word;
    
    U2BRG = 11; // (7372800 / 4) / (16 * 9600) ? 1 
    U2MODEbits.UARTEN = 1; // enable UART 
    U2STAbits.UTXEN = 1; // enable U1TX (must be after UARTEN)
    U2STAbits.URXISEL = 0b10;   // set interrupt when buffer is 3/4 full
    IEC1bits.U2RXIE = 1; // enable UART receiver interrupt

    SPI1CONbits.MSTEN = 1; // master mode 
    SPI1CONbits.MODE16 = 0; // 8 bit mode 
    SPI1CONbits.PPRE = 3; // primary prescaler 
    SPI1CONbits.SPRE = 6; // secondary prescaler 
    SPI1STATbits.SPIEN = 1; // enable SPI
    
    tmr_wait_ms(TIMER1, 1500);
  
    tmr_setup_period(TIMER1, 10);
    
    cb_init(buffer_ptr);
    
    while (1) {
        // remove residual data
        IEC1bits.U2RXIE = 0; // disable UART receiver interrupt
        while (U2STAbits.URXDA == 1){ // there is something to read
            int error = cb_push_back(buffer_ptr, U2RXREG); // put data in the circular buffer
            if(error == -1)
                U2TXREG = 'E';
            U2TXREG = buffer_ptr -> count + '0';
        }
        IEC1bits.U2RXIE = 1; // enable UART receiver interrupt
        
        //U2TXREG = buffer_ptr -> count + '0';
        while(buffer_ptr -> count != 0){
            cb_pop_front(buffer_ptr, &word);
            U2TXREG = word;
        }
        
        
        //while (U2STAbits.URXDA == 0); // wait until there is a char to read
        //char word = U2RXREG; // read the input char
        //U2TXREG = word;
        //displayWord(word); // print the char on the LCD

        
        tmr_wait_period(TIMER1);
    }
    
    while(1);
    return 0;
}