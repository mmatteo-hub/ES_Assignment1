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

#define TIMER_FOR_BUTTON_S5 3
#define TIMER_FOR_BUTTON_S6 4

#include "my_timer_lib.h"
#include "my_print_lib.h"
#include "my_circular_buffer_lib.h"
#include "my_btn_lib.h"
#include <stdio.h>

// global variables
volatile circular_buffer buffer;
volatile long int character_counter = 0;
volatile short int flagS5ToUART = 0;
volatile short int flagS6Reset = 0;

void onBtnS5Released()
{
    flagS5ToUART = 1;
}

void onBtnS6Released()
{
    flagS6Reset = 1;
}

void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt()
{
    // UART receive interrupt, triggered when buffer is 3/4 full
    IFS1bits.U2RXIF = 0; // reset interrupt flag
    
    short int err = 0;
    
    while(U2STAbits.URXDA == 1) // there is something to read 
        // put data in the circular buffer
        if(cb_push_back(&buffer, U2RXREG) == -1) // not space to write
        {
            err = 1;
            LATBbits.LATB0 = 1; // LED D3 on
        }
    
    if(err)
        LATBbits.LATB0 = 0; // LED D3 off
}

void algorithm()
{
    tmr_wait_ms(TIMER2, 7);
}

void recUARTOverFlow() // manage the overflow for the UART in receive
{
    LATBbits.LATB1 = 1; // LED D4 on
    for(int i = 0; i < 5; i++)
        cb_push_back(&buffer, U2RXREG); // put data in the circular buffer
    
    U2STAbits.OERR = 0; // clear flag

    LATBbits.LATB1 = 0; // LED D4 off
}

int main(void)
{
    char word;
    int column_index = 0;
    cb_init(&buffer);
    int ret_val_pop = -1;
    short int err = 0;
    
    U2BRG = 11; // (7372800 / 4) / (16 * 9600) ? 1 
    U2MODEbits.UARTEN = 1; // enable UART 
    U2STAbits.UTXEN = 1; // enable U1TX (must be after UARTEN)
    U2STAbits.URXISEL = 0b10; // set interrupt when buffer is 3/4 full
    IEC1bits.U2RXIE = 1; // enable UART receiver interrupt
    
    initializeButtonS5(&onBtnS5Released);
    initializeButtonS6(&onBtnS6Released);
    
    TRISBbits.TRISB0 = 0; // set the LED D3 as output
    TRISBbits.TRISB1 = 0; // set the LED D4 as output
    
    init_SPI();
    
    tmr_wait_ms(TIMER1, 1500);
    
    refresh_second_line();

    tmr_setup_period(TIMER1, 10);

    while (1)
    {
        algorithm();
        
        // remove residual data
        IEC1bits.U2RXIE = 0; // disable UART receiver interrupt
        while (U2STAbits.URXDA == 1) // there is something to read
            // put data in the circular buffer
            if(cb_push_back(&buffer, U2RXREG) == -1) // not space to write
            {
                err = 1;
                LATBbits.LATB0 = 1; // LED D3 on
            }

        if(err)
            LATBbits.LATB0 = 0; // LED D3 off
        
        IEC1bits.U2RXIE = 1; // enable UART receiver interrupt
        
        if(U2STAbits.OERR == 1) // overflow occurred
            recUARTOverFlow();
        
        if(flagS5ToUART == 1) // write to the UART iff flag enabled
        {
            char str[5];
            sprintf(str, "%ld", character_counter);

            for(int i = 0; str[i] != '\0'; i++)
                U2TXREG = str[i];

            flagS5ToUART = 0; // reset flag to UART
        }
        
        if(flagS6Reset == 1) // clear first row and reset counter
        {
            clearFirstRow();
            clearSecondRow();
            character_counter = 0;
            refresh_second_line();
            
            flagS6Reset = 0; // reset flag to reset
        }
        
        while (SPI1STATbits.SPITBF == 1);
        SPI1BUF = 0x80 + column_index;
    
        // print to the LCD all chars in the circular buffer
        while(buffer.count != 0 && !IFS0bits.T1IF) // while the buffer count is different and the timer has not expired yet
        
        // while(buffer.count != 0)
        {
            ret_val_pop = cb_pop_front(&buffer, &word);
            if(ret_val_pop) // something read
            {
                character_counter++; // store the number of character received

                // if the end of the row has been reached, clear the first row and 
                // start writing again from the first row first column
                if(column_index == 0)
                    clearFirstRow();

                if(word == '\r' || word == '\n')
                {
                    clearFirstRow();
                    column_index = 0;
                }
                else
                {
                    while (SPI1STATbits.SPITBF == 1); // wait until not full
                    SPI1BUF = word; // write on the LCD
                    column_index = (column_index + 1) % 16;
                }
            }
        }
        update_second_line(character_counter);
        tmr_wait_period(TIMER1); // loop at 100Hz
    }
    return 0;
}