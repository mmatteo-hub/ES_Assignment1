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

// global variables
volatile circular_buffer buffer;
volatile long int character_counter = 0;
volatile short int flagS5ToUART = 0;
volatile short int flagS6Reset = 0;

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

// button interrupt S5
void __attribute__((__interrupt__, __auto_psv__)) _INT0Interrupt()
{
    IFS0bits.INT0IF = 0;           // reset interrupt flag
    IEC0bits.INT0IE = 0;           // disable button interrupt
    tmr_setup_period(TIMER3, 30);  // setup timer 3 for mechanical bouncing
}

// timer 3 interrupt
void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt()
{
    IFS0bits.T3IF = 0; // reset interrupt flag
    
    if (PORTEbits.RE8 == 1) // if the button S5 is not pressed
        flagS5ToUART = 1;
    
    T3CONbits.TON = 0;   // stop the timer
    
    // Mind the sequence !!!!
    IFS0bits.INT0IF = 0; // reset interrupt flag of the button S5
    IEC0bits.INT0IE = 1; // enable button S5 interrupt
}

// button interrupt S6
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt()
{
    IFS1bits.INT1IF = 0;           // reset interrupt flag
    IEC1bits.INT1IE = 0;           // disable button interrupt
    tmr_setup_period(TIMER4, 30);  // setup timer 4 for mechanical bouncing
}

// timer 4 interrupt
void __attribute__((__interrupt__, __auto_psv__)) _T4Interrupt()
{
    IFS1bits.T4IF = 0; // reset interrupt flag
    
    if (PORTDbits.RD0 == 1) // if the button S6 is not pressed
        flagS6Reset = 1;
    
    T4CONbits.TON = 0;   // stop the timer
    
    // Mind the sequence !!!!
    IFS1bits.INT1IF = 0; // reset interrupt flag of the button S6
    IEC1bits.INT1IE = 1; // enable button S6 interrupt
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
    int line_index = 0;
    cb_init(&buffer);
    int ret_val_pop = -1;
    int ret_val_push = 1;
    short int err = 0;
    
    U2BRG = 11; // (7372800 / 4) / (16 * 9600) ? 1 
    U2MODEbits.UARTEN = 1; // enable UART 
    U2STAbits.UTXEN = 1; // enable U1TX (must be after UARTEN)
    U2STAbits.URXISEL = 0b10; // set interrupt when buffer is 3/4 full
    IEC1bits.U2RXIE = 1; // enable UART receiver interrupt
    
    TRISEbits.TRISE8 = 1; // set the button S5 pin as input
    IEC0bits.INT0IE = 1; // enable button S5 interrupt
    IEC0bits.T3IE = 1; // enable timer 3 interrupt
    
    TRISDbits.TRISD0 = 1; // set the button S6 pin as input
    IEC1bits.INT1IE = 1; // enable button S6 interrupt
    IEC1bits.T4IE = 1; // enable timer 3 interrupt
    
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
            sprintf(str, "%d", character_counter);

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
        
        // print to the LCD all chars in the circular buffer
        while(buffer.count != 0 && !IFS0bits.T1IF) // while the buffer count is different and the timer has not expired yet
        {
            ret_val_pop = cb_pop_front(&buffer, &word);
            if(ret_val_pop) // something read
            {
                line_index++;
                character_counter++; // store the number of character received

                // if the end of the row has been reached, clear the first row and 
                // start writing again from the first row first column
                if(line_index == 17)
                {
                    clearFirstRow();
                    line_index = 1;
                }

                if(word == '\r' || word == '\n')
                {
                    clearFirstRow();
                    line_index = 1;
                }

                else
                {
                    while (SPI1STATbits.SPITBF == 1); // wait until not full
                    SPI1BUF = 0x80 + line_index - 1; // set the cursor to the correct index
                    while (SPI1STATbits.SPITBF == 1); // wait until not full
                    SPI1BUF = word; // write on the LCD
                }
            }
        }
        update_second_line(character_counter);
        tmr_wait_period(TIMER1); // loop at 100Hz
    }
    return 0;
}