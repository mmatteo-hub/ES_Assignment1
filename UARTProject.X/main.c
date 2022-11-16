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

// This is called when the UART buffer is 3/4 full
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt()
{
    // Always reset the interrupt flags
    IFS1bits.U2RXIF = 0;
    // Handle the reading oft eh buffer
    handleUARTReading();
}

// Handles the reading of the UART periferal and related errors
void handleUARTReading()
{
    // Check if there is something to read from UART
    while(U2STAbits.URXDA == 1)
        if(cb_push_back(&buffer, U2RXREG) == -1)
            LATBbits.LATB0 = 1;
            
    // If there is a overflow in the circular buffer, the led D3 is enabled
    // and then disabled when there is nothing else do read.
    // It is implicit that the data is disgarded.
    LATBbits.LATB0 = 0;
}

// Handles the overflow of the UART if occurred
void handleUARTOverflow()
{
    // Overflow did not occur, do nothing
    if(U2STAbits.OERR == 0)
        return;

    // If there is a overflow in the UART buffer, the led D4 is enabled
    LATBbits.LATB1 = 1;
    // Clearing the UART buffer by storing the available data
    handleUARTReading();
    // Turning off led D4
    LATBbits.LATB1 = 0;

    // Clearing the UART overflow flag
    U2STAbits.OERR = 0;
}

void algorithm()
{
    tmr_wait_ms(TIMER2, 7);
}

int main(void)
{
    char word;
    int column_index = 0;
    
    U2BRG = 11; // (7372800 / 4) / (16 * 9600) ? 1 
    U2MODEbits.UARTEN = 1; // enable UART 
    U2STAbits.UTXEN = 1; // enable U1TX (must be after UARTEN)
    U2STAbits.URXISEL = 0b10; // set interrupt when buffer is 3/4 full
    IEC1bits.U2RXIE = 1; // enable UART receiver interrupt
    
    TRISBbits.TRISB0 = 0; // set the LED D3 as output
    TRISBbits.TRISB1 = 0; // set the LED D4 as output
    
    cb_init(&buffer);
    init_SPI();    
    init_btn_s5(&onBtnS5Released);
    init_btn_s6(&onBtnS6Released);
    
    tmr_wait_ms(TIMER1, 1500);

    tmr_setup_period(TIMER1, 10);

    refresh_second_line();
    while (1)
    {
        algorithm();
        
        // Temporarely disable the UART interrupt to read data
        // This does not cause problems if data arrives now since we are empting the buffer
        IEC1bits.U2RXIE = 0;
        // Handle the reading oft eh buffer
        handleUARTReading();
        // Enable UART interrupt again
        IEC1bits.U2RXIE = 1;
        
        // Check if there was an overflow in the UART buffer
        recUARTOverFlow();
        
        // If the button is pressed, write the chars back to the UART
        if(flagS5ToUART == 1) 
        {
            char str[5];
            sprintf(str, "%ld", character_counter);

            for(int i = 0; str[i] != '\0'; i++)
                U2TXREG = str[i];

            // Resetting button flag
            flagS5ToUART = 0;
        }
        
        // If the button is pressed, clear the first row and reset the counter
        if(flagS6Reset == 1)
        {
            clearFirstRow();
            clearSecondRow();
            refresh_second_line();
            
            // Resetting button flag
            flagS6Reset = 0; // reset flag to reset
        }
        
        while (SPI1STATbits.SPITBF == 1);
        SPI1BUF = 0x80 + column_index;
    
        // Printing to the LCD all chars in the circular buffer
        while(buffer.count != 0 && !IFS0bits.T1IF)
        {
            // If there is nothing to read, exit
            if(!cb_pop_front(&buffer, &word))
                break; // something read

            // Store the number of characters written on the LCD
            character_counter++;

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
                // Wait for a possible ongoing transmission
                while (SPI1STATbits.SPITBF == 1);
                SPI1BUF = word; // write on the LCD
                column_index = (column_index + 1) % 16;
            }
        }

        update_second_line(character_counter);

        // Looping at 100HZ
        tmr_wait_period(TIMER1);
    }
    return 0;
}