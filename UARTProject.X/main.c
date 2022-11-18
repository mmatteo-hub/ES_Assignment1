/*
 * File:   main.c
 * Authors: Carlone Matteo, Maragliano Matteo, Musumeci Mattia, Sani Ettore
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
volatile unsigned int character_counter = 0;
volatile short int flagS5ToUART = 0;
volatile short int flagS6Reset = 0;
volatile short int flagCounterOverflow = 0;

void onBtnS5Released()
{
    flagS5ToUART = 1;
}

void onBtnS6Released()
{
    flagS6Reset = 1;
}

// Handles the reading of the UART periferal and related errors
void handleUARTReading()
{
    // Check if there is something to read from UART
    while(U2STAbits.URXDA == 1)
        // Put the data in the circular buffer
        if(cb_push_back(&buffer, U2RXREG) == -1)
            LATBbits.LATB0 = 1;
            
    // If there is a overflow in the circular buffer, the led D3 is enabled
    // and then disabled when there is nothing else do read.
    // It is implicit that the data is disgarded.
    LATBbits.LATB0 = 0;
}

// This is triggered when the UART buffer is 3/4 full
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt()
{
    // Reset the interrupt flag
    IFS1bits.U2RXIF = 0;
    // Handle the reading of the buffer
    handleUARTReading();
}

// Handle the overflow of the UART
void handleUARTOverflow()
{
    // Overflow did not occur, do nothing
    if(U2STAbits.OERR == 0)
        return;
    
    // If there is a overflow in the UART buffer, the led D4 is enabled
    LATBbits.LATB1 = 1;
    // Clear the UART buffer by storing all the available data
    handleUARTReading();
    // Turn off led D4
    LATBbits.LATB1 = 0;
    // Clear the UART overflow flag
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
    char counter_str[6] = "     \0";
    
    TRISBbits.TRISB0 = 0;          // set the LED D3 as output
    TRISBbits.TRISB1 = 0;          // set the LED D4 as output
    
    cb_init(&buffer);              // init the circular buffer structure
    init_btn_s5(&onBtnS5Released); // init the button S5 handler
    init_btn_s6(&onBtnS6Released); // init the button S6 handler
    init_uart();                   // init the UART
    init_spi();                    // init the SPI
    tmr_wait_ms(TIMER1, 1500);     // wait 1.5s to start the SPI correctly

    tmr_setup_period(TIMER1, 10);  // main timer to syncronize the loop

    lcd_write(16, "Char Recv: ");  // init the SPI screen
    
    while (1)
    {
        algorithm();
        
        // Temporarely disable the UART interrupt to read data
        // This does not cause problems if data arrives now since we are empting the buffer
        IEC1bits.U2RXIE = 0;
        // Handle the reading of the buffer
        handleUARTReading();
        // Enable UART interrupt again
        IEC1bits.U2RXIE = 1;
        
        // Check if there was an overflow in the UART buffer
        handleUARTOverflow();
        
        // If the button S5 is pressed, write the chars back to the UART
        if(flagS5ToUART == 1)
        {
            // convert efficiently the character counter to string, handling the overflow
            charcounter_to_str(character_counter, flagCounterOverflow, counter_str);
            uart_write(counter_str);
        
            // Reset the S5 button flag
            flagS5ToUART = 0;
        }
        
        // If the button S6 is pressed, clear the first row and reset the counter
        if(flagS6Reset == 1)
        {
            character_counter = 0;
            // convert efficiently the character counter to string, handling the overflow
            charcounter_to_str(character_counter, flagCounterOverflow, counter_str);
            
            lcd_clear(0, 16);
            lcd_write(27, counter_str);
            column_index = 0;
            // Reset the counter overflow
            flagCounterOverflow = 0;
            
            // Reset the S6 button flag
            flagS6Reset = 0;
        }
        
        // update the second line of the LCD
        charcounter_to_str(character_counter, flagCounterOverflow, counter_str);
        lcd_write(27, counter_str);
        
        lcd_move_cursor(column_index);
    
        // Print to the LCD all chars in the circular buffer until the timer expires
        // or the buffer is empty
        while(buffer.count != 0 && !IFS0bits.T1IF)
        {
            // If there is nothing to read, exit, otherwise read one char
            if(!cb_pop_front(&buffer, &word))
                break; // something read

            // Increment the number of characters written on the LCD
            character_counter++;
            // Check on the overflow of the counter
            if(character_counter >= OVERFLOW_UNSIGNED_INT)
                flagCounterOverflow = 1;

            // If the end of the row has been reached, clear the first row and 
            // start writing again from the first row first column
            if(column_index == 0)
            {
                lcd_clear(0, 16);
                lcd_move_cursor(0);
            }
            
            // Whenever a CR ?\r? or LF ?\n? character is received, clear the 
            // first row
            if(word == '\r' || word == '\n')
            {
                lcd_clear(0, 16);
                column_index = 0;
            }
            else
            {
                // Wait for a possible ongoing transmission
                while (SPI1STATbits.SPITBF == 1);
                // Write on the LCD
                SPI1BUF = word;
                // Increment the column index
                column_index = (column_index + 1) % 16;
            }
        } // end while
        // Syncronize with the 100HZ frequency
        tmr_wait_period(TIMER1);
    } // end while
    return 0;
}