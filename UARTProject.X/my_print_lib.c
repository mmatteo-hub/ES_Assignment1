/*
 * File:   my_print_lib.c
 * Author: ettor
 *
 * Created on 3 novembre 2022, 11.15
 */

#include "xc.h"
#include "my_print_lib.h"
#include <p30F4011.h>

void displayText(char text[])
{
    init_SPI();
    
    SPI1BUF = 0x80;
    
    for(int i = 0; text[i] != '\0'; i++)
    {
        
        if (i % 16 == 0 && i > 15)
        {
            
            if ((int)(i>>4) % 2 == 0)
            {
                while (SPI1STATbits.SPITBF == 1); // wait until not full
                SPI1BUF = 0x80;
            }

            if ((int)(i>>4) % 2 != 0)
            {
                while (SPI1STATbits.SPITBF == 1); // wait until not full
                SPI1BUF = 0xC0;
            }
        }
     
        while (SPI1STATbits.SPITBF == 1); // wait until not full
        SPI1BUF = text[i];
    }
}

void init_SPI()
{
    SPI1CONbits.MSTEN = 1; // master mode 
    SPI1CONbits.MODE16 = 0; // 8 bit mode 
    SPI1CONbits.PPRE = 3; // primary prescaler 
    SPI1CONbits.SPRE = 6; // secondary prescaler 
    SPI1STATbits.SPIEN = 1; // enable SPI
}

void clearFirstRow()
{
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x80;
    for(int i = 0; i<16 ; i++)
    {
        while (SPI1STATbits.SPITBF == 1); // wait until not full
        SPI1BUF = ' ';
    }
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x80;
}

void clearSecondRow()
{
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0xC0;
    for (int i = 0; i < 16; i++)
    {
        while (SPI1STATbits.SPITBF == 1); // wait until not full
        SPI1BUF = ' ';
    }
}

void update_second_line(long int number)
{  
    // We have 5 char for writing the number, this is initialized as all blank
    // spaces so that the number is printed on the right part of the screen
    char output[5] = "     ";
    // If the number is over the max printable number, printing error
    if(number > 99999)
    {
        output[0] = 'O';
        output[1] = 'V';
        output[2] = 'R';
        output[3] = 'F';
        output[4] = 'L';
    }
    else
    {
        int i = 4;
        while(number > 0)
        {
            // Getting each digit from the number
            output[i--] = (char)((number%10)+48);
            number /= 10;
        }
    }
    
    // Wait for a possible transmission to end
    while (SPI1STATbits.SPITBF == 1);
    // Setting the cursor at the right place for printing the number
    SPI1BUF = 0xCB;
    
    // Always printing 5 characters
    for(int i = 0; i<5; i++)
    {
        // Wait for a possible transmission to end
        while (SPI1STATbits.SPITBF == 1);
        SPI1BUF = output[i];
    }
    
}

void refresh_second_line(void)
{
    while (SPI1STATbits.SPITBF == 1); // wait until not full
    SPI1BUF = 0xC0;
    
    char str[] = "Char Recv: 0";
    
    for(int i = 0; str[i] != '\0'; i++)
    {
        while (SPI1STATbits.SPITBF == 1); // wait until not full
        SPI1BUF = str[i];
    }
}