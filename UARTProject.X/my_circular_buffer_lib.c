/*
 * File:   circular_buffer.c
 * Authors: Carlone Matteo, Maragliano Matteo, Musumeci Mattia, Sani Ettore
 *
 * Created on 8 novembre 2022, 11.55
 */


#include "xc.h"
#include "my_circular_buffer_lib.h"

int cb_length()
{
    return SIZE;
}

void cb_init(volatile circular_buffer *cb)
{
    cb->count = 0;
    cb->head = 0;
    cb->tail = 0;
}

int cb_push_back(volatile circular_buffer *cb, char item)
{
    if(cb->count == SIZE)
        return -1; // no space to write
    
    cb->container[cb->head] = item;
    cb->head = (cb->head+1) % SIZE;
    cb->count++;
    return 0; // space to write
}

int cb_pop_front(volatile circular_buffer *cb, char* item)
{
    if(cb->count == 0)
        return 0; // no things to be read
    
    *item = cb->container[cb->tail];
    cb->tail = (cb->tail+1) % SIZE;
    cb->count--;
    return 1; // things to be read
}