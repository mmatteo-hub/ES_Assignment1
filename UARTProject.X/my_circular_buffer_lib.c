/*
 * File:   circular_buffer.c
 * Author: ettor
 *
 * Created on 8 novembre 2022, 11.55
 */


#include "xc.h"
#include "my_circular_buffer_lib.h"

int cb_length()
{
    return SIZE;
}

void cb_init(circular_buffer *cb)
{
    cb->count = 0;
    cb->head = 0;
    cb->tail = 0;
}

int cb_push_back(circular_buffer *cb, char item)
{
    if(cb->count == SIZE)
        return -1;
    
    cb->container[cb->head] = item;
    cb->head = (cb->head+1) % SIZE;
    cb->count++;
    return 0;
}

int cb_pop_front(circular_buffer *cb, char* item)
{
    if(cb->count == 0)
        return -1;
    
    *item = cb->container[cb->tail];
    cb->tail = (cb->tail+1) % SIZE;
    cb->count--;
    return 0;
}