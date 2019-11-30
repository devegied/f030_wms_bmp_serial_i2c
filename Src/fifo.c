/************************************************************************
Copyright (c) 2011, Nic McDonald
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met: 
1. Redistributions of source code must retain the above copyright 
   notice, this list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above 
   copyright notice, this list of conditions and the following 
   disclaimer in the documentation and/or other materials provided 
   with the distribution. 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE 
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*************************************************************************
 Information:
   File Name  :  fifo.c
   Author(s)  :  Nic McDonald
   Hardware   :  Any
   Purpose    :  First In First Out Buffer
*************************************************************************
 Modification History:
   Revision   Date         Author    Description of Revision
   1.00       05/30/2011   NGM       initial
*************************************************************************
 Theory of Operation:
   This FIFO implementation provides a memory safe 'First In First Out'
   circular buffer.  If the operating conditions of a FIFO causes it
   to 'underflow' or 'overflow' the FIFO will not corrupt memory other
   than its own data buffer.  However, memory accesses into the buffer
   will be invalid.  If a FIFO 'underflows' or 'overflows', it should
   be re-initialized or cleared.
   Example Usage:
      volatile uint8_t fifo_buf[128];
      FIFO fifo;
      fifo_init(&fifo, 128, fifo_buf);
************************************************************************/
#include "fifo.h"

void fifo_init(FIFO* f, uint8_t size, uint8_t* data) {
    f->size     = size;
    f->data     = data;
    f->status   = FIFO_UNDERFLOW;
    f->in = f->out = 0U;
}
uint8_t fifo_isFull(FIFO* f) {
    return (f->status == FIFO_OVERFLOW);
}
uint8_t fifo_isEmpty(FIFO* f) {
    //return (f->in == f->out);
    return (f->status == FIFO_UNDERFLOW);
}
uint8_t fifo_get(FIFO* f) {
    uint8_t c;
    if (f->status != FIFO_UNDERFLOW) {
        c = f->data[f->out];
        //f->data[f->out] = 0;//MOD+ zero out unused buffer space
        if(++f->out==f->size)
            f->out = 0U;
        //f->out = (f->out+1) % f->size;
        f->status = (f->out == f->in)?FIFO_UNDERFLOW:FIFO_GOOD;
        return c;
    } else {
        return 0U;
    }
}
uint8_t fifo_put(FIFO* f, uint8_t c) {
    if (f->status != FIFO_OVERFLOW){
        f->data[f->in] = c;
        //f->in = (f->in+1) % f->size;
        if(++f->in==f->size)
            f->in = 0U;
        f->status = (f->in == f->out)?FIFO_OVERFLOW:FIFO_GOOD;
        return 1U;
    }else{
        return 0U;
    }
}
uint8_t fifo_peek(FIFO* f) {
    return f->data[f->out];
}
uint8_t fifo_available(FIFO* f) {
    //return f->used;
    int8_t r = f->in - f->out;
    return r >= 0 ? r : r + f->size;
}
void fifo_clear(FIFO* f) {
    f->status = FIFO_UNDERFLOW;
    f->in = f->out = 0U;
}
FIFO_StatusTypeDef fifo_status(FIFO* f) {
    return f->status;
}