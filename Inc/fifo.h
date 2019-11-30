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
   File Name  :  fifo.h
   Author(s)  :  Nic McDonald
   Hardware   :  Any
   Purpose    :  First In First Out Buffer
*************************************************************************
 Modification History:
   Revision   Date         Author    Description of Revision
   1.00       05/30/2011   NGM       initial
*************************************************************************/
#ifndef _FIFO_H_
#define _FIFO_H_

/* includes */
#include <stdint.h>
/* typedefs */
typedef enum {
  FIFO_GOOD       = 0x00U,
  FIFO_OVERFLOW   = 0x01U,
  FIFO_UNDERFLOW  = 0x02U
} FIFO_StatusTypeDef;
typedef struct {
    volatile uint8_t* data;
    volatile uint8_t size;
    volatile uint8_t in;
    volatile uint8_t out;
    volatile FIFO_StatusTypeDef status;
} FIFO;
/* functions */
void    fifo_init(FIFO* f, uint8_t size, uint8_t* data);
uint8_t fifo_isFull(FIFO* f);
uint8_t fifo_isEmpty(FIFO* f);
uint8_t fifo_get(FIFO* f);
uint8_t fifo_put(FIFO* f, uint8_t c);
uint8_t fifo_peek(FIFO* f);
uint8_t fifo_available(FIFO* f);
void    fifo_clear(FIFO* f);
FIFO_StatusTypeDef fifo_status(FIFO* f);
#endif // _FIFO_H_