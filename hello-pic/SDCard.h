/*
 * Copyright (c) 2023 @hanyazou
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef __SDCARD_H__
#define __SDCARD_H__

#define SDCARD_SUCCESS          0
#define SDCARD_TIMEOUT          1
#define SDCARD_NOTSUPPORTED     2
#define SDCARD_BADRESPONSE      3
#define SDCARD_CRC_ERROR        4

#define SDCARD_R1_IDLE_STATE    0x01
#define SDCARD_R1_ERASE_RESET   0x02
#define SDCARD_R1_ILLIGAL_CMD   0x04
#define SDCARD_R1_CRC_ERR       0x08
#define SDCARD_R1_ERASE_SEQ_ERR 0x10
#define SDCARD_R1_ADDR_ERR      0x20
#define SDCARD_R1_PARAM_ERR     0x40

int SDCard_init(uint16_t initial_clock_delay, uint16_t clock_delay, uint16_t timeout);
int SDCard_command(uint8_t command, uint32_t argument, void *response, int length);
int SDCard_read512(uint32_t addr, void *response);
uint8_t SDCard_crc(void *buf, int count);
uint16_t SDCard_crc16(void *buf, int count);


#endif  // __SDCARD_H__