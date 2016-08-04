#pragma once
#ifndef _BITBAND_
#define _BITBAND_

#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum<<2))
#define BIT_ADDR(addr, bitnum) *((volatile unsigned long *)(BITBAND(addr, bitnum)))

#endif