#ifndef UTILITY_H_
#define UTILITY_H_

#include <stdint.h>

//***************************************************
uint32_t createMask(uint8_t lo, uint8_t hi) {
    uint32_t r = 0;
    for (unsigned i = lo; i <= hi; i++)
        r |= 1 << i;
    return r;
}

uint32_t bit32Extract(uint32_t dword, uint8_t displ, uint8_t lth) {
    uint32_t r = (dword & createMask(displ, (displ + lth - 1))) >> displ;
    return r;
}

//***************************************************
// Mask then AND the shifted bits, then OR them to the payload
// uint32_t bit32Pack(uint32_t dword, uint8_t displ, uint8_t lth) {
//     uint32_t dw_and_mask = (dword << displ) & (createMask(displ, displ + lth - 1));
//     //fr_payload |= dw_and_mask;
//     return dw_and_mask;
// }


#endif