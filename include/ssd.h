#ifndef SSD_H
#define SSD_H

#include <stdint.h>

void SSD_Init(void);
void SSD_DisplayValue(uint16_t value);
void SSD_Refresh(void);

#endif