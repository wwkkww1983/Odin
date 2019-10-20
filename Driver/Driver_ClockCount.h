#ifndef __DRIVE_CLOCKCOUNT_H
#define __DRIVE_CLOCKCOUNT_H

#include "bsp.h"

#define CLOCKCOUNT_PRESCALER 90-1
#define CLOCKCOUNT_PERIOD    1000000-1       // 1s ÷–∂œ“ª¥Œ

#define CLOCKCOUNT_PRE       2
#define CLOCKCOUNT_SUB       0

void clockCountInit(void);

#endif
