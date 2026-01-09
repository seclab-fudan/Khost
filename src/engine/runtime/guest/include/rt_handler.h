#ifndef RT_HANDLER_H
#define RT_HANDLER_H

#include "rt_global.h"

extern void handler_dabt_fail();

extern void handler_prefetch_fail();

extern void handler_und_fail();

#endif  // RT_HANDLER_H