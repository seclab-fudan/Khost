#ifndef RT_TRIGGER_H
#define RT_TRIGGER_H

#include "rt_global.h"
#include "rt_data.h"

void trigger_reset();

void calc_next_irq_time(int irq);

#endif  // RT_TRIGGER_H