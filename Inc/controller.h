#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "stm32f4xx_hal.h"
#include "tim.h"
#include "sbus.h"
#include "mti.h"

void pid_control();
void controller();

#endif
