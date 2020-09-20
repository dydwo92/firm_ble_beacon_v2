#ifndef __CUSTOM_BOARD_H__
#define __CUSTOM_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define LEDS_NUMBER   2

#define LED_1          23
#define LED_2          12

#define LEDS_ACTIVE_STATE 1

#define LEDS_LIST { LED_1, LED_2 }

#define BUTTONS_NUMBER 1

#define BUTTONS_START 9
#define BUTTON_1      9
#define BUTTON_STOP   9
#define BUTTON_PULL    NRF_GPIO_PIN_NOPULL

#define BUTTONS_ACTIVE_STATE  1

#define BUTTONS_LIST  { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1
 
#endif