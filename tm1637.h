#ifndef _TM1637_H_
#define _TM1637_H_


/*
 *	Original Author:     Nima Askari
 *	WebSite:    https://www.github.com/NimaLTD
 *	Instagram:  https://www.instagram.com/github.NimaLTD
 *	LinkedIn:   https://www.linkedin.com/in/NimaLTD
 *	Youtube:    https://www.youtube.com/channel/UCUhY7qY1klJm1d2kulr9ckw
 *
 *  Modified by:         Spencer Maroukis (smaroukis)
 */

/* Notes
- currenlty blocking with nops
*/

/*
 * Version:	1.1.2
 *
 * History:
 *
 * (1.1.2):	Add Fill segments
 * (1.1.1): Add tm1637_show_zero(), show or hide lower zero in tm1637_show_float() function.
 * (1.1.0): Add tm1637_write_int() and tm1637_write_float() functions.
 * (1.0.0): First release.
 * 
 * */
 
#ifdef __cplusplus
extern "C" {
#endif
  
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

//####################################################################################################################

typedef struct
{
  uint8_t               lock;
  uint8_t               brightness;
  bool                  show_zero;    
  GPIO_TypeDef          *gpio_clk;
  GPIO_TypeDef          *gpio_dat;
  uint16_t              pin_clk;   
  uint16_t              pin_dat;  
  
}tm1637_t;

//####################################################################################################################
// Configurations
//####################################################################################################################

void tm1637_init(tm1637_t *tm1637, GPIO_TypeDef *gpio_clk, uint16_t pin_clk, GPIO_TypeDef *gpio_dat, uint16_t pin_dat);

//####################################################################################################################
// Segment and Digit  Control
//####################################################################################################################

void tm1637_brightness(tm1637_t *tm1637, uint8_t brightness_0_to_7);

void tm1637_write_segment(tm1637_t *tm1637, const uint8_t *segments, uint8_t length, uint8_t pos);

void tm1637_write_int(tm1637_t *tm1637, int32_t digit, uint8_t pos);

void tm1637_write_float(tm1637_t *tm1637, float digit, uint8_t floating_digit, uint8_t pos);

//####################################################################################################################
// Full Display Control
//####################################################################################################################

void tm1637_show_zero(tm1637_t *tm1637, bool enable);

void tm1637_fill(tm1637_t *tm1637, bool enable);

//####################################################################################################################

#ifdef __cplusplus
}
#endif

#endif
