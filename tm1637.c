
#include "tm1637.h"
#include "tm1637_config.h"
#include <string.h>
#include <stdio.h>

#if _TM1637_FREERTOS == 0
#define tm1637_delay_ms(x)  HAL_Delay(x)
#else
#include "cmsis_os.h"
#define tm1637_delay_ms(x)  osDelay(x)
#endif

#define TM1637_COMM1    0x40
#define TM1637_COMM2    0xC0
#define TM1637_COMM3    0x80

const uint8_t _tm1637_digit[] =
  {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f};
const uint8_t _tm1637_on[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const uint8_t _tm1637_off[6] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
const uint8_t fill_off[4] = {0x00, 0x00, 0x00, 0x00};
const uint8_t _tm1637_minus = 0x40;
const uint8_t _tm1637_dot = 0x80;  

//#######################################################################################################################
// SYSTEM & Delay
//#######################################################################################################################
static inline void __nop(void) {
  __asm("NOP"); 
}

//#######################################################################################################################
/**
 * @brief Delays for a specified number of microseconds in a blocking manner.
 * 
 * This function provides a delay in microseconds by executing NOP (No Operation) instructions
 * to consume time. It's a simple and effective way to introduce short delays in the program.
 * The delay mechanism is blocking, meaning it will halt the execution of subsequent code until
 * the delay period has elapsed.
 * 
 * @param delay The number of microseconds to delay.
 * TODO non blocking HAL_Delay implementation
 */
void tm1637_delay_us(uint8_t delay)
{
  while (delay > 0)
  {
    delay--;
    __nop();__nop();__nop();__nop();
  }
}

//#######################################################################################################################
// Start, Stop & Write
//#######################################################################################################################

//#######################################################################################################################
/**
 * @brief Starts the transmission process for the TM1637 display.
 * 
 * This function initiates the transmission process by setting the data pin to LOW and then
 * delaying for a specific number of microseconds.
 * 
 * @param tm1637 Pointer to a tm1637_t structure that contains the configuration information for the display.
 * @note the Transmission is a quasi I2C protocol, see datasheet
 */
void tm1637_start(tm1637_t *tm1637)
{
  HAL_GPIO_WritePin(tm1637->gpio_dat, tm1637->pin_dat, GPIO_PIN_RESET);
  tm1637_delay_us(_TM1637_BIT_DELAY);
}


//#######################################################################################################################
/**
 * @brief Generates stop condition (while CLK = HIGH, Data LOW->HIGH)
 * 
 * @param tm1637 Pointer to a tm1637_t structure that contains the configuration information for the display.
*/
void tm1637_stop(tm1637_t *tm1637)
{
  HAL_GPIO_WritePin(tm1637->gpio_dat, tm1637->pin_dat, GPIO_PIN_RESET);
  tm1637_delay_us(_TM1637_BIT_DELAY);
  HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, GPIO_PIN_SET);
  tm1637_delay_us(_TM1637_BIT_DELAY);
  HAL_GPIO_WritePin(tm1637->gpio_dat, tm1637->pin_dat, GPIO_PIN_SET);
  tm1637_delay_us(_TM1637_BIT_DELAY);
}

//#######################################################################################################################
/**
 * @brief Writes an I2C-formatted data byte. 
 * 
 * @param tm1637 Pointer to a tm1637_t structure that contains the configuration information for the display.
 * @param data The data byte to be written.
 * @return uint8_t The acknowledge status of the write operation.
*/
uint8_t tm1637_write_byte(tm1637_t *tm1637, uint8_t data)
{
  //  write 8 bit data
  for (uint8_t i = 0; i < 8; i++)
  {
    HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, GPIO_PIN_RESET);
    tm1637_delay_us(_TM1637_BIT_DELAY);
    if (data & 0x01)
      HAL_GPIO_WritePin(tm1637->gpio_dat, tm1637->pin_dat, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(tm1637->gpio_dat, tm1637->pin_dat, GPIO_PIN_RESET);
    tm1637_delay_us(_TM1637_BIT_DELAY);
    HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, GPIO_PIN_SET);
    tm1637_delay_us(_TM1637_BIT_DELAY);
    data = data >> 1;
  }
  // wait for acknowledge
  HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(tm1637->gpio_dat, tm1637->pin_dat, GPIO_PIN_SET);
  tm1637_delay_us(_TM1637_BIT_DELAY);
  HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, GPIO_PIN_SET);
  tm1637_delay_us(_TM1637_BIT_DELAY);
  uint8_t ack = HAL_GPIO_ReadPin(tm1637->gpio_dat, tm1637->pin_dat);
  if (ack == 0)
    HAL_GPIO_WritePin(tm1637->gpio_dat, tm1637->pin_dat, GPIO_PIN_RESET);
  tm1637_delay_us(_TM1637_BIT_DELAY);
  HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, GPIO_PIN_RESET);
  tm1637_delay_us(_TM1637_BIT_DELAY);
  return ack;
}

//#######################################################################################################################
/**
 * @brief Locks the TM1637 display to prevent concurrent access.
 * 
 * This function ensures that concurrent access to the TM1637 display is prevented by
 * checking if the lock flag is set to 1. If it is, the function waits for 1 millisecond
 * before checking again. This process is repeated until the lock flag is set to 0,
 * indicating that the display is ready for access.
 * 
 * @param tm1637 Pointer to a tm1637_t structure that contains the configuration information for the display.
*/
void tm1637_lock(tm1637_t *tm1637)
{
  while (tm1637->lock == 1)
    tm1637_delay_ms(1);
  tm1637->lock = 1;  
}

//#######################################################################################################################
/**
 * @brief Unlocks the TM1637 display to allow concurrent access.
 * 
 * This function sets the lock flag to 0, indicating that the display is ready for access.
 * 
 * @param tm1637 Pointer to a tm1637_t structure that contains the configuration information for the display.
*/
void tm1637_unlock(tm1637_t *tm1637)
{
  tm1637->lock = 0;  
}

//#######################################################################################################################
 /**
 * @brief Initializes the TM1637 display with specific GPIO ports and pins.
 * 
 * @param tm1637 Pointer to a tm1637_t structure that contains the configuration information for the TM1637 display.
 * @param gpio_clk Pointer to the GPIO_TypeDef structure that contains the configuration information for the clock GPIO port.
 * @param pin_clk The GPIO pin number associated with the clock line.
 * @param gpio_dat Pointer to the GPIO_TypeDef structure that contains the configuration information for the data GPIO port.
 * @param pin_dat The GPIO pin number associated with the data line.
 * @note GPIO_TypeDef is found in stm32f303xe.h
 * @note Overrides GPIO output pin configuration to Open Drain, ~~High Speed~~
 */
void tm1637_init(tm1637_t *tm1637, GPIO_TypeDef *gpio_clk, uint16_t pin_clk, GPIO_TypeDef *gpio_dat, uint16_t pin_dat)
{
  memset(tm1637, 0, sizeof(tm1637_t)); 
  //  set max brightess
  tm1637_brightness(tm1637, 7);  
  tm1637_lock(tm1637);
  //  init gpio
  tm1637->gpio_clk = gpio_clk;
  tm1637->pin_clk = pin_clk;
  tm1637->gpio_dat = gpio_dat;
  tm1637->pin_dat = pin_dat;
  GPIO_InitTypeDef g = {0};
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_NOPULL;
  // g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Pin = pin_clk;
  HAL_GPIO_Init(gpio_clk, &g);
  g.Pin = pin_dat;
  HAL_GPIO_Init(gpio_dat, &g);    
  tm1637_unlock(tm1637);
}

//#######################################################################################################################
/**
 * Set the brightness level of the TM1637 display to one of 8 levels
 * 
 * @param tm1637 Pointer to a tm1637_t structure that contains the configuration information for the display.
 * @param brightness_0_to_7 Brightness level (0 to 7) to set the display to, where 0 is the dimmest and 7 is the brightest.
 */
void tm1637_brightness(tm1637_t *tm1637, uint8_t brightness_0_to_7)
{
  tm1637_lock(tm1637);
  tm1637->brightness = (brightness_0_to_7 & 0x7) | 0x08;
  tm1637_unlock(tm1637);
}

//#######################################################################################################################
void tm1637_write_raw(tm1637_t *tm1637, const uint8_t *raw, uint8_t length, uint8_t pos)
{
  if (pos > 5)
    return;
  if (length > 6)
    length = 6;
  // write COMM1
  tm1637_start(tm1637);
  tm1637_write_byte(tm1637, TM1637_COMM1);
  tm1637_stop(tm1637);
  // write COMM2 + first digit address
  tm1637_start(tm1637);
  tm1637_write_byte(tm1637, TM1637_COMM2 + (pos & 0x03));
  // write the data bytes
  for (uint8_t k=0; k < length; k++)
    tm1637_write_byte(tm1637, raw[k]);
  tm1637_stop(tm1637);
  // write COMM3 + brightness
  tm1637_start(tm1637);
  tm1637_write_byte(tm1637, TM1637_COMM3 + tm1637->brightness);
  tm1637_stop(tm1637);
}

//#######################################################################################################################
/**
 * Alloss direct control over the segments of the TM1637 display. Each byte in the segments array
 * corresponds to a segment of the display, where the LSB is the A segment and the MSB is the DP (decimal point) segment.
 * 
 * @param tm1637 Pointer to a tm1637_t structure that contains the configuration information for the display.
 * @param segments Pointer to an array of uint8_t where each element represents the segments for one digit.
 * @param length The number of digits to write to the display. This should not exceed the number of digits supported by the display.
 * @param pos The position on the display where the first digit from the segments array will be written.
 */
void tm1637_write_segment(tm1637_t *tm1637, const uint8_t *segments, uint8_t length, uint8_t pos)
{
  tm1637_lock(tm1637);
  tm1637_write_raw(tm1637, segments, length, pos);
  tm1637_unlock(tm1637);  
}

//#######################################################################################################################
/**
 * Displays an integer on the TM1637 display starting from the specified position.
 * 
 * @param tm1637 Pointer to a tm1637_t structure that contains the configuration information for the display.
 * @param digit The integer value to be displayed.
 * @param pos The position on the display where the first digit of the integer will be written. TODO how is it indexed?
 */
void tm1637_write_int(tm1637_t *tm1637, int32_t digit, uint8_t pos)
{
  tm1637_lock(tm1637);
  char str[7];
  uint8_t buffer[6] = {0};
  snprintf(str, sizeof(str) , "%d", digit);
  for (uint8_t i=0; i < 6; i++)
  {
    if (str[i] == '-')
      buffer[i] = _tm1637_minus;
    else if((str[i] >= '0') && (str[i] <= '9'))
      buffer[i] = _tm1637_digit[str[i] - 48];
    else
    {
      buffer[i] = 0;
      break;
    }
  }
  tm1637_write_raw(tm1637, buffer, 6, pos);              
  tm1637_unlock(tm1637);  
}

//#######################################################################################################################
 /**
 * Displays a floating-point number on the TM1637 display with the specified number of digits after the decimal point, starting from the specified position.
 * 
 * @param tm1637 Pointer to a tm1637_t structure that contains the configuration information for the display.
 * @param digit The floating-point value to be displayed.
 * @param floating_digit The number of digits to be displayed after the decimal point.
 * @param pos The position on the display where the first digit of the floating-point number will be written.
 */
void tm1637_write_float(tm1637_t *tm1637, float digit, uint8_t floating_digit, uint8_t pos)
{
  tm1637_lock(tm1637);
  char str[8];
  uint8_t buffer[6] = {0};
  if (floating_digit >6)
    floating_digit = 6;
  switch (floating_digit)
  {
    case 0:
      snprintf(str, sizeof(str) , "%.0f", digit);
    break;
    case 1:
      snprintf(str, sizeof(str) , "%.1f", digit);
    break;
    case 2:
      snprintf(str, sizeof(str) , "%.2f", digit);
    break;
    case 3:
      snprintf(str, sizeof(str) , "%.3f", digit);
    break;
    case 4:
      snprintf(str, sizeof(str) , "%.4f", digit);
    break;
    case 5:
      snprintf(str, sizeof(str) , "%.5f", digit);
    break;
    case 6:
      snprintf(str, sizeof(str) , "%.6f", digit);
    break;
  } 
  if (tm1637->show_zero == false)
  {
    for (int8_t i = strlen(str) - 1; i > 0; i--)
    {
      if (str[i] == '0')
        str[i] = 0;
      else
        break;            
    }
  }
  uint8_t index = 0;  
  for (uint8_t i=0; i < 7; i++)
  {
    if (str[i] == '-')
    {
      buffer[index] = _tm1637_minus;
      index++;
    }
    else if((str[i] >= '0') && (str[i] <= '9'))
    {
      buffer[index] = _tm1637_digit[str[i] - 48];
      index++;
    }
    else if (str[i] == '.')
    {
      if (index > 0)
        buffer[index - 1] |= _tm1637_dot;      
    }
    else
    {
      buffer[index] = 0;
      break;
    }
  }
  tm1637_write_raw(tm1637, buffer, 6, pos);              
  tm1637_unlock(tm1637);  
}

/**
 * Controls whether leading zeros should be displayed or not.
 * 
 * @param tm1637 Pointer to a tm1637_t structure that contains the configuration information for the display.
 * @param enable Set to true to enable the display of leading zeros, or false to disable them.
*/
void tm1637_show_zero(tm1637_t *tm1637, bool enable)
{
  tm1637->show_zero = enable;
}

//#######################################################################################################################
 /**
 * Turns all segments on or off.
 * 
 * @param tm1637 Pointer to a tm1637_t structure that contains the configuration information for the display.
 * @param enable Set to true to turn all segments on, or false to turn them off.
**/
void tm1637_fill(tm1637_t *tm1637, bool enable)
{
	if (enable)
		tm1637_write_segment(tm1637, _tm1637_on, 6, 0);
	else
		tm1637_write_segment(tm1637, _tm1637_off, 6, 0);		
}








