#ifndef RGBLED_H
#define RGBLED_H
#include "Arduino.h"
#include <String.h>


class RGBLED
{
  private:
  
  uint8_t _pwr_pin;
  uint8_t _data_pin;
  uint8_t _overcurrent_pin;
  uint8_t _brightness = 88;
  uint8_t _r, _g, _b;
  
  //colorArray variables
  uint16_t _pixels;
  /* Size of colorArray (the bits to shift out) in bytes */
  uint16_t _color_array_size;
  /* Stores the global brightness value, applied to all pixels globally */
  //uint8_t globel_brightness;
  /* Stores the R, G and B values that are sent directly out to string of LEDs */
  uint8_t * color_array;

  //pin control variables for fast write
  volatile uint32_t * _port_set;
  volatile uint32_t * _port_clr;
  uint32_t _pin_mask;
  

 public:
  /* PUBLIC FIELDS */
     
  RGBLED(uint8_t pwr_pin, uint8_t data_pin,uint8_t overcurrent_pin, uint16_t pixels);
  ~RGBLED(void);
  void begin(void);
  void clear();
  void set_pwr(bool en);
  void set_colour( uint8_t  arg2, uint8_t  arg3, uint8_t  arg4);
  void set_green();
  void set_blue();
  void set_orange();
  void set_pixel_colour(uint16_t from_pixel, uint16_t to_pixel, uint8_t red, uint8_t green, uint8_t blue);
  void set_brightness(uint8_t brightness);
  void refresh_all(void);
  uint16_t pixels();
  uint8_t * pixel_colour_GRB(uint16_t pixel);
  bool is_over_current();

  static bool validate_args( char * arg1, char * arg2, char * arg3, char * arg4 );
  
};
#endif

#if F_CPU == 80000000L
    //  220 ns
    #define GRB_delay_T0H(); {asm volatile("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n ");}
    // 1000 ns
    #define GRB_delay_T0L(); {asm volatile("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n ");}
    //  800 ns
    #define GRB_delay_T1H(); {asm volatile("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n ");}
    //  350 ns
    #define GRB_delay_T1L(); {asm volatile("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n ");}
#else
    #warning F_CPU is not defined to a known value
    #define GRB_delay_T0H();
    #define GRB_delay_T0L();
    #define GRB_delay_T1H();
    #define GRB_delay_T1L();
#endif


RGBLED::RGBLED(uint8_t pwr_pin, uint8_t data_pin, uint8_t overcurrent_pin, uint16_t pixel_count) :
  _port_set(portOutputRegister(digitalPinToPort(data_pin)) + 2),
  _port_clr(portOutputRegister(digitalPinToPort(data_pin)) + 1),
  _pin_mask(digitalPinToBitMask(data_pin)),
  _data_pin(data_pin),
  _pwr_pin(pwr_pin),
  _overcurrent_pin(overcurrent_pin)

{
  if (color_array != NULL)
  {
    free(color_array);
  }
  _pixels = pixel_count;
  _color_array_size = 3 * pixel_count;
  color_array = (uint8_t *)calloc(_color_array_size, sizeof(uint8_t));

  if (color_array == NULL) //Memory Full??? don't do anything in other methods/functions
  {
    _pixels = 0;
    _color_array_size = 0;
  }
}

RGBLED::~RGBLED()
{
  if (color_array != NULL)
  {
    free(color_array);
  }
}

void RGBLED::begin()
{
  if (_color_array_size)
  {
    // let chipKIT library handle the tri-state buffer and
    // assign analog inputs as needed
    pinMode(_data_pin, OUTPUT);

    // clear the pin
    *_port_clr = _pin_mask;

    pinMode(_pwr_pin, OUTPUT);
    digitalWrite(_pwr_pin, LOW);

    pinMode(_overcurrent_pin, INPUT);
  }
}

void RGBLED::set_pwr(bool en)
{
  digitalWrite(_pwr_pin, en);
}

void RGBLED::set_brightness(uint8_t brightness){
  _brightness = brightness;
  set_pixel_colour(0,_pixels-1,_r,_g,_b);
}

void RGBLED::set_pixel_colour(uint16_t from_pixel, uint16_t to_pixel, uint8_t red, uint8_t green, uint8_t blue)
{
  if ((_color_array_size) &&  (to_pixel < _pixels) && (from_pixel <= to_pixel) )
  {
    for (uint16_t i = from_pixel; i <= to_pixel; i++)
    {
      uint8_t * p_pixel = &color_array[i * 3];
      //rearrange in GRB order
      p_pixel[0] = (green*_brightness)/255;
      p_pixel[1] = (red*_brightness)/255;
      p_pixel[2] = (blue*_brightness)/255;
    }
  }
  refresh_all();
}

uint16_t RGBLED::pixels()
{
  return _pixels;
}
uint8_t * RGBLED::pixel_colour_GRB(uint16_t pixel)
{
  if (pixel > _pixels)
  {
    return NULL;
  }
  uint8_t * p_pixel = &color_array[pixel * 3];
  return p_pixel;
    

}
void RGBLED::set_green(){
  set_colour(0,255,0);
}

void RGBLED::set_blue(){
  set_colour(0,50,200);
}

void RGBLED::set_orange(){
  set_colour(128,128,20);
}

void RGBLED::set_colour(uint8_t r, uint8_t g, uint8_t b)
{
  if (digitalRead(_pwr_pin) == LOW)
  {
    Serial.println("Power Not Enabled.");
    return;
  }
  if (digitalRead(_overcurrent_pin) == LOW)
  {
    Serial.println("Over Current Detected.");
    return;
  }

  _r = r;
  _g = g;
  _b = b;

  set_pixel_colour(0,_pixels-1,_r,_g,_b);
}

void RGBLED::clear()
{
  memset(color_array,0,_color_array_size);
}
void RGBLED::refresh_all()
{
  uint32_t interrupt_bits;
  uint8_t* p_color_array = color_array;
  uint8_t bit_select;

  /* Disable interrupts, but save current bits so we can restore them later */
  interrupt_bits = disableInterrupts();

  for (uint16_t j = 0; j < _color_array_size; j++)
  {
    bit_select = 0x80;

    while (bit_select)
    {
      if (*p_color_array & bit_select)
      {
        * _port_set = _pin_mask;
        GRB_delay_T1H();
        * _port_clr = _pin_mask;
        GRB_delay_T1L();
      }
      else
      {
        *_port_set = _pin_mask;
        GRB_delay_T0H();
        *_port_clr = _pin_mask;
        GRB_delay_T0L();
      }
      bit_select = (bit_select >> 1);
    }
    p_color_array++;
  }

  /* Restore the interrupts now */
  restoreInterrupts(interrupt_bits);

}
bool RGBLED::is_over_current()
{
  return (LOW == digitalRead(_overcurrent_pin));
}
bool RGBLED::validate_args( char * arg1, char * arg2, char * arg3, char * arg4 )
{
  //arg1
  if (0 != strcmp("H" , arg1) &&
      0 != strcmp("L" , arg1) )
  {
    Serial.println(arg1);
    Serial.println("Invalid arg1 (LEDPWR). expect H/L.");
    return false;
  }

  int r = atoi(arg2);//R
  int g = atoi(arg3);//G
  int b = atoi(arg4);//B

  //note: atoi return 0 for empty strings.

  if (r < 0 || r > 255 ||
      g < 0 || g > 255 ||
      b < 0 || b > 255 )
  {
    Serial.println("Invalid arg 2/3/4 (Colour). expect 0 to 255.");
    return false;
  }
  return true;
}
