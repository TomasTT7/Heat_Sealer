/*
  Arduino ProMini (5V, 16MHz)
  OLED 128x64
  Voltage Divider 1 (100.4k and 33.3k resistors, 12V->3V) - A0
  ACS712 (30A) - A1
  Voltage Divider 2 (100.5k and 33.1k resistors, 12V->3V) - A2
  Button 1 (160k resistor to GND) - D2
  Button 2 (160k resistor to GND) - D3
  Button 3 (180k resistor to GND) - D4
  IRL1404ZPbF Transistor (N-MOSFET's gate) - D13
*/


#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>

extern "C"{
  #include "fonts.h"
}

#define BLUETOOTH                                                     // uncomment to enable Bluetooth data logging

#define OLED_ADDRESS                  0x3C
#define ADCSAMPLES                    10
#define B_PRESS                       5                               // 5 * 20ms = 100ms
#define B_DEBOUNCE                    5                               // 8 * 20ms = 100ms
#define TIMER1_RES                    20                              // [ms] interrupt fires every x ms
#define COOL_DOWN                     10000                           // [ms] forced gap between successive heat sealing rounds
#define DISPLAY_RATE                  260                             // [ms] update rate of displayed data
#define ADC_V                         4.97                            // [V] ADC voltage reference - (5.01 original)
#define VD1_RATIO                     0.248                           // voltage divider 1 ratio - (0.249 original)
#define VD2_RATIO                     0.247                           // voltage divider 2 ratio - (0.248 original)
#define ACS712_SENS                   66                              // [mV/A] sensitivity
#define ACS712_OFFSET                 1.5                             // [ADC] midpoint offset
#define WIRE_RES                      0.280                           // [Ω] wire restistance at 20°C
#define WIRE_TEMP                     20.0                            // [°C] temperature of wire resistance measurement


uint16_t displayVoltage               = 0;                            // [mV] 0-65535 range (0-65.5V)
uint16_t displayCurrent               = 0;                            // [mA] 0-65535 range (0-65.5A)
uint32_t displayEnergy                = 0;                            // [mWs] 0-4294967295 range (0-9999Ws)
uint16_t displayWireTemp              = 0;                            // [°C] 0-65535 range (0-999°C)
uint16_t sealDuration                 = 10000;                        // [ms] 100-60000 range

volatile uint16_t measuredDuration    = 0;
volatile uint16_t measuredDisplay     = 0;
volatile uint8_t sampleADC            = 0;
unsigned long last_sampling           = 0;

volatile uint8_t pressButton1         = 0;
volatile uint8_t pressButton2         = 0;
volatile uint8_t pressButton3         = 0;
volatile uint8_t ButtonDebounce1      = 0;
volatile uint8_t ButtonDebounce2      = 0;
volatile uint8_t ButtonDebounce3      = 0;
volatile uint8_t chngAllowed          = 1;
volatile uint8_t inCoolDown           = 0;

volatile uint8_t update_flags         = 0;                            // Bit 0: Seal Duration
                                                                      // Bit 1: enable Cool Down
                                                                      // Bit 2: disable Cool Down
                                                                      // Bit 3: Voltage
                                                                      // Bit 4: Current
                                                                      // Bit 5: Energy
                                                                      // Bit 6: Passed Time
                                                                      // Bit 7: Wire Temperature


// Setup function ---------------------------------------------------------------------------------
void setup()
{
#ifdef BLUETOOTH

  Serial.begin(115200);

#endif /* BLUETOOTH */

  I2C_init();                                                         // Arduino <-> OLED

  OLED_init();
  OLED_clear();
  OLED_initial_screen();                                              // show initial screen

  DDRD &= 0b11100011;                                                 // D2, D3 and D4 set as INPUT (Buttons)

  DDRB |= 0b00100000;                                                 // D13 set as OUTPUT - Transistor
  PORTB &= 0b11011111;                                                // D13 set LOW

  ADC_init();

  Timer1_setup();                                                     // start a background timer
}


// Loop function ----------------------------------------------------------------------------------
void loop()
{
  /* Button 1 Press - decrease Seal Duration */
  if(pressButton1 >= B_PRESS)
  {
    if(chngAllowed)
    {
      sealDuration -= 100;
      if(sealDuration < 100) sealDuration = 100;

      update_flags |= 0b00000001;                                     // update Seal Duration
    }
    
    pressButton1 = 0;
    ButtonDebounce1 = B_DEBOUNCE;
  }

  /* Button 2 Press - increase Seal Duration */
  if(pressButton2 >= B_PRESS)
  {
    if(chngAllowed)
    {
      sealDuration += 100;
      if(sealDuration > 60000) sealDuration = 60000;

      update_flags |= 0b00000001;                                     // update Seal Duration
    }
    
    pressButton2 = 0;
    ButtonDebounce2 = B_DEBOUNCE;
  }

  /* Button 3 Press - start Heat Sealing */
  if(pressButton3 >= B_PRESS)
  {
    if(chngAllowed && !inCoolDown)                                    // START HEAT SEALING
    {
      chngAllowed = 0;
      measuredDuration = 0;
      displayEnergy = 0;
      TCNT1 = 0;                                                      // reset counter

      PORTB |= 0b00100000;                                            // D13 set HIGH (transistor's gate)
    }
    else if(!chngAllowed)                                             // MANUAL ABORT
    {
      PORTB &= 0b11011111;                                            // D13 set LOW (transistor's gate)
      
      chngAllowed = 1;
      measuredDuration = 0;
      inCoolDown = 1;
      update_flags |= 0b00000011;                                     // update Cool Down, Seal Duration
    }
    
    pressButton3 = 0;
    ButtonDebounce3 = B_DEBOUNCE;
  }

  /* ADC */
  if(sampleADC)
  {
    uint16_t adc_acs712 = ADC_sample(1);
    uint16_t adc_voltage_1 = ADC_sample(0);
    uint16_t adc_voltage_2 = ADC_sample(2);

    uint16_t firstVoltage = (uint16_t)((float)adc_voltage_1 / 1024.0 * ADC_V / VD1_RATIO * 1000.0);
    uint16_t secondVoltage = (uint16_t)((float)adc_voltage_2 / 1024.0 * ADC_V / VD2_RATIO * 1000.0);
    
    if(!chngAllowed) displayVoltage = firstVoltage - secondVoltage;
    else displayVoltage = firstVoltage;
    
    uint16_t midpoint = (uint16_t)(ADC_V * 1000.0 / 2.0);
    displayCurrent = (uint16_t)(((float)adc_acs712 + ACS712_OFFSET) / 1024.0 * ADC_V * 1000.0);
    
    if(displayCurrent >= midpoint) displayCurrent = 0;
    else displayCurrent = (uint16_t)((uint32_t)(midpoint - displayCurrent) * 1000 / ACS712_SENS);

    uint32_t deltaEnergy = ((uint32_t)displayVoltage * (uint32_t)displayCurrent) / 1000 * (millis() - last_sampling) / 1000;
    
    if(!chngAllowed)
    {
      displayEnergy += deltaEnergy;

      if(displayCurrent > 0)
      {
        float wireRes = ((float)displayVoltage / 1000.0) / ((float)displayCurrent / 1000.0);
        float wireTemp = (WIRE_TEMP + wireRes / (WIRE_RES * 0.00017) - 5882.4);
        if(wireTemp < 0) displayWireTemp = 20;
        else displayWireTemp = (uint16_t)wireTemp;
      }
      else
      {
        displayWireTemp = 20;
      }
    }

#ifdef BLUETOOTH

    Serial.print(millis());
    Serial.print(',');
    Serial.print(firstVoltage);
    Serial.print(',');
    Serial.print(secondVoltage);
    Serial.print(',');
    Serial.print(displayCurrent);
    Serial.print(',');
    Serial.print(displayEnergy);
    Serial.print(',');
    Serial.println(displayWireTemp);

#endif /* BLUETOOTH */

    last_sampling = millis();
    sampleADC = 0;
  }

  /* Update OLED */
  if(update_flags & 0b01000000)                                       // update Passed Time
  {
    update_flags &= 0b10111111;
    OLED_update_passed_time();
  }
  
  if(update_flags & 0b00000001)                                       // update Seal Duration
  {
    update_flags &= 0b11111110;
    OLED_update_seal_duration();
  }

  if(update_flags & 0b00000010)                                       // show Cool Down graphic
  {
    update_flags &= 0b11111101;
    OLED_update_cool_down(1);
  }

  if(update_flags & 0b00000100)                                       // hide Cool Down graphic
  {
    update_flags &= 0b11111011;
    OLED_update_cool_down(0);
  }

  if(update_flags & 0b00001000)                                       // update Voltage
  {
    update_flags &= 0b11110111;
    OLED_update_voltage();
  }

  if(update_flags & 0b00010000)                                       // update Current
  {
    update_flags &= 0b11101111;
    OLED_update_current();
  }

  if(update_flags & 0b00100000)                                       // update Energy
  {
    update_flags &= 0b11011111;
    OLED_update_energy();
  }

  if(update_flags & 0b10000000)                                       // update Wire Temperature
  {
    update_flags &= 0b01111111;
    OLED_update_wire_temperature();
  }
}


// I2C functions ----------------------------------------------------------------------------------
/*
  I2C interface between Arduino and OLED display. 
*/


/*
  
*/
void I2C_init(void)
{
  TWSR = 0x00;                                                        // set the prescaler value to 1
  //TWBR = 0x48;                                                        // set the division factor for 100kHz clock signal (0x48 -> 16000000/(16+2*72*1)=100000)
  TWBR = 0x0C;                                                        // set the division factor for 400kHz clock signal (0x48 -> 16000000/(16+2*12*1)=400000)
  TWCR = (1<<TWEN);                                                   // I2C enable
}


/*

*/
void I2C_start(void)
{
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);                             // TWINT - clearing the 'job finished' flag, TWSTA - if the bus is clear become Master, TWEN - I2C enable
  while ((TWCR & (1<<TWINT)) == 0);                                   // waiting for the 'job finished' flag
}


/*

*/
void I2C_stop(void)
{
  TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);                             // TWSTO - generate a Stop condition
}


/*

*/
void I2C_write_byte(uint8_t u8data)
{
  TWDR = u8data;                                                      // fill the Data Register
  TWCR = (1<<TWINT)|(1<<TWEN);                                        // TWINT - clearing the 'job finished' flag, TWEN - I2C enable
  while ((TWCR & (1<<TWINT)) == 0);                                   // waiting for the 'job finished' flag
}


// OLED functions ---------------------------------------------------------------------------------
/*
  OLED display 128x64 pixels (I2C 400kHz max).

  Approximate transmission duration via 100kHz I2C.
    OLED_init()               27 bytes        2.4ms
    OLED_clear()              1034 bytes      93.1ms
    OLED_initial_screen()     795 bytes       71.6ms
    OLED_draw_string_8x16()   16+16*n bytes   1.4ms + 1.4ms * n
    OLED_draw_char_16x32()    96 bytes        8.6ms

  OLED responds immediatelly. No gaps between successive bytes via I2C.
*/


/*
  keywords:
    SEG (segment) = COL (column) = byte of data (bits represent 8 rows within the column)
    COM = row
    Page = 8 rows of pixels of 128 columns
    Display = 8 pages
*/
void OLED_init(void)
{
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x00);                                               // BYTE_CMD_STREAM
  I2C_write_byte(0xAE);                                               // DISPLAY_OFF
  I2C_write_byte(0xA8);                                               // SET_MUX_RATIO
  I2C_write_byte(0x3F);
  I2C_write_byte(0xD3);                                               // SET_DISPLAY_OFFSET
  I2C_write_byte(0x00);
  I2C_write_byte(0x40);                                               // SET_DISPLAY_START_LINE
  I2C_write_byte(0xA1);                                               // SET_SEGMENT_REMAP
  I2C_write_byte(0xC8);                                               // SET_COM_SCAN_MODE
  I2C_write_byte(0xDA);                                               // SET_COM_PIN_MAP
  I2C_write_byte(0x12);
  I2C_write_byte(0x81);                                               // SET_CONTRAST
  I2C_write_byte(0x7F);
  I2C_write_byte(0xA4);                                               // DISPLAY_RAM
  I2C_write_byte(0xA6);                                               // DISPLAY NORMAL
  I2C_write_byte(0xD5);                                               // SET_DISPLAY_CLK_DIV
  I2C_write_byte(0x80);
  I2C_write_byte(0x8D);                                               // SET_CHARGE_PUMP
  I2C_write_byte(0x14);
  I2C_write_byte(0xD9);                                               // SET_PRECHARGE
  I2C_write_byte(0x22);
  I2C_write_byte(0xDB);                                               // SET_VCOMH_DESELECT
  I2C_write_byte(0x30);
  I2C_write_byte(0x20);                                               // SET_MEMORY_ADDR_MODE
  I2C_write_byte(0x00);
  I2C_write_byte(0xAF);                                               // DISPLAY_ON
  I2C_stop();
}


/*

*/
void OLED_draw_string_8x16(uint8_t * string, uint8_t column, uint8_t page, uint16_t stringsize)
{
  uint8_t * stringf = string;
  
  /* First Row */
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x00);                                               // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                                               // SET_COLUMN_ADDRESS
  I2C_write_byte(column);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                                               // SET_PAGE_ADDRESS
  I2C_write_byte(page);
  I2C_write_byte(0x07);
  I2C_stop();
  
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x40);                                               // BYTE_DATA_STREAM
  
  for(uint8_t i = 0; i < stringsize; i++)
  {
    uint8_t c = *stringf - 0x20;
    uint16_t x = (uint16_t)c * 16;
    
    for(uint8_t y = 0; y < 8; y++)
    {
      I2C_write_byte(pgm_read_byte(&font8x16[x]));
      x++;
    }
    
    stringf++;
  }
  
  I2C_stop();

  /* Second Row */
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x00);                                               // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                                               // SET_COLUMN_ADDRESS
  I2C_write_byte(column);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                                               // SET_PAGE_ADDRESS
  I2C_write_byte(page+1);
  I2C_write_byte(0x07);
  I2C_stop();

  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x40);                                               // BYTE_DATA_STREAM
  
  for(uint8_t i = 0; i < stringsize; i++)
  {
    uint8_t c = *string - 0x20;
    uint16_t x = (uint16_t)c * 16 + 8;
    
    for(uint8_t y = 0; y < 8; y++)
    {
      I2C_write_byte(pgm_read_byte(&font8x16[x]));
      x++;
    }
    
    string++;
  }
  
  I2C_stop();
}


/*
  C     CHAR  OFFSET
  0     0     0
  1     1     64
  2     2     128
  3     3     192
  4     4     256
  5     5     320
  6     6     384
  7     7     448
  8     8     512
  9     9     576
  10    :     640
  11    ,     704
  12    -     768
  13    .     832
  14    /     896
  15          960

  PAGE
    for 2 rows only 0 or 4
    for 1 row 0 to 4
*/
void OLED_draw_char_16x32(uint8_t c, uint8_t column, uint8_t page)
{
  uint16_t offset = c * 64;
  
  for(uint8_t r = 0; r < 4; r++)
  {
    I2C_start();
    I2C_write_byte(OLED_ADDRESS << 1);                                // ADDRESS
    I2C_write_byte(0x00);                                             // BYTE_CMD_STREAM
    I2C_write_byte(0x21);                                             // SET_COLUMN_ADDRESS
    I2C_write_byte(column);
    I2C_write_byte(0x7F);
    I2C_write_byte(0x22);                                             // SET_PAGE_ADDRESS
    I2C_write_byte(page+r);
    I2C_write_byte(0x07);
    I2C_stop();
  
    I2C_start();
    I2C_write_byte(OLED_ADDRESS << 1);                                // ADDRESS
    I2C_write_byte(0x40);                                             // BYTE_DATA_STREAM
  
    for(uint16_t i = 0; i < 16; i++)
    {
      I2C_write_byte(pgm_read_byte(&font16x32[offset + (3 - r) + i * 4]));
    }
  
    I2C_stop();
  }
}


/*
  Example:
    [0xE0, 0xE0, 0xE0]
    -> 0b11100000, 0b11100000, 0b11100000 (a 3x3 pixel dot)
*/
void OLED_draw_chars_1x8(uint8_t * chars, uint8_t column, uint8_t page, uint8_t len)
{
  uint8_t * _ch = chars;
  
  /* Row */
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x00);                                               // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                                               // SET_COLUMN_ADDRESS
  I2C_write_byte(column);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                                               // SET_PAGE_ADDRESS
  I2C_write_byte(page);
  I2C_write_byte(0x07);
  I2C_stop();
  
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x40);                                               // BYTE_DATA_STREAM
  
  for(uint8_t i = 0; i < len; i++)
  {
    I2C_write_byte(*_ch++);
  }
  
  I2C_stop();
}


/*

*/
void OLED_clear(void)
{
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x00);                                               // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                                               // SET_COLUMN_ADDRESS
  I2C_write_byte(0x00);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                                               // SET_PAGE_ADDRESS
  I2C_write_byte(0x00);
  I2C_write_byte(0x07);
  I2C_stop();

  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x40);                                               // BYTE_DATA_STREAM
  
  for(uint16_t i = 0; i < 1024; i++)
  {
    I2C_write_byte(0x00);
  }
  
  I2C_stop();
}


/*
  Draws initial screen arrangement.
*/
void OLED_initial_screen(void)
{
  if(sealDuration > 9900) OLED_draw_char_16x32(sealDuration / 10000 % 10, 0, 0);
  else OLED_draw_char_16x32(15, 0, 0);
  
  OLED_draw_char_16x32(sealDuration / 1000 % 10, 16, 0);
  
  uint8_t dot[] = {0xE0, 0xE0, 0xE0};
  OLED_draw_chars_1x8(dot, 33, 3, 3);
  
  OLED_draw_char_16x32(sealDuration / 100 % 10, 39, 0);
  OLED_draw_string_8x16((uint8_t*)"s", 0, 4, 1);

  OLED_draw_string_8x16((uint8_t*)" ", 27, 4, 1);                     // Cool Down sign " " or "!"
  OLED_draw_string_8x16((uint8_t*)" 20^C", 44, 4, 5);                 // Wire Temperature
  OLED_draw_string_8x16((uint8_t*)" ", 93, 4, 1);                     // Cool Down sign " " or "!"

  OLED_draw_char_16x32(15, 64, 0);                                    // ' '
  OLED_draw_char_16x32(15, 80, 0);                                    // ' '
  OLED_draw_char_16x32(15, 96, 0);                                    // ' '
  OLED_draw_char_16x32(0, 112, 0);                                    // '0'
  OLED_draw_string_8x16((uint8_t*)"Ws", 113, 4, 2);

  OLED_draw_string_8x16((uint8_t*)" 0.0 V", 5, 6, 6);
  OLED_draw_string_8x16((uint8_t*)" 0.0 A", 75, 6, 6);
}


/*
  Update Seal Duration.
*/
void OLED_update_seal_duration(void)
{
  if(sealDuration > 9900) OLED_draw_char_16x32(sealDuration / 10000 % 10, 0, 0);
  else OLED_draw_char_16x32(15, 0, 0);
  OLED_draw_char_16x32(sealDuration / 1000 % 10, 16, 0);
  OLED_draw_char_16x32(sealDuration / 100 % 10, 39, 0);
}


/*
  Update Passed Time.
*/
void OLED_update_passed_time(void)
{
  if(measuredDuration > 9900) OLED_draw_char_16x32(measuredDuration / 10000 % 10, 0, 0);
  else OLED_draw_char_16x32(15, 0, 0);
  OLED_draw_char_16x32(measuredDuration / 1000 % 10, 16, 0);
  OLED_draw_char_16x32(measuredDuration / 100 % 10, 39, 0);
}


/*
  Update Energy.
*/
void OLED_update_energy(void)
{
  uint32_t _energy = 0;
  uint8_t energy[4];

  if(displayEnergy >= 9999500)
  {
    energy[0] = 9;
    energy[1] = 9;
    energy[2] = 9;
    energy[3] = 9;
  }
  else
  {
    if(displayEnergy / 100 % 10 >= 5) _energy = displayEnergy / 1000 + 1;
    else _energy = displayEnergy / 1000;

    if(_energy >= 1000) energy[0] = _energy / 1000 % 10;
    else energy[0] = 15;

    if(_energy >= 100) energy[1] = _energy / 100 % 10;
    else energy[1] = 15;

    if(_energy >= 10) energy[2] = _energy / 10 % 10;
    else energy[2] = 15;

    energy[3] = _energy % 10;
  }

  OLED_draw_char_16x32(energy[0], 64, 0);
  OLED_draw_char_16x32(energy[1], 80, 0);
  OLED_draw_char_16x32(energy[2], 96, 0);
  OLED_draw_char_16x32(energy[3], 112, 0);
}


/*
  Update Voltage.
*/
void OLED_update_voltage(void)
{
  uint8_t rnd = 0;
  uint8_t voltage[4];

  if(displayVoltage / 10 % 10 >= 5) rnd = 1;
  else rnd = 0;
  voltage[3] = (displayVoltage / 100 + rnd) % 10 + '0';
  voltage[2] = '.';
  if(voltage[3] == '0' && rnd == 1)
  {
    voltage[1] = ((displayVoltage / 1000 + 1) % 10 + '0');
    if(voltage[1] == '0') rnd = 1;
    else rnd = 0;
  }
  else
  {
    voltage[1] = displayVoltage / 1000 % 10 + '0';
    rnd = 0;
  }
  if(displayVoltage >= 10000 || rnd == 1)
  {
    voltage[0] = (displayVoltage / 10000 + rnd) % 10 + '0';
  }
  else
  {
    voltage[0] = ' ';
  }
  
  OLED_draw_string_8x16(voltage, 5, 6, 4);
}


/*
  Update Current.
*/
void OLED_update_current(void)
{
  uint8_t rnd = 0;
  uint8_t current[4];

  if(displayCurrent / 10 % 10 >= 5) rnd = 1;
  else rnd = 0;
  current[3] = (displayCurrent / 100 + rnd) % 10 + '0';
  current[2] = '.';
  if(current[3] == '0' && rnd == 1)
  {
    current[1] = ((displayCurrent / 1000 + 1) % 10 + '0');
    if(current[1] == '0') rnd = 1;
    else rnd = 0;
  }
  else
  {
    current[1] = displayCurrent / 1000 % 10 + '0';
    rnd = 0;
  }
  if(displayCurrent >= 10000 || rnd == 1)
  {
    current[0] = (displayCurrent / 10000 + rnd) % 10 + '0';
  }
  else
  {
    current[0] = ' ';
  }
  
  OLED_draw_string_8x16(current, 75, 6, 4);
}


/*
  Update Cool Down graphic.
*/
void OLED_update_cool_down(uint8_t enable)
{
  if(enable)
  {
    OLED_draw_string_8x16((uint8_t*)"!", 27, 4, 1);
    OLED_draw_string_8x16((uint8_t*)"!", 93, 4, 1);
  }
  else
  {
    OLED_draw_string_8x16((uint8_t*)" ", 27, 4, 1);
    OLED_draw_string_8x16((uint8_t*)" ", 93, 4, 1);
  }
}


/*
  Update Wire Temperature.
*/
void OLED_update_wire_temperature(void)
{
  uint8_t temperature[3];

  if(displayWireTemp >= 999)
  {
    temperature[0] = '9';
    temperature[1] = '9';
    temperature[2] = '9';
  }
  else
  {
    if(displayWireTemp >= 100) temperature[0] = displayWireTemp / 100 % 10 + '0';
    else temperature[0] = ' ';
    if(displayWireTemp >= 10) temperature[1] = displayWireTemp / 10 % 10 + '0';
    else temperature[1] = ' ';
    temperature[2] = displayWireTemp % 10 + '0';
  }
  
  OLED_draw_string_8x16(temperature, 44, 4, 3);
}


// Timer1 functions -------------------------------------------------------------------------------
/*
  Timer0 used by Arduino's core (delay(), millis(), micros()) - pins D5 and D6.
  Timer1 16-bit used by Servo library - pins D9 and D10.
  Timer2 8-bit used by Tone library - pins D3 and D11.
  
  D2 - button 1 (160k resistor to GND)
  D3 - button 2 (160k resistor to GND)
  D4 - button 3 (180k resistor to GND)
*/


/*
  Timer1 in CTC Mode running in the background providing 20ms time pulse.
*/
void Timer1_setup(void)
{
  TCCR1B = (1 << WGM12) | (1 << CS12);                                // 16MHz/256 prescaling, CLK=62.5kHz, CTC mode
  TCCR1A = 0;                                                         // CTC mode (OCR1A top)
  OCR1A = 1250;                                                       // output compare match every 20ms  
  TCNT1 = 0;                                                          // initialize counter
  TIMSK1 |= (1 << OCIE1A);                                            // enable output compare A match interrupt
  sei();
}


/*
  Timer/Counter1 Compare Match A interrupt service routine is called when TCNT1 matches OCR1A.
*/
ISR(TIMER1_COMPA_vect)
{
  TCNT1 = 0;                                                          // reset counter

  uint8_t buttons = PIND;                                             // read D2, D3 and D4
  
  /* Button 1 Press */
  if(ButtonDebounce1)
  {
    ButtonDebounce1--;
  }
  else
  {
    if(buttons & 0b00000100) pressButton1++;
    else pressButton1 = 0;
  }

  /* Button 2 Press */
  if(ButtonDebounce2)
  {
    ButtonDebounce2--;
  }
  else
  {
    if(buttons & 0b00001000) pressButton2++;
    else pressButton2 = 0;
  }

  /* Button 3 Press */
  if(ButtonDebounce3)
  {
    ButtonDebounce3--;
  }
  else
  {
    if(buttons & 0b00010000) pressButton3++;
    else pressButton3 = 0;
  }

  /* Heat Sealing Duration Measurement */
  if(!chngAllowed || inCoolDown) measuredDuration += TIMER1_RES;
  
  if(!chngAllowed && measuredDuration >= sealDuration)
  {
    PORTB &= 0b11011111;                                              // D13 set LOW (transistor's gate)
    
    chngAllowed = 1;
    measuredDuration = 0;
    inCoolDown = 1;
    update_flags |= 0b00000011;                                       // update Cool Down, Seal Duration
  }
  
  if(inCoolDown && measuredDuration >= COOL_DOWN)
  {
    measuredDuration = 0;
    inCoolDown = 0;
    update_flags |= 0b00000100;                                       // update Cool Down
  }

  /* Display */
  measuredDisplay += TIMER1_RES;

  if(measuredDisplay >= DISPLAY_RATE)
  {
    measuredDisplay = 0;
    update_flags |= 0b10111000;                                       // update Energy, Voltage, Current, Wire Temperature
  }

  if(!chngAllowed && measuredDisplay >= (DISPLAY_RATE - TIMER1_RES))
  {
    update_flags |= 0b01000000;                                       // update Passed Time
  }

  /* ADC */
  sampleADC = 1;
}


// ADC functions ----------------------------------------------------------------------------------
/*
  (ATMEGA328P)
  REFS1 REFS0
  0     0     AREF, Internal Vref turned off
  0     1     AVCC with external capacitor at AREF pin
  1     0     Reserved
  1     1     Internal 1.1V Voltage Reference with external capacitor at AREF pin

  (ATMEGA328P)
  ADC: 10-bit
  Vint: 1.0V (min) 1.1V (typ) 1.2V (max)
  ADC clock: 125kHz (maximum for full resolution at 16MHz)
    normal conversion: 13 ADC clock cycles (+1 to start the next)
    first conversion (ADEN in ADC-SRA is set): 25 ADC clock cycles
  Analog Input Resistance: 100MΩ
*/


/*
  
*/
void ADC_init(void)
{
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // enable ADC, ADC requires clock signal between 50kHz and 200kHz, prescaler 128 (16000000 / 128 = 125kHz)
  ADMUX = (1 << REFS0);                                               // voltage reference AVCC with external capacitor at AREF pin, MUX3:0 -> channel 0 default
  _delay_us(20000);                                                   // delay for the voltage to settle
  // one conversion after REF change should be discarded
  ADCSRA |= (1 << ADSC);                                              // start single conversion for selected Analog Input
  while(ADCSRA & (1 << ADSC));                                        // wait for the end of conversion (ADSC = 0)
}


/*

*/
uint16_t ADC_sample(uint8_t channel)
{
  uint32_t temp = 0;

  channel &= 0b00011111;
  ADMUX = (1 << REFS0) | channel;                                     // voltage reference AVCC with external capacitor at AREF pin, MUX3:0 -> channel

  for(uint16_t y = 0; y < ADCSAMPLES; y++)
  {
    ADCSRA |= (1 << ADSC);                                            // start single conversion for selected Analog Input
    while(ADCSRA & (1 << ADSC));                                      // wait for the end of conversion (ADSC = 0)
    temp += ADC;
  }

  return (temp / ADCSAMPLES);
}











