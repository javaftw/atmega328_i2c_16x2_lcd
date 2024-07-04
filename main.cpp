/******************************************************************************/
/*                             PROJECT PINOUT                                 */
/******************************************************************************/
/*                             ATMega328P
 *                                ______
 *            RESET/PCINT14/PC6 =|01* 28|= PC5/PCINT13/SCL/ADC5
 *               RX/PCINT16/PD0 =|02  27|= PC4/PCINT12/SDA/ADC4
 *               TX/PCINT17/PD1 =|03  26|= PC3/PCINT11/ADC3
 *             INT0/PCINT18/PD2 =|04  25|= PC2/PCINT10/ADC2
 *                  PCINT19/PD3 =|05  24|= PC1/PCINT9/ADC1
 *                  PCINT20/PD4 =|06  23|= PC0/PCINT8/ADC0
 *                          Vcc =|07  22|= GND
 *                          GND =|08  21|= Aref
 *             XTAL1/PCINT6/PB6 =|09  20|= AVcc
 *             XTAL2/PCINT7/PB7 =|10  19|= PB5/PCINT5/SCK
 *             OC0B/PCINT21/PD5 =|11  18|= PB4/PCINT4/MISO
 *        OC0A/AIN0/PCINT22/PD6 =|12  17|= PB3/PCINT3/MOSI/OC2A/OC2
 *             AIN1/PCINT23/PD7 =|13  16|= PB2/PCINT2/SS/OC1B
 *                   PCINT0/PB0 =|14  15|= PB1/PCINT1/OC1A
 *                                ------
 * 
 *                                ______
 *                              =|01* 28|= SCL 
 *                              =|02  27|= SDA 
 *                              =|03  26|= 
 *                              =|04  25|= 
 *                              =|05  24|= 
 *                              =|06  23|= 
 *                          Vcc =|07  22|= GND
 *                          GND =|08  21|= Aref
 *                              =|09  20|= 
 *                              =|10  19|= 
 *                              =|11  18|= 
 *                              =|12  17|= 
 *                              =|13  16|= 
 *                              =|14  15|= 
 *                                ------
 * 
 * 
 */

/* *** MASTER *** */


/******************************************************************************/
/*                                   DEFS                                     */
/******************************************************************************/

#define F_CPU   8000000
#define MEMSIZE   2048
#define TRUE    1
#define FALSE   0
#define ON      1
#define OFF     0
#define HIGH    1
#define LOW     0
#define NOP     asm("nop")

#define ADC_REF _BV(REFS0)

//#define SLAVE_ADDRESS 0b00111100 //0x3c
#define SLAVE_ADDRESS 0x3F
//#define SLAVE_ADDRESS 0x27


// commands
#define LCD_CLEARDISPLAY            0x01
#define LCD_RETURNHOME              0x02
#define LCD_ENTRYMODESET            0x04
#define LCD_DISPLAYCONTROL          0x08
#define LCD_CURSORSHIFT             0x10
#define LCD_FUNCTIONSET             0x20
#define LCD_SETCGRAMADDR            0x40
#define LCD_SETDDRAMADDR            0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT              0x00
#define LCD_ENTRYLEFT               0x02
#define LCD_ENTRYSHIFTINCREMENT     0x01
#define LCD_ENTRYSHIFTDECREMENT     0x00

// flags for display on/off control
#define LCD_DISPLAYON               0x04
#define LCD_DISPLAYOFF              0x00
#define LCD_CURSORON                0x02
#define LCD_CURSOROFF               0x00
#define LCD_BLINKON                 0x01
#define LCD_BLINKOFF                ~(LCD_BLINKON)

// flags for display/cursor shift
#define LCD_DISPLAYMOVE             0x08
#define LCD_CURSORMOVE              0x00
#define LCD_MOVERIGHT               0x04
#define LCD_MOVELEFT                0x00

// flags for function set
#define LCD_8BITMODE                0x10
#define LCD_4BITMODE                0x00
#define LCD_2LINE                   0x08
#define LCD_1LINE                   0x00
#define LCD_5x10DOTS                0x04
#define LCD_5x8DOTS                 0x00

// flags for backlight control
#define LCD_BACKLIGHT_ON            0b00001000
#define LCD_BACKLIGHT_OFF           0b00000000
#define En                          0b00000100  // Enable bit
#define Rw                          0b00000010  // Read/Write bit
#define Rs                          0b00000001  // Register select bit



#define LED 0x80


//---MACROS
char rxbuf[8];
char txbuf[8];
#define CMD_EQ(A)       strcmp(A, rxbuf) == 0
#define CMD_EQN(A,N)    strncmp(A, rxbuf, N) == 0
#define ITOA(A)         itoa(A, txbuf, 10)
#define ITOA2(A)         itoa(A, txbuf, 2)
#define ITOA16(A)         itoa(A, txbuf, 16)

#define _setpin(P,p)          (P |= p)
#define _clearpin(P,p)        (P &= ~(p))
#define _togglepin(P,p)       (P ^= p)
#define _testpin(P,p)         ((P & p) == p)

/******************************************************************************/
/*                                 INCLUDES                                   */
/******************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdlib.h>
#include <string.h>
#include "i2c.h"

/******************************************************************************/
/*                                VARIABLES                                   */

uint16_t timer_counter;
uint16_t adcval;
uint8_t numBytesRead;
uint16_t pwm_freq;

//------------------------------------------bitmaps
//https://maxpromer.github.io/LCD-Character-Creator/
uint8_t map[] = {
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000
};

const uint8_t snail_map[] = {
    0b11011,
    0b11011,
    0b01010,
    0b11111,
    0b10000,
    0b10110,
    0b10001,
    0b01110
};

const uint8_t rocket_map[] = {
    0b00100,
    0b01010,
    0b01110,
    0b01010,
    0b01010,
    0b11011,
    0b11111,
    0b10001
};

const uint8_t PWM_duty_cycle_map[] = {
  0b01110,
  0b01010,
  0b00000,
  0b00000,
  0b01110,
  0b01010,
  0b11011,
  0b00000
};

const uint8_t PWM_period_map[] = {
  0b11111,
  0b10001,
  0b00000,
  0b00000,
  0b11100,
  0b10101,
  0b00111,
  0b00000
};

/******************************************************************************/


/******************************************************************************/

/*                             STRUCTS AND ENUMS                              */
enum Events {
    EVT_NONE,
    EVT_READING_ADC,
    EVT_ADCVAL_CHANGED
};
volatile Events evt;
/******************************************************************************/


/******************************************************************************/
/*                            FUNCTION DECLARATIONS                           */
/******************************************************************************/

int main(void);
static void init(void);
void mainloop(void);
//LCD
void initLCD(void);
void lcd_send_byte(uint8_t, uint8_t);
void lcd_send_nibble(uint8_t);
void lcd_send_nibble_pulse_enable(uint8_t);
void lcd_write_char(uint8_t);
void lcd_write_string(const char *, uint8_t);
void lcd_send_command(uint8_t);
void lcd_clear(void);
void lcd_home(void);
void lcd_cursor_at(uint8_t, uint8_t);
void lcd_update_multiline(uint8_t *);
void lcd_scroll_top_line(uint8_t *, uint8_t);
void lcd_create_char(uint8_t, const uint8_t *);
//ADC
uint16_t read_adc(uint8_t chan);
uint16_t compute_average(uint16_t data[], uint8_t len);
//timer
ISR(TIMER0_OVF_vect);
//util
void delay_n_us(uint16_t n);
void delay_n_ms(uint16_t n);
int available_sram(void);

/******************************************************************************
 *                                FUNCTIONS                                   *
 ******************************************************************************/

int main(void) {
    evt = EVT_NONE;
    //variable initialization
    //function initialization
    init();
    init_i2c();
    mainloop();
    return 0;
}

/*******************************************************************************
 *                                                                        INIT*/

static void init(void) {

    //=======================================================================I/O
    //Direction registers port c 1=output 0=input
    /*If DDxn is written logic one, Pxn is configured as an output pin. 
     * If DDxn is written logic zero, Pxn is configured as an input pin.
     * If PORTxn is written logic one AND the pin is configured as an input pin (0), 
     * the pull-up resistor is activated.
     */
    DDRB |= 0b00000001; //PC0 is input
    PORTB |= 0b00000010; //Pull-up activated on PC0

    DDRC |= 0b00000000; //
    PORTC |= 0b11111111; //Pull-up activated on PC0

    DDRD |= 0b11000000; //
    PORTD |= 0b00000000; //Pull-up activated on PC0

    //=======================================================================ADC
    ADMUX = ADC_REF | 0x00; // | _BV(MUX0);//source is AVCC, select only channel 1
    //enable ADC, select ADC clock to 8000000Hz/64 = 125000Hz (ADPS = 110)
    ADCSRA |= _BV(ADEN) | _BV(ADPS1) | _BV(ADPS2);


    //====================================================================TIMER0
    TCNT0 = 0x00;
    TCCR0B |= _BV(CS00); // | _BV(CS02);//prescaler
    TIMSK0 |= _BV(TOV0); //timer 0 overflow interrupt enable


    //================================================================INTERRUPTS
    sei();
}

void initLCD() {
    _delay_ms(500);
    lcd_send_nibble(0x30);
    _delay_ms(5);
    lcd_send_nibble(0x30);
    _delay_ms(5);
    lcd_send_nibble(0x30);
    _delay_ms(15);
    lcd_send_byte(0x20,Rs);
    //lcd_send_nibble_pulse_enable(0x20); //4-bit mode, 1 line, 5x7 chars
    //lcd_send_nibble_pulse_enable(0x00); //4-bit mode, 1 line, 5x7 chars
    _delay_ms(5);
    //lcd_send_nibble_pulse_enable(0x28); //2 lines??
    // lcd_send_nibble_pulse_enable(0x00); //
    _delay_ms(5);
    lcd_send_nibble_pulse_enable(0x00); //0x0C dispaly on cursor off
    lcd_send_nibble_pulse_enable(0xC0); //0x0C dispaly on cursor off
    //lcd_send_nibble(0x0E);//0x0E display on cursor on.
    //lcd_send_nibble(0x0F);//0x0F display on cursor blink.
    _delay_ms(5);
    //_delay_ms(150);
    lcd_send_byte(0x01,Rs);
    //lcd_send_nibble_pulse_enable(0x00); //0x01 clear DDRAM
    //lcd_send_nibble_pulse_enable(0x01); //0x01 clear DDRAM
    _delay_ms(500);
}

/******************************************************************************
 *                                                                    MAINLOOP*/


void mainloop(void) {
    initLCD();
    _delay_ms(250);
    lcd_home();
    _delay_ms(100);
    lcd_clear();
    _delay_ms(200);
    lcd_clear();
    _delay_ms(200);
    lcd_create_char(0, snail_map);
    lcd_cursor_at(9, 1);
    lcd_send_byte(0, Rs);
    _delay_ms(100);
    lcd_create_char(1, rocket_map);
    lcd_cursor_at(10, 2);
    lcd_send_byte(1, Rs);
    _delay_ms(100);
    lcd_create_char(2, PWM_duty_cycle_map);
    lcd_cursor_at(1, 1);
    lcd_send_byte(2, Rs);
    _delay_ms(100);
    lcd_create_char(3, PWM_period_map);
    lcd_cursor_at(14, 1);
    lcd_send_byte(3, Rs);
    _delay_ms(1200);
    lcd_home();
    lcd_cursor_at(2, 1);
    lcd_write_string(":", 0);
    while (TRUE) {
        switch (evt) {
            case EVT_NONE:
                _delay_ms(1);
                break;

            case EVT_READING_ADC:
                uint16_t temp;
                temp = (read_adc(0) + adcval) / 20;
                if (adcval != temp) {
                    adcval = temp;
                    evt = EVT_ADCVAL_CHANGED;
                }
                break;

            case EVT_ADCVAL_CHANGED:
                evt = EVT_NONE;
                lcd_home();
                lcd_cursor_at(4, 1);
                lcd_write_string(ITOA(adcval * 2), 0);
                lcd_write_string("% ", 0);
                _delay_ms(200);
                break;
        }

    }
}

/******************************************************************************
 *                                                                LCD   */

void lcd_send_byte(uint8_t data, uint8_t mode) {
    uint8_t HI = 0xF0 & data;
    uint8_t LO = 0xF0 & (data << 4);
    lcd_send_nibble_pulse_enable(HI | mode);
    lcd_send_nibble_pulse_enable(LO | mode);
}

void lcd_send_nibble(uint8_t data) {
    i2c_send_command_W(SLAVE_ADDRESS, data | LCD_BACKLIGHT_ON/**/);
}

void lcd_send_nibble_pulse_enable(uint8_t data) {
    i2c_send_command_W(SLAVE_ADDRESS, data | LCD_BACKLIGHT_ON/**/ | En);
    _delay_ms(2);
    i2c_send_command_W(SLAVE_ADDRESS, data | LCD_BACKLIGHT_ON/**/ | ~En);
    _delay_ms(2);
}

void lcd_write_char(uint8_t data) {
    lcd_send_byte(data, Rs);
}

void lcd_write_string(const char * str, uint8_t len) {
    if (len == 0) {
        for (uint8_t idx = 0; str[idx] != '\0'; idx++) {
            lcd_write_char(str[idx]);
        }
    } else {
        for (uint8_t idx = 0; idx < len && str[idx] != '\0'; idx++) {
            lcd_write_char(str[idx]);
        }
    }
}

void lcd_send_command(uint8_t data) {
    lcd_send_byte(data, 0x00);
}

//-----------------------------LCD higher level

void lcd_clear(void) {
    lcd_send_command(0x01);
    _delay_ms(2);
}

void lcd_home(void) {
    lcd_send_command(0x02);
    _delay_ms(2);
}

void lcd_cursor_at(uint8_t col, uint8_t row) {
    int row_offsets[] = {0x00, 0x40};
    lcd_send_command(LCD_SETDDRAMADDR | (col - 1 + row_offsets[row - 1]));
}

void lcd_update_multiline(uint8_t *str) {
    lcd_clear();
    lcd_home();
    /*if (line_1_len == 0) {
        line_1_len = uart_received_buffer_size;
        copyarr(str, lcd_line_1);
    } else {
        line_2_len = uart_received_buffer_size;
        copyarr(str, lcd_line_2);
    }
    lcd_write_string(lcd_line_1);
    if (line_2_len > 0) {
        lcd_cursor_at(1, 2);
        lcd_write_string(lcd_line_2);
        copyarr(lcd_line_2, lcd_line_1);
        line_1_len = line_2_len;
        line_2_len = 0;
    }*/
}

void lcd_scroll_top_line(uint8_t *str, uint8_t speed) {
    /*uint8_t displaybuf[8];*/

}

void lcd_create_char(uint8_t location, const uint8_t * _map) {
    location &= 0x07;
    lcd_send_command(LCD_SETCGRAMADDR | (location << 3));
    for (uint8_t i = 0; i < 8; i++) {
        lcd_send_byte(_map[i], Rs);
    }
}

/******************************************************************************
 *                                                                        ADC*/

uint16_t read_adc(uint8_t chan) {
    ADMUX = ADC_REF | chan; //channel 1
    _delay_ms(1);
    ADCSRA |= _BV(ADSC); //enable adc and start conversion
    while (ADCSRA & _BV(ADSC)); //wait for conversion to complete
    return ADCW;
}

uint16_t compute_average(uint16_t data[], uint8_t len) {
    uint16_t ret = 0;
    for (uint8_t i = 0; i < len; i++) {
        ret += data[i];
    }
    return ret / len;
}

/******************************************************************************
 *                                                                      TIMER0*/
ISR(TIMER0_OVF_vect) {
    timer_counter++;
    if (timer_counter > 0x0100) {
        timer_counter = 0;
        evt = EVT_READING_ADC;
    }
}

/*****************************************************************************
 *                                                                        UTIL*/

void delay_n_us(uint16_t n) {
    while (n--) {
        _delay_us(1);
    }
}

void delay_n_ms(uint16_t n) {
    while (n--) {
        _delay_ms(1);
    }
}

int available_sram(void) {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}