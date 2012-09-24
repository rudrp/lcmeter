#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>


/* note: lcd model MC21605A6W */
#define CONFIG_LCD 1
#define CONFIG_UART 0


#if CONFIG_LCD

/* notes: DB0:3 and RW must be grounded */

#define LCD_POS_DB 0x00
#define LCD_PORT_DB PORTC
#define LCD_DIR_DB DDRC
#define LCD_MASK_DB (0x0f << LCD_POS_DB)

#define LCD_POS_RS 0x04
#define LCD_PORT_RS PORTC
#define LCD_DIR_RS DDRC
#define LCD_MASK_RS (0x01 << LCD_POS_RS)

#define LCD_POS_EN 0x05
#define LCD_PORT_EN PORTC
#define LCD_DIR_EN DDRC
#define LCD_MASK_EN (0x01 << LCD_POS_EN)


static inline void wait_50_ns(void)
{
  __asm__ __volatile__ ("nop\n\t");
}

static inline void wait_500_ns(void)
{
  /* 8 cycles at 16mhz */
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
}

static inline void wait_50_us(void)
{
  /* 800 cycles at 16mhz */
  uint8_t x;
  for (x = 0; x < 100; ++x) wait_500_ns();
}

static inline void wait_2_ms(void)
{
  wait_50_us();
  wait_50_us();
  wait_50_us();
  wait_50_us();
}

static inline void wait_50_ms(void)
{
  /* FIXME: was _delay_ms(50), but not working */
  uint8_t x;
  for (x = 0; x < 25; ++x) wait_2_ms();
}

static inline void lcd_pulse_en(void)
{
  /* assume EN low */
  LCD_PORT_EN |= LCD_MASK_EN;
  wait_50_us();
  LCD_PORT_EN &= ~LCD_MASK_EN;
  wait_2_ms();
}

static inline void lcd_write_db4(uint8_t x)
{
  /* configured in 4 bits mode */

  LCD_PORT_DB &= ~LCD_MASK_DB;
  LCD_PORT_DB |= (x >> 4) /* >> LCD_POS_DB */ ;
  lcd_pulse_en();

  LCD_PORT_DB &= ~LCD_MASK_DB;
  LCD_PORT_DB |= (x & 0xf) /* >> LCD_POS_DB */ ;
  lcd_pulse_en();
}

static inline void lcd_write_db8(uint8_t x)
{
  /* configured in 8 bits mode */

  /* only hi nibble transmitted, (0:3) grounded */
  LCD_PORT_DB &= ~LCD_MASK_DB;
  LCD_PORT_DB |= (x >> 4) /* >> LCD_POS_DB */ ;
  lcd_pulse_en();
}

static inline void lcd_setup(void)
{
  LCD_DIR_DB |= LCD_MASK_DB;
  LCD_DIR_RS |= LCD_MASK_RS;
  LCD_DIR_EN |= LCD_MASK_EN;

  LCD_PORT_DB &= ~LCD_MASK_DB;
  LCD_PORT_RS &= ~LCD_MASK_RS;
  LCD_PORT_EN &= ~LCD_MASK_EN;

  /* small delay for the lcd to boot */
  wait_50_ms();

  /* datasheet init sequence */

#define LCD_MODE_BLINK (1 << 0)
#define LCD_MODE_CURSOR (1 << 1)
#define LCD_MODE_DISPLAY (1 << 2)

  lcd_write_db8(0x30);
  wait_2_ms();
  wait_2_ms();
  wait_500_ns();

  lcd_write_db8(0x30);
  wait_2_ms();

  lcd_write_db4(0x32);
  wait_2_ms();

  lcd_write_db4(0x28);
  wait_2_ms();

  lcd_write_db4((1 << 3) | LCD_MODE_DISPLAY);
  wait_2_ms();

  lcd_write_db4(0x01);
  wait_2_ms();

  lcd_write_db4(0x0f);
  wait_2_ms();
}

static inline void lcd_clear(void)
{
  /* clear lcd */
  lcd_write_db4(0x01);
  wait_2_ms();
}

static inline void lcd_home(void)
{
  /* set cursor to home */
  lcd_write_db4(0x02);
  wait_2_ms();
}

static inline void lcd_set_cursor(uint8_t addr)
{
  lcd_write_db4((1 << 7) | addr);
  wait_2_ms();
}

static void lcd_write(const uint8_t* s, unsigned int n)
{
  wait_50_ns();

  LCD_PORT_RS |= LCD_MASK_RS;
  for (; n; --n, ++s)
  {
    lcd_write_db4(*s);
    wait_2_ms();
  }
  LCD_PORT_RS &= ~LCD_MASK_RS;
}

#endif /* CONFIG_LCD */


#if CONFIG_UART /* uart */

static inline void set_baud_rate(long baud)
{
  uint16_t UBRR0_value = ((F_CPU / 16 + baud / 2) / baud - 1);
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;
}

static void uart_setup(void)
{
  /* #define CONFIG_FOSC (F_CPU * 2) */
  /* const uint16_t x = CONFIG_FOSC / (16 * BAUDS) - 1; */
#if 0 /* (bauds == 9600) */
  const uint16_t x = 206;
#elif 0 /* (bauds == 115200) */
  const uint16_t x = 16;
#elif 0 /* (bauds == 500000) */
  const uint16_t x = 3;
#elif 0 /* (bauds == 1000000) */
  const uint16_t x = 1;
#endif

  set_baud_rate(9600);

  /* baud doubler off  - Only needed on Uno XXX */
  UCSR0A &= ~(1 << U2X0);

  UCSR0B = 1 << TXEN0;

  /* default to 8n1 framing */
  UCSR0C = (3 << 1);
}

static void uart_write(const uint8_t* s, uint8_t n)
{
  for (; n; --n, ++s)
  {
    /* wait for transmit buffer to be empty */
    while (!(UCSR0A & (1 << UDRE0))) ;
    UDR0 = *s;
  }

  /* wait for last byte to be sent */
  while ((UCSR0A & (1 << 6)) == 0) ;
}

#endif /* CONFIG_UART */


static inline uint8_t nibble(uint32_t x, uint8_t i)
{
  return (x >> (i * 4)) & 0xf;
}

static inline uint8_t hex(uint8_t x)
{
  return (x >= 0xa) ? 'a' + x - 0xa : '0' + x;
}

__attribute__((unused))
static uint8_t* uint32_to_string(uint32_t x)
{
  static uint8_t buf[8];

  buf[7] = hex(nibble(x, 0));
  buf[6] = hex(nibble(x, 1));
  buf[5] = hex(nibble(x, 2));
  buf[4] = hex(nibble(x, 3));
  buf[3] = hex(nibble(x, 4));
  buf[2] = hex(nibble(x, 5));
  buf[1] = hex(nibble(x, 6));
  buf[0] = hex(nibble(x, 7));

  return buf;
}

static unsigned int ulong_to_string(unsigned long n, char* s)
{
  unsigned char buf[8];
  unsigned long i = 0;
  unsigned int len = 0;
  if (n == 0) { *s = '0'; return 1; }
  while (n > 0) { buf[i++] = n % 10; n /= 10; }
  for (; i > 0; i--, ++len) s[len] = '0' + buf[i - 1];
  return len;
}

static unsigned int double_to_string(double x, const uint8_t** s)
{
  static char buf[32];
  uint8_t digits = 3;
  double rounding = 0.5;
  unsigned int len = 0;
  uint8_t i;
  unsigned long int_part;
  double remainder;

  if (x < 0.0) { buf[len++] = '-'; x = -x; }

  for (i = 0; i < digits; ++i) rounding /= 10.0;
  x += rounding;

  int_part = (unsigned long)x;
  remainder = x - (double)int_part;

  len += ulong_to_string(int_part, buf + len);

  if (digits > 0) buf[len++] = '.';

  while ((digits--) > 0)
  {
    unsigned long xx;
    remainder *= 10.0;
    xx = (unsigned long)remainder;
    len += ulong_to_string(xx, buf + len);
    remainder -= xx; 
  }

  *s = (const uint8_t*)buf;

  return len;
}


/* high resolution frequency counter implementation
 */

/* timer2 interrupt handler. timer1 is an extended
   32 bits register (16 bits hard + 16 softs)
   incremented once per:
   1 / (fcpu / prescal) <=> prescal / fcpu
   thus, it will overflow at:
   2^16 * prescal / fcpu
   on tim2 overflow, the interrupt handler is called
   and stores the tim1 current value in tim1_cur_counter.
   thus, the tim1 value integrated over the whole
   tim1 period is:
   (tim1_ovf_counter * 2^16) + tim1_cur_counter.
   tim2_is_ovf is set to notify the application.
 */

static volatile uint8_t tim2_ovf_counter;
static volatile uint8_t tim2_is_ovf;
static volatile uint16_t tim1_cur_counter;

ISR(TIMER2_OVF_vect)
{
  if ((tim2_ovf_counter--) == 0)
  {
    /* disable tim1 before reading */
    TCCR1B = 0;
    tim1_cur_counter = TCNT1;

    /* disable tim2 */
    TCCR2B = 0;

    tim2_is_ovf = 1;
  }
}

/* timer2 interrupt handler. timer2 is a 8 bits counter
   incremented by the input signal rising edges. since
   8 bits are not enough to integrate, an auxiliary
   register (tim2_ovf_counter) is updated on overflow.
   tim2_ovf_counter is an 8 bits register, and will
   overflow without any notice past 0xff.
 */

static volatile uint8_t tim1_ovf_counter;

ISR(TIMER1_OVF_vect)
{
  ++tim1_ovf_counter;
}

static void hfc_start(void)
{
  /* resolution: 1.907349 hz per tick */
  /* fmax: 500 khz */
  /* acquisition time: 0.524288 seconds */

  /* disable interrupts */
  TIMSK1 = 0;
  TIMSK2 = 0;

  /* reset stuff */
  tim1_ovf_counter = 0;
  tim1_cur_counter = 0;
  tim2_is_ovf = 0;

  /* 0x100 overflows make 16 bits */
  tim2_ovf_counter = 0xff;

  /* configure tim2
     normal operation
     prescaler 128
     enable interrupt on overflow
   */
  TCNT2 = 0;
  TIMSK2 = 1 << 0;
  TCCR2A = 0;
  TCCR2B = 0;

  /* configure tim1
     t1 pin (pd5) rising edge as external clock
   */
  DDRD &= ~(1 << 5);
  TCNT1 = 0;
  TIMSK1 = 1 << 0;
  TCCR1A = 0;
  TCCR1B = 0;

  /* start tim1, tim2 */
  TCCR1B = 7 << 0;
  TCCR2B = 5 << 0;
}

static uint8_t hfc_poll(void)
{
  return tim2_is_ovf;
}

static uint32_t hfc_wait(void)
{
  /* busy wait for tim1 to overflow. returns the resulting
     16 bits counter, to be multiplied by the frequency
     resolution (refer to hfc_start) to get the actual
     frequency.
   */

  /* force inline, do not use hfc_poll */
  while (tim2_is_ovf == 0) ;

  return ((uint32_t)tim1_ovf_counter << 16) | (uint32_t)tim1_cur_counter;
}

static inline uint32_t hfc_start_wait(void)
{
  hfc_start();
  return hfc_wait();
}

static inline double hfc_to_hz(uint32_t counter)
{
  return 1.907349 * (double)counter;
}


/* frequency conversion */

/* theoritical values */
#define lth 331 /* uh */
#define cth 1000 /* pf */
#define fth 276634.602 /* hz */

/* calibrated values */
static double lref = lth; /* uh */
static double cref = cth; /* pf */
static double fref = fth; /* hz */

static double freq_to_xxx(double f, double xref)
{
  const double x = fref / f;
  return (x * x - 1.0) * xref;
}

static double freq_to_c(double f)
{
  /* frequency (hz) to capacitance (pf) */
  return freq_to_xxx(f, cref);
}

static double freq_to_l(double f)
{
  /* frequency (hz) to inductance (uh) */
  return freq_to_xxx(f, lref);
}


int main(void)
{
  uint8_t is_l;
  uint8_t is_calib;
  uint32_t counter;
  const uint8_t* s;
  double f = 1;
  double prev_x = 0;
  double x;
  unsigned int len;

#if CONFIG_LCD
  lcd_setup();
  lcd_clear();
  lcd_home();
#endif

#if CONFIG_UART
  uart_setup();
#endif

  sei();

  /* l or c mode sense */
  DDRD &= ~(1 << 2);
  PORTD |= 1 << 2;

  /* calibrate button */
  DDRD &= ~(1 << 3);
  PORTD |= 1 << 3;

  /* measure */
  while (1)
  {
    is_l = 0;
    if ((PIND & (1 << 2)) == 0) is_l = 1;

    is_calib = 0;
    if ((PIND & (1 << 3)) == 0) is_calib = 1;

    counter = hfc_start_wait();
    f = hfc_to_hz(counter);
    x = is_l ? freq_to_l(f) : freq_to_c(f);

    /* calibration */
    if (is_calib)
    {
      if (is_l) lref = lth + x;
      else cref = cth + x;
      fref = f;
    }

    /* avoid update */
    if (prev_x == x) continue ;

#if CONFIG_LCD
    lcd_clear();
    lcd_home();
#endif

    if (is_l)
    {
#if CONFIG_UART
      uart_write((uint8_t*)"L", 1);
#endif
#if CONFIG_LCD
      lcd_write((uint8_t*)"L", 1);
#endif
    }
    else
    {
#if CONFIG_UART
      uart_write((uint8_t*)"C", 1);
#endif
#if CONFIG_LCD
      lcd_write((uint8_t*)"C", 1);
#endif
    }

#if CONFIG_UART || CONFIG_LCD
    /* print x */
    len = double_to_string(x, &s);

#if CONFIG_UART
    uart_write((uint8_t*)" == ", 4);
    uart_write((uint8_t*)s, len);
    uart_write((uint8_t*)"\r\n", 2);
#endif

#if CONFIG_LCD
    lcd_write((uint8_t*)" == ", 4);
    /* avoid lcd overflow */
    lcd_write((uint8_t*)s, len > 10 ? 10 : len);
#endif

    /* print frequency */
    len = double_to_string(f, &s);

#if CONFIG_UART
    uart_write((uint8_t*)s, len);
    uart_write((uint8_t*)" hz\r\n", 5);
    uart_write((uint8_t*)"\r\n", 2);
#endif

#if CONFIG_LCD
    lcd_set_cursor(0x40);
    lcd_write((uint8_t*)s, len);
#endif

#endif /* CONFIG_UART || CONFIG_LCD */

    prev_x = x;
  }

  return 0;
}
