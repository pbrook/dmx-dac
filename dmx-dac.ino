/*
  DMX DAC controller
 */
 
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "dmx-dac.h"

/* DAC module connector pinout (pin1 to middle of board):
      12v 1  2 GND
       5v 3  4 CS0
      SCK 5  6 CS1
     DATA 7  8 CS2
      3v3 9 10 CS3
  */
#define NUM_DACS 6

/* Hardware bug: bus 2 CS1/2 (pin 7 and 8) are swapped.  */
static const uint8_t load_pin[NUM_DACS] = {A0, A1, A2, 9, 7, 8};
// Additional unuised chip selects on A3, 6

#define MAX_CHANNELS (NUM_DACS * 8)
int first_channel;
int frame_bytes = -1;
volatile uint8_t channel_value[MAX_CHANNELS];
volatile uint8_t channel_changed[NUM_DACS];
int last_channel;
int last_value;
volatile bool dac_active;

static void
spi_write(uint8_t val)
{
  SPDR = val;
}

static void
send_next_dac(void)
{
  int channel;
  uint8_t dac;
  uint8_t n;

  channel = last_channel;
  while (true) {
      channel++;
      if (channel == MAX_CHANNELS)
	channel = 0;
      dac = channel >> 3;
      n = channel & 7;
      if (channel_changed[dac] & (1 << n)) {
	  channel_changed[dac] &= ~(1 << n);
	  last_channel = channel;
	  last_value = channel_value[channel];
	  dac_active = true;
	  spi_write(n << 1);
	  return;
      }

      if (channel == last_channel) {
	  dac_active = 0;
	  return;
      }
  }
}

static void
restart_dac(void)
{
  uint8_t dac;
  bool changed;

  changed = false;
  for (dac = 0; dac < NUM_DACS; dac++) {
      if (channel_changed[dac])
	changed = true;
  }
  if (!changed)
    return;

  send_next_dac();
}

ISR(SPI_STC_vect)
{
  uint8_t pin;

  if (last_value == -1) {
      pin = load_pin[last_channel >> 3];
      delayMicroseconds(1);
      digitalWrite(pin, 0);
      delayMicroseconds(1);
      digitalWrite(pin, 1);
      send_next_dac();
  } else {
      spi_write(last_value);
      last_value = -1;
  }
}

void
dmx_set_channel(int channel, uint8_t val)
{
  channel -= first_channel;
  if (channel < 0 || channel >= MAX_CHANNELS)
    return;
  if (channel_value[channel] != val) {
      channel_value[channel] = val;
      channel_changed[channel >> 3] |= (1 << (channel & 7));
      if (!dac_active) {
	  restart_dac();
      }
  }
}

ISR(USART1_RX_vect)
{
  uint8_t flags;
  uint8_t data;

  flags = UCSR1A;
  data = UDR1;
  if (flags & _BV(FE1)) {
      frame_bytes = (data == 0) ? 0 : -1;
      return;
  }
  if (frame_bytes < 0)
    return;
  if (frame_bytes == 0) {
      frame_bytes = (data == 0) ? 1 : -1;
      return;
  }
  dmx_set_channel(frame_bytes - 1, data);
  frame_bytes++;
}

static void
setup_uart(void)
{
  UCSR1A = 0;
  UCSR1C = _BV(USBS1) | _BV(UCSZ11) | _BV(UCSZ10);
#if F_CPU == 16000000ul
  UBRR1 = 3;
#else
#error unsupported CPU speed
#endif
  UCSR1B = _BV(RXCIE1) | _BV(RXEN1);
}

static void
dac_init(void)
{
  int i;

  for (i = 0; i < NUM_DACS; i++) {
      pinMode(load_pin[i], OUTPUT);
      digitalWrite(load_pin[i], 1);
      channel_value[i] = 0;
      channel_changed[i] = ~0;
  }
  pinMode(SS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  /* fosc/32, MSB first, mode 1.  */
  SPCR = _BV(SPIE) | _BV(SPE) | _BV(MSTR) | _BV(CPHA) | _BV(SPR1);
  SPSR = _BV(SPI2X);
}

static void
watchdog_init(void)
{
  wdt_enable(WDTO_1S);
}

int main(void)
{
  watchdog_init();
  init();
  setup_uart();
  dac_init();
  usb_init();

  sei();
  set_sleep_mode(SLEEP_MODE_IDLE);
  //dmx_set_channel(0, 128);
  while (true) {
      wdt_reset();
      sleep_cpu();
  }
}

