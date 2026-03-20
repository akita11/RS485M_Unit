#include <stdint.h>
#include "fw_reg_stc8g.h"

// STC8G1K08A-36I-DFN8 (ROM=8kB)
// P3.0: RXD (UART1 receive)
// P3.2: TX_EN (RS485 transmit enable, push-pull output)

#define FOSC 11059200UL

//#define DEBUG  // Uncomment to enable UART debug output during auto-baud detection

// Pin definitions
#define pinRXD  P30  // P3.0
#define pinTXEN P32  // P3.2

// SCON.7 is FE when SMOD0=1 (shares bit with SM0)
#define FE SM0

// EEPROM (IAP) definitions
#define IAP_ADDR_BASE   0x0000
#define IAP_MAGIC       0xA5
#define IAP_MAGIC2      0x5A
// Layout: [0]=MAGIC [1]=MAGIC2 [2]=bit_period_H [3]=bit_period_L [4]=txen_per_byte

// Baud rate detection timeout
#define BAUD_DETECT_TIMEOUT_MS  5000UL
#define TIMER0_OVERFLOW_MS      6UL
#define BAUD_TIMEOUT_OVERFLOWS  (BAUD_DETECT_TIMEOUT_MS / TIMER0_OVERFLOW_MS)
#define TOGGLE_PERIOD_OVERFLOWS  (500UL / TIMER0_OVERFLOW_MS)   // ~83 overflows ≈ 0.5 second

// Global variables
volatile uint16_t bit_period = FOSC / 9600;  // initial: 1152 SYSclk per bit
volatile uint8_t txen_per_byte = 1;          // 1: per-byte mode, 0: per-packet mode
volatile uint16_t baud_detect_timeout_counter = 0;
volatile uint8_t baud_detect_in_progress = 0;
volatile uint8_t do_auto_baud = 0;

void delay_cycles(uint16_t cycles)
{
  CL = 0;
  CH = 0;
  CR = 1;
  while (((uint16_t)CH << 8 | CL) < cycles);
  CR = 0;
}

void delay_ms(uint16_t ms)
{
  // 1ms ~ 11059 SYSclk cycles (@ 11.0592MHz)
  while (ms--) {
    delay_cycles(11059);
  }
}

// IAP (EEPROM) functions
void iap_idle(void)
{
  IAP_CONTR = 0;
  IAP_CMD = 0;
  IAP_TRIG = 0;
  IAP_ADDRH = 0x80;  // point to non-existent address
  IAP_ADDRL = 0;
}

uint8_t iap_read(uint16_t addr)
{
  IAP_CONTR = 0x80;      // IAPEN=1
  IAP_TPS = 11;          // FOSC=11.0592MHz -> 11
  IAP_CMD = 1;           // read
  IAP_ADDRH = addr >> 8;
  IAP_ADDRL = addr & 0xFF;
  IAP_TRIG = 0x5A;
  IAP_TRIG = 0xA5;
  NOP();
  {
    uint8_t val = IAP_DATA;
    iap_idle();
    return val;
  }
}

void iap_write(uint16_t addr, uint8_t dat)
{
  IAP_CONTR = 0x80;
  IAP_TPS = 11;
  IAP_CMD = 2;           // write
  IAP_ADDRH = addr >> 8;
  IAP_ADDRL = addr & 0xFF;
  IAP_DATA = dat;
  IAP_TRIG = 0x5A;
  IAP_TRIG = 0xA5;
  NOP();
  iap_idle();
}

void iap_erase(uint16_t addr)
{
  IAP_CONTR = 0x80;
  IAP_TPS = 11;
  IAP_CMD = 3;           // erase (512-byte sector)
  IAP_ADDRH = addr >> 8;
  IAP_ADDRL = addr & 0xFF;
  IAP_TRIG = 0x5A;
  IAP_TRIG = 0xA5;
  NOP();
  iap_idle();
}

void eeprom_save_baud(void)
{
  EA = 0;
  iap_erase(IAP_ADDR_BASE);
  iap_write(IAP_ADDR_BASE + 0, IAP_MAGIC);
  iap_write(IAP_ADDR_BASE + 1, IAP_MAGIC2);
  iap_write(IAP_ADDR_BASE + 2, bit_period >> 8);
  iap_write(IAP_ADDR_BASE + 3, bit_period & 0xFF);
  iap_write(IAP_ADDR_BASE + 4, txen_per_byte);
  EA = 1;
}

void eeprom_load_baud(void)
{
  if (iap_read(IAP_ADDR_BASE + 0) == IAP_MAGIC &&
      iap_read(IAP_ADDR_BASE + 1) == IAP_MAGIC2) {
    uint16_t bp = ((uint16_t)iap_read(IAP_ADDR_BASE + 2) << 8)
                | iap_read(IAP_ADDR_BASE + 3);
    // Validate: FOSC/115200 <= bp <= FOSC/300
    if (bp < (uint16_t)(FOSC / 115200UL) || bp > (uint16_t)(FOSC / 300UL)) return;
    bit_period = bp;
    uint8_t mode = iap_read(IAP_ADDR_BASE + 4);
    if (mode <= 1) txen_per_byte = mode;
    uint16_t reload = 65536U - (bit_period / 4U);
    TH1 = reload >> 8;
    TL1 = reload & 0xFF;
  }
}

#ifdef DEBUG
// UART debug output helpers
void uart_send_byte(uint8_t dat)
{
  uint16_t tout;
  SBUF = dat;
  for (tout = 60000U; !TI && tout; tout--);
  if (!TI) {
    // TI never set: Timer2/UART TX broken -> blink P3.2 rapidly as error signal
    while (1) {
      uint16_t _d;
      pinTXEN ^= 1;
      for (_d = 0; _d < 5000; _d++) NOP();  // ~500us per half-cycle -> ~1kHz blink
    }
  }
  TI = 0;
}

void uart_send_string(const char *str)
{
  while (*str) {
    uart_send_byte(*str++);
  }
}

void uart_send_hex_byte(uint8_t val)
{
  const char hex[] = "0123456789ABCDEF";
  uart_send_byte(hex[val >> 4]);
  uart_send_byte(hex[val & 0x0F]);
}

void uart_send_hex_word(uint16_t val)
{
  uart_send_hex_byte(val >> 8);
  uart_send_hex_byte(val & 0xFF);
}
#endif /* DEBUG */

void auto_baud_detect(void);

// INT4 ISR: P3.0 falling edge (start bit detection)
void INT4_ISR(void) __interrupt (16)
{
  pinTXEN = 1;
  AUXINTIF &= ~0x40;  // clear INT4 interrupt flag
  INTCLKO &= ~0x40;   // disable INT4
#ifdef DEBUG
  uart_send_byte('I');  // INT4 fired
#endif

  if (!txen_per_byte) {
    // Packet mode: stop timeout timer (next byte arrived in time)
    TR0 = 0;
  }
}

// Timer0 ISR: packet timeout OR baud detection timeout
void Timer0_ISR(void) __interrupt (1)
{
  if (baud_detect_in_progress) {
    // Baud detection timeout mode
    baud_detect_timeout_counter++;
    if (baud_detect_timeout_counter >= BAUD_TIMEOUT_OVERFLOWS) {
      TR0 = 0;  // Stop timer when timeout reached
    }
    // Note: Timer continues counting until timeout is reached
  } else {
    // Normal packet timeout mode
    TR0 = 0;
    if (!txen_per_byte) {
      pinTXEN = 0;
    }
  }
}

// UART1 ISR: RI or FE
void UART1_ISR(void) __interrupt (4)
{
  volatile uint8_t dummy;
  if (FE) {
    FE = 0;
    dummy = SBUF;
    (void)dummy;
    // BREAK = FE + received 0x00 + RXD still Low
    if (dummy == 0x00 && pinRXD == 0) {
#ifdef DEBUG
      uart_send_byte('B');  // BREAK detected → do_auto_baud
#endif
      pinTXEN = 0;
      INTCLKO &= ~0x40;  // disable INT4
      do_auto_baud = 1;  // defer to main loop (Timer0 ISR can't run in ISR context)
      return;            // return immediately; INT4 re-enabled by auto_baud_detect()
    }
#ifdef DEBUG
    uart_send_byte('F');  // FE (non-BREAK)
#endif
    // Non-BREAK FE: re-enable INT4
    pinTXEN = 0;
    AUXINTIF &= ~0x40;
    INTCLKO |= 0x40;
    return;
  }
  if (RI) {
    RI = 0;
    dummy = SBUF;       // read received data
    (void)dummy;
#ifdef DEBUG
    uart_send_byte('R');  // RI fired
#endif

    if (txen_per_byte) {
      // Per-byte mode: turn off TX_EN, re-enable INT4
      // Note: delay removed - RI fires at mid-stop-bit, sufficient for TX_EN deassertion
      pinTXEN = 0;
      AUXINTIF &= ~0x40;
      INTCLKO |= 0x40;
#ifdef DEBUG
      uart_send_byte('X');  // per-byte: TXen cleared
#endif
    } else {
#ifdef DEBUG
      uart_send_byte('P');  // per-packet path
#endif
      // Per-packet mode: re-enable INT4, start timeout timer
      AUXINTIF &= ~0x40;
      INTCLKO |= 0x40;

      // Reconfigure Timer0 for packet timeout (1.5 bit period)
      {
        uint16_t timeout = bit_period + (bit_period >> 1);  // 1.5 * bit_period
        uint16_t reload = 65536 - timeout;
        TH0 = reload >> 8;
        TL0 = reload & 0xFF;
      }
      TF0 = 0;
      TR0 = 1;
    }
  }
}

// Auto-baud detection using BREAK + single byte (0x55/0x35) pattern
// Measures 5 bit periods, reads D5/D6 to determine mode, blinks LED (3 or 5 times), then applies baud rate
void auto_baud_detect(void)
{
  uint16_t t_start, t_end;
  uint16_t old_bit_period = bit_period;
  uint8_t old_txen_per_byte = txen_per_byte;
#ifdef DEBUG
  uint8_t bit_period_measured = 0;  // Track if bit_period was measured
  uint8_t d5d6_measured = 0;        // Track if D5/D6 were read
#endif
  uint8_t d5_bit = 0;               // D5 bit value
  uint8_t d6_bit = 0;               // D6 bit value
  uint16_t last_toggle_counter = 0;

  REN = 0;
  INTCLKO &= ~0x40;
  pinTXEN = 1;  // Hold TXEN during baud rate measurement
  TR0 = 0;

  // Configure Timer0 for 5-second baud detection timeout
  baud_detect_in_progress = 1;
  baud_detect_timeout_counter = 0;
  TH0 = 0;
  TL0 = 0;
  TF0 = 0;
  TR0 = 1;  // Start timeout timer

  TR1 = 0;           // Stop Timer1 UART clock before reconfiguring for measurement

  // Timer1: 16-bit free-run, 1T mode
  TMOD = (TMOD & 0x0F) | 0x00;
  AUXR |= 0x40;  // T1x12=1

  TH1 = 0; TL1 = 0;
  TF1 = 0;
  TR1 = 1;

  // Wait for BREAK to end (RXD goes high)
  while (pinRXD == 0) {
    if (baud_detect_timeout_counter >= BAUD_TIMEOUT_OVERFLOWS) {
      goto abort;
    }
  }

  // Wait for falling edge (start bit)
  while (pinRXD == 1) {
    if (baud_detect_timeout_counter >= BAUD_TIMEOUT_OVERFLOWS) goto abort;
    if ((uint16_t)(baud_detect_timeout_counter - last_toggle_counter) >= TOGGLE_PERIOD_OVERFLOWS) {
      pinTXEN ^= 1;  // Toggle every ~1 second to show loop is running
      last_toggle_counter = baud_detect_timeout_counter;
    }
  }
  t_start = ((uint16_t)TH1 << 8) | TL1;

  // Measure 5 bit periods: S(0)->D0(1)->D1(0)->D2(1)->D3(0)->D4(1)
  // S(0)→D0(1)
  while (pinRXD == 0) {
    if (baud_detect_timeout_counter >= BAUD_TIMEOUT_OVERFLOWS) goto abort;
  }
  // D0(1)→D1(0)
  while (pinRXD == 1) {
    if (baud_detect_timeout_counter >= BAUD_TIMEOUT_OVERFLOWS) goto abort;
  }
  // D1(0)→D2(1)
  while (pinRXD == 0) {
    if (baud_detect_timeout_counter >= BAUD_TIMEOUT_OVERFLOWS) goto abort;
  }
  // D2(1)→D3(0)
  while (pinRXD == 1) {
    if (baud_detect_timeout_counter >= BAUD_TIMEOUT_OVERFLOWS) goto abort;
  }
  // D3(0)→D4(1)
  while (pinRXD == 0) {
    if (baud_detect_timeout_counter >= BAUD_TIMEOUT_OVERFLOWS) goto abort;
  }

  // Capture t_end; compute D5/D6 sample thresholds with shifts BEFORE expensive division
  t_end = ((uint16_t)TH1 << 8) | TL1;
  {
    uint16_t span = t_end - t_start;               // = 5T (subtraction only, ~2 cycles)
    uint16_t thr_d5 = (span >> 2) + (span >> 4);  // = 5/16 * span ≈ 1.5625T (within D5 window)
    uint16_t thr_d6 = span >> 1;                  // = 1/2 * span = 2.5T (exact D6 midpoint)
    uint16_t now_t;

    // Wait until middle of D5 bit period
    do {
      if (baud_detect_timeout_counter >= BAUD_TIMEOUT_OVERFLOWS) goto abort;
      now_t = ((uint16_t)TH1 << 8) | TL1;
    } while ((uint16_t)(now_t - t_end) < thr_d5);
    d5_bit = pinRXD;

    // Wait until middle of D6 bit period
    do {
      if (baud_detect_timeout_counter >= BAUD_TIMEOUT_OVERFLOWS) goto abort;
      now_t = ((uint16_t)TH1 << 8) | TL1;
    } while ((uint16_t)(now_t - t_end) < thr_d6);
    d6_bit = pinRXD;

    // Compute bit_period AFTER timing-critical sampling section
    bit_period = span / 5;
#ifdef DEBUG
    bit_period_measured = 1;
#endif
  }
#ifdef DEBUG
  d5d6_measured = 1;  // Mark as measured
#endif

  // Validate bit pattern and set mode
  if (d5_bit == 0 && d6_bit == 1) {
    // Pattern 01 → 0x55 → per-byte mode
    txen_per_byte = 1;
  } else if (d5_bit == 1 && d6_bit == 0) {
    // Pattern 10 → 0x35 → per-packet mode
    txen_per_byte = 0;
  } else {
    // Invalid pattern → abort
    goto abort;
  }

  // Stop Timer1 (measurement done)
  TR1 = 0;

  // Restore Timer1 for UART baud at old bit_period (needed for BP_PRE debug output)
  {
    uint16_t _r = 65536U - (old_bit_period / 4U);
    TH1 = _r >> 8;
    TL1 = _r & 0xFF;
    TR1 = 1;
  }

  // Stop timeout timer before blinking
  TR0 = 0;
  baud_detect_in_progress = 0;

  // Blink pinTXEN: 3 times for per-byte mode (0x55), 5 times for per-packet mode (0x35)
  // (100ms ON, 100ms OFF)
  {
    uint8_t i;
    uint8_t blink_count = txen_per_byte ? 3 : 5;  // 3 for mode=1, 5 for mode=0
    for (i = 0; i < blink_count; i++) {
      pinTXEN = 1;
      delay_ms(100);
      pinTXEN = 0;
      delay_ms(100);
    }
  }

  // Send BP_PRE at old baud rate for diagnosis (before Timer2 is updated)
#ifdef DEBUG
  uart_send_string("BP_PRE:");
  uart_send_hex_word(bit_period);
  uart_send_string("\r\n");
#endif

  // Apply new baud rate to Timer1
  {
    uint16_t reload = 65536U - (bit_period / 4U);
    TH1 = reload >> 8;
    TL1 = reload & 0xFF;
    // TR1 already running; new reload takes effect at next overflow
  }

  // Send all debug messages at new baud rate
#ifdef DEBUG
  uart_send_string("START\r\n");
  uart_send_string("BP:");
  uart_send_hex_word(bit_period);
  uart_send_string("\r\n");
  uart_send_string("D56:");
  uart_send_hex_byte(d5_bit);
  uart_send_hex_byte(d6_bit);
  uart_send_string("\r\n");
  uart_send_string("OK:MODE=");
  uart_send_hex_byte(txen_per_byte);
  uart_send_string("\r\n");
#endif

  // Save to EEPROM
  eeprom_save_baud();
  goto resume;

abort:
  // CRITICAL: Turn off TXEN immediately to fix timeout issue
  pinTXEN = 0;

  // Stop timers
  TR0 = 0;
  TR1 = 0;
  baud_detect_in_progress = 0;

  // Restore Timer1 for UART baud at old bit_period (for abort debug output)
  {
    uint16_t _r = 65536U - (old_bit_period / 4U);
    TH1 = _r >> 8;
    TL1 = _r & 0xFF;
    TR1 = 1;
  }

  // Display measured bit_period BEFORE restoring old value (UART still at old baud rate)
#ifdef DEBUG
  if (bit_period_measured) {
    uart_send_string("BP:");
    uart_send_hex_word(bit_period);
    uart_send_string("\r\n");
  }
#endif

  // Restore previous baud rate and mode
  bit_period = old_bit_period;
  txen_per_byte = old_txen_per_byte;

  // Send all debug messages at restored baud rate
#ifdef DEBUG
  uart_send_string("START\r\n");

  if (!bit_period_measured) {
    uart_send_string("BP:NONE\r\n");
  }

  if (d5d6_measured) {
    uart_send_string("D56:");
    uart_send_hex_byte(d5_bit);
    uart_send_hex_byte(d6_bit);
    uart_send_string("\r\n");
  } else {
    uart_send_string("D56:NONE\r\n");
  }

  uart_send_string("ABORT:TO=");
  uart_send_hex_word(baud_detect_timeout_counter);
  uart_send_string("\r\n");
#endif

resume:
  // Resume normal operation
  REN = 1;
  pinTXEN = 0;
  AUXINTIF &= ~0x40;
  INTCLKO |= 0x40;  // re-enable INT4
}

void main(void)
{
  // GPIO: P3.2 push-pull output, others quasi-bidirectional
#ifdef DEBUG
  P3M1 = 0x00;
  P3M0 = 0x06;  // P3.1 push-pull (TXD for debug), P3.2 push-pull (TXen)
#else
  P3M1 = 0x02;  // P3.1 high-Z input (externally pulled up)
  P3M0 = 0x04;  // P3.2 push-pull (TXen only)
#endif
  P5M1 = 0x00;
  P5M0 = 0x00;  // P5.4, P5.5 quasi-bidirectional
  pinTXEN = 0;

  // Pull-up on unused pins: P3.1, P3.3, P5.4, P5.5
  P_SW2 |= 0x80;  // enable extended SFR access
  P3PU = 0x0A;     // P3.1, P3.3 pull-up
  P5PU = 0x30;     // P5.4, P5.5 pull-up
  P_SW2 &= ~0x80;

  // PCA: for precise delay (SYSCLK/1, no interrupts)
  CMOD = 0x08;  // CPS=100: SYSCLK/1

  // UART1 pin routing: RXD=P3.0, TXD=P3.1 (explicit, in case ISP left P_SW1 non-default)
  P_SW1 &= ~0xC0;  // S1_S[1:0]=00 -> UART1 on P3.0/P3.1

  // UART1: Mode 1 (8-bit variable baud), REN=1
  // SMOD0=1 -> SCON.7 = FE (frame error flag)
  PCON |= 0x40;   // SMOD0=1
  SCON = 0x50;     // SM1=1, REN=1

  // UART1 baud generator: Timer1 Mode 0 (16-bit auto-reload), 1T
  // S1ST2=0: UART1 uses Timer1 (clear AUXR.0)
  // T1x12=1: 1T mode (AUXR.6)
  // Timer1 Mode 0 (TMOD[7:4]=0x00): 16-bit auto-reload (STC8G specific)
  // baud = FOSC / (4 * (65536 - reload)), reload = 65536 - bit_period/4
  // 9600bps @ 11.0592MHz: reload = 65536 - 288 = 65248 = 0xFEE0
  AUXR &= ~0x01;   // S1ST2=0: UART1 uses Timer1
  AUXR |= 0x40;    // T1x12=1: 1T mode
  TMOD &= 0x0F;    // Timer1 Mode 0 (16-bit auto-reload)
  TH1 = 0xFE;
  TL1 = 0xE0;      // reload = 65248 for 9600bps  (4分周, Timer2 と同じ値)
  TR1 = 1;

  // Load saved baud rate from EEPROM (overrides default if valid)
  eeprom_load_baud();

#ifdef DEBUG
  // Checkpoint A: 2 pulses on P3.2 to confirm code reached uart_send_string
  {
    uint8_t _cp;
    for (_cp = 0; _cp < 2; _cp++) {
      pinTXEN = 1;
      { uint16_t _d; for (_d = 0; _d < 3000; _d++) NOP(); }  // ~300us @11MHz
      pinTXEN = 0;
      { uint16_t _d; for (_d = 0; _d < 3000; _d++) NOP(); }
    }
  }
  uart_send_string("ST:BP=");
  uart_send_hex_word(bit_period);
  uart_send_string(",M=");
  uart_send_hex_byte(txen_per_byte);
  uart_send_string("\r\n");
#endif

  // Packet mode startup notification: toggle pinTXEN 3 times
  if (!txen_per_byte) {
    uint8_t i;
    for (i = 0; i < 3; i++) {
      pinTXEN = 1;
      delay_ms(100);
      pinTXEN = 0;
      delay_ms(100);
    }
  }

  // Timer0: 16-bit mode, 1T (packet timeout)
  TMOD = (TMOD & 0xF0) | 0x00;  // Timer0 Mode 0 (16-bit auto-reload)
  AUXR |= 0x80;                  // T0x12=1 (1T mode)
  TR0 = 0;

  // INT4: falling edge on P3.0
  INTCLKO |= 0x40;  // EX4=1

  // Enable interrupts
  EA = 1;
  ES = 1;            // UART1 interrupt
  ET0 = 1;           // Timer0 interrupt (packet timeout)

  while (1) {
    if (do_auto_baud) {
      do_auto_baud = 0;
      auto_baud_detect();
    }
  }
}
