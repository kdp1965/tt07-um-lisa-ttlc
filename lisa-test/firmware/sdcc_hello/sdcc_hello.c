/*
 * sdcc_hello.c - the bring-up demo rewritten in C for sdcc -mlisa.
 *
 * Prints a banner at reset and then echoes the UART:  '?' prints the owl,
 * 's' prints the loop count and the sum of the received bytes with a
 * working %u (the hand-written printf of the original demo printed "d").
 * Build: make   (needs lisa-tools/sdcc-lisa, see Makefile)
 */
#include <stdio.h>

__sfr __at(0x210) UART_RXTX;
__sfr __at(0x211) UART_STATUS;
__sfr __at(0x201) PORTB;
__sfr __at(0x20c) TIMER1_CTRL;

#define UART_RX_AVAIL    0x01
#define UART_TX_EMPTY    0x02
#define TIMER_ROLLOVER   0x80

int putchar(int c)
{
  while (!(UART_STATUS & UART_TX_EMPTY))
    ;
  UART_RXTX = c;
  return c;
}

static const char *const owl[] = {
  "     .{{{}}}}}}.",
  "    {{{{{}}}}}}}.",
  "   {{{{  {{{{{}}}}",
  "  }}}}} _   _ {{{{{",
  "  }}}}  6   6  }}}}",
  " {{{{C    ^    {{{{",
  "}}}}}}\\  '='  /}}}}}",
  "{{{{{{{;.___.;}}}}}}",
  " {{{{{{{)   (}}}}}}'",
  "  ''\"''':   :''''''",
  "  jgs    `@` ",
  0,
};

static void print_lisa(void)
{
  const char *const *p;
  putchar('\n');
  for (p = owl; *p; p++)
    printf("%s\n", *p);
  printf("\nHello from TT07 LISA, compiled by sdcc -mlisa\n");
}

void main(void)
{
  unsigned int count = 0;
  unsigned int sum = 0;
  unsigned char ticks = 0;
  unsigned char c;

  PORTB = 0x4f;
  printf("\nsdcc_hello ready: ? = banner, s = count and sum\n");
  for (;;)
    {
      count++;
      if (TIMER1_CTRL & TIMER_ROLLOVER)
        {
          if (++ticks == 4)
            {
              ticks = 0;
              PORTB ^= 0x08;
            }
        }
      if (UART_STATUS & UART_RX_AVAIL)
        {
          c = UART_RXTX;
          putchar(c);
          sum += c;
          if (c == '\r')
            putchar('\n');
          if (c == '?')
            print_lisa();
          if (c == 's')
            printf("\nCount: %u sum: %u\n", count, sum);
        }
    }
}
