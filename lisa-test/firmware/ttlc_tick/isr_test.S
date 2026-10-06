
    .section .text
    .public porta_isr

#define IRQ_STATUS 0x20F

porta_isr:
   xchg_ia
   xchg_ia
   ldi  0x10
   sta  IRQ_STATUS
   rets

