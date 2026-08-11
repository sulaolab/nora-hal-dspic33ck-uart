/*
 * nora_uart_dspic33ck_isr.c
 * --------------------
 * UART RX interrupt vector wrappers. Each _UxRXInterrupt vector forwards into
 * the RX ISR ring handler; the vector is compiled only when the corresponding
 * RX interrupt flag symbol exists for the selected device.
 *
 * This file is OPTIONAL and neither board in this repo compiles it: both own their
 * console vector, or use polling RX. Add it when you want the HAL to own the RX
 * vectors so the module is self-contained; if the application/integration layer
 * provides its own _UxRXInterrupt instead, compile it with
 * -DNORA_UART_HAL_OWNS_RX_VECTORS=0 to avoid a duplicate-symbol clash. (That macro
 * name is what the code below tests; this comment named its pre-NORA spelling until
 * 2026-08-11.)
 */

#include <xc.h>

#include "nora_uart.h"
#include "nora_uart_dspic33ck_rx_isr_ring.h"

#ifndef NORA_UART_HAL_OWNS_RX_VECTORS
#define NORA_UART_HAL_OWNS_RX_VECTORS 1
#endif

#if NORA_UART_HAL_OWNS_RX_VECTORS

#if defined(_U1RXIF)
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{
    nora_uart_rx_irq_handler(NORA_UART_INST_1);
}
#endif

#if defined(_U2RXIF)
void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt(void)
{
    nora_uart_rx_irq_handler(NORA_UART_INST_2);
}
#endif

#if defined(_U3RXIF)
void __attribute__((interrupt, no_auto_psv)) _U3RXInterrupt(void)
{
    nora_uart_rx_irq_handler(NORA_UART_INST_3);
}
#endif

#endif /* NORA_UART_HAL_OWNS_RX_VECTORS */

/*
 * TX interrupt vector wrappers for the asynchronous TX transfer engine.
 *
 * OFF BY DEFAULT, unlike the RX vectors above, and that asymmetry is deliberate:
 * the RX ring is the receive path this file exists to serve, while async TX is a
 * second, independent opt-in. (An earlier version of this note called the ring "the
 * console path every configuration uses", which is not so -- the console on the
 * hardware-validated board is polling RX, and no configuration in this tree selects
 * the ring.) Defining these vectors references nora_uart_tx_irq_handler(), which
 * anchors the whole TX engine against remove-unused-sections -- so a build that
 * never calls nora_uart_tx_start() would pay ROM for code it cannot reach. The
 * EV88G73A configuration sits at 96 % of flash, which is where that matters.
 *
 * Async TX users build this file with NORA_UART_HAL_OWNS_TX_VECTORS=1, or supply
 * their own _UxTXInterrupt that forwards into nora_uart_tx_irq_handler(). This is
 * the same contract the dsPIC33AK HAL states ("requires the application to route
 * the device TX interrupt vector").
 */
#ifndef NORA_UART_HAL_OWNS_TX_VECTORS
#define NORA_UART_HAL_OWNS_TX_VECTORS 0
#endif

#if NORA_UART_HAL_OWNS_TX_VECTORS

#if defined(_U1TXIF)
void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void)
{
    nora_uart_tx_irq_handler(NORA_UART_INST_1);
}
#endif

#if defined(_U2TXIF)
void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void)
{
    nora_uart_tx_irq_handler(NORA_UART_INST_2);
}
#endif

#if defined(_U3TXIF)
void __attribute__((interrupt, no_auto_psv)) _U3TXInterrupt(void)
{
    nora_uart_tx_irq_handler(NORA_UART_INST_3);
}
#endif

#endif /* NORA_UART_HAL_OWNS_TX_VECTORS */
