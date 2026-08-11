#ifndef NORA_UART_DSPIC33CK_DEVICE_H
#define NORA_UART_DSPIC33CK_DEVICE_H

#include <stdbool.h>

#include "nora_uart.h"
#include "nora_uart_dspic33ck_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool present;
    nora_uart_dspic33ck_regs_t regs;
} nora_uart_dspic33ck_device_t;

const nora_uart_dspic33ck_device_t *nora_uart_dspic33ck_get_device(
    nora_uart_instance_t inst);

bool nora_uart_dspic33ck_instance_is_present(
    nora_uart_instance_t inst);

bool nora_uart_dspic33ck_device_set_rx_irq_priority(
    nora_uart_instance_t inst,
    uint8_t priority);

bool nora_uart_dspic33ck_device_set_tx_irq_priority(
    nora_uart_instance_t inst,
    uint8_t priority);

/*
 * RX interrupt flag / enable bit, per instance.
 *
 * These exist as instance switches rather than as (register pointer, mask) pairs
 * for one reason: a constant address with a constant bit compiles to a single
 * bclr/bset, which no interrupt can land inside. A runtime pointer and a runtime
 * mask cannot, so the compiler has to emit load / and / store -- and IFSx/IECx are
 * shared words, so anything the hardware sets between the load and the store is
 * erased by it. On this part IFS0 alone holds T1IF, DMA0/1IF, CCP1/CCT1IF,
 * SPI1RX/TXIF and U1RX/TXIF: the tick, the audio block ISR and the load timer are
 * all in the same word as UART1.
 *
 * Same shape as set_rx_irq_priority() above, and the same shape as hal_dma's
 * dma_irq_clear_flag(). Returns false for an instance this device does not have.
 */
bool nora_uart_dspic33ck_device_rx_irq_flag_clear(
    nora_uart_instance_t inst);

bool nora_uart_dspic33ck_device_rx_irq_enable(
    nora_uart_instance_t inst,
    bool enable);

/* 1 = enabled, 0 = disabled OR no such instance. Callers that need to tell those
 * apart must check nora_uart_dspic33ck_instance_is_present() first -- the previous
 * descriptor-based helper collapsed them the same way. */
uint8_t nora_uart_dspic33ck_device_rx_irq_get_enable(
    nora_uart_instance_t inst);

/*
 * TX interrupt flag / enable bit, per instance. Same instance-switch shape as the
 * RX trio above and for the same atomicity reason (IFS0 holds U1TXIF next to the
 * tick and the audio block ISR).
 *
 * ..._raise_flag() SETS the flag by hand: the TX interrupt asks "the FIFO has
 * room", which is already true when a transfer starts, so nothing would latch it
 * and the engine would wait for a condition that had passed. Used only to enter
 * the handler once at start-up.
 */
bool nora_uart_dspic33ck_device_tx_irq_flag_clear(
    nora_uart_instance_t inst);

bool nora_uart_dspic33ck_device_tx_irq_raise_flag(
    nora_uart_instance_t inst);

bool nora_uart_dspic33ck_device_tx_irq_enable(
    nora_uart_instance_t inst,
    bool enable);

#ifdef __cplusplus
}
#endif

#endif /* NORA_UART_DSPIC33CK_DEVICE_H */
