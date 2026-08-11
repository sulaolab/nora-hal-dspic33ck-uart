#ifndef NORA_UART_DSPIC33CK_RX_ISR_RING_H
#define NORA_UART_DSPIC33CK_RX_ISR_RING_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nora_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * UART RX interrupt-driven ring HAL (dsPIC33CK).
 *
 * Provides the RX ISR ring core: a single-producer (ISR) / single-consumer
 * (reader) software ring fed by draining the RX FIFO, plus the RX ISR ring
 * runtime status counters.
 *
 * Design policy:
 *   - No printf / halt / blocking calls, no application/board dependencies.
 *   - No dynamic allocation; the ring buffer storage is caller-provided.
 *   - The scattered RX interrupt Flag/Enable/Priority bits are isolated in the
 *     device layer (nora_uart_dspic33ck_device.c) and the reg helpers.
 *
 * Interrupt vector ownership:
 *   nora_uart_rx_irq_handler() is an ordinary function, NOT an interrupt
 *   vector declaration. The _UxRXInterrupt vector wrappers that forward into it
 *   live in nora_uart_dspic33ck_isr.c (this repo is a self-contained lab; if the
 *   application owns the vectors instead, build with
 *   NORA_UART_HAL_OWNS_RX_VECTORS=0).
 */

/* The NORA_UART_EVENT_* bit-flags this RX handler collects now live in
 * nora_uart.h with the rest of the async transfer model -- they are part of the
 * portable surface, not of this chip-tagged backend. */

typedef struct
{
    uint8_t *buffer;        /* caller-provided ring storage (not owned)        */
    uint16_t buffer_size;   /* size of buffer in bytes; must be >= 2           */
    uint8_t irq_priority;   /* CPU interrupt priority for the RX interrupt     */
} nora_uart_dspic33ck_rx_isr_config_t;

/*
 * RX ISR ring runtime status snapshot (accumulated counters, distinct from the
 * nora_uart_status_t function return code).
 */
typedef struct
{
    uint32_t rx_isr_count;
    uint32_t rx_byte_count;
    uint32_t rx_fifo_overflow_count;
    uint32_t framing_error_count;
    uint32_t parity_error_count;
    uint32_t rx_ring_overflow_count;
    uint32_t rx_max_drain_count;
    /*
     * How many times the READER found the receiver stalled and restarted it --
     * see nora_uart_dspic33ck_rx_isr_service() for what a stall is and why the ISR
     * alone cannot end one. Both stay 0 on a board that never stalls, so a
     * non-zero value is the evidence, not the alarm.
     *
     * rx_ie_lost_count is the sharper of the two: it counts stalls found with
     * the RX interrupt ENABLE bit clear on a configured instance, which nothing
     * in this HAL does deliberately -- i.e. it separates "the enable bit was
     * lost" from "the FIFO overran while the ISR was away".
     */
    uint32_t rx_stall_recovery_count;
    uint32_t rx_ie_lost_count;
} nora_uart_dspic33ck_rx_isr_status_t;

/*
 * Configure the RX ISR ring for an instance. Validates the instance/config,
 * binds the caller-provided buffer, resets the ring and counters, sets the RX
 * FIFO watermark to interrupt on >= 1 byte, and programs the RX interrupt
 * priority. The RX interrupt is left DISABLED; call _rx_isr_enable() to start it.
 *
 *   NORA_UART_ERR_INVALID_ARG     config/buffer NULL or buffer_size < 2
 *   NORA_UART_ERR_NOT_PRESENT     instance not present on this device
 *   NORA_UART_ERR_NOT_INITIALIZED UART not initialized yet
 *   NORA_UART_ERR_UNSUPPORTED     no RX interrupt mapping for this instance
 *   NORA_UART_OK                  configured
 */
nora_uart_status_t nora_uart_dspic33ck_rx_isr_config(
    nora_uart_instance_t inst,
    const nora_uart_dspic33ck_rx_isr_config_t *config);

/* Internal HAL integration hook for the backend-aware public snapshot. */
bool nora_uart_dspic33ck_rx_isr_is_configured(
    nora_uart_instance_t inst);

/* Enable the RX interrupt (instance must be configured and initialized). */
nora_uart_status_t nora_uart_dspic33ck_rx_isr_enable(
    nora_uart_instance_t inst);

/* Disable the RX interrupt (safe direction; allowed even if not configured). */
nora_uart_status_t nora_uart_dspic33ck_rx_isr_disable(
    nora_uart_instance_t inst);

/*
 * End a stalled reception if there is one, and report whether it did.
 *
 * WHY THIS EXISTS -- and it is a hole in the design above, not a refinement.
 * Everything that can restart a stopped receiver used to live in the ISR: OERR is
 * R/W and the receiver accepts nothing more until it is cleared, and the ISR is
 * what clears it. But an ISR that is not entered clears nothing, and there are two
 * ways to reach that state, both of which leave TX perfectly healthy so the board
 * looks alive:
 *
 *   1. OERR latches while the ISR is away (a higher-priority handler, or any
 *      window with the RX interrupt disabled).  Reception stops, so no further
 *      byte -- and therefore no further interrupt -- ever arrives.  The ISR is not
 *      late, it is never called again.
 *   2. The RX interrupt ENABLE bit is lost.  The Flag/Enable bits are written with
 *      read-modify-write on registers shared with every other peripheral in the
 *      same IFS/IEC word, so an interrupt landing between the read and the write
 *      makes the write-back stale.  MEASURED CONSEQUENCE: a console that echoes
 *      nothing while it keeps printing, recoverable only by reset.
 *
 * Both are ended by the READER, which is always running: this looks for the
 * signature (OERR set, or bytes in the FIFO that the ring never received), and
 * only then takes the ISR's own critical section to drain the FIFO, clear OERR and
 * re-enable the interrupt.  In the steady state it is two register READS and no
 * writes, so it is not a poll loop bolted onto a hot path.
 *
 * nora_uart_dspic33ck_rx_isr_ready() and _read_byte() call this when the ring is empty,
 * which is why a caller using them needs no new code.  It is exposed because a
 * caller that services RX some other way still needs a way out of a stall.
 *
 * ONE CONSEQUENCE, stated rather than hidden: a stall and a DELIBERATE disable look
 * the same from here, so recovery re-enables the RX interrupt.  Asking _ready() for
 * a byte therefore means "RX is wanted"; a caller that wants reception off must call
 * _rx_isr_disable() and stop polling, which is what that function is for.
 */
bool nora_uart_dspic33ck_rx_isr_service(
    nora_uart_instance_t inst);

/*
 * True when the ring holds at least one buffered byte -- and, when it does not,
 * this is also where a stalled receiver is restarted (see _rx_isr_service()), so a
 * byte the FIFO is holding onto because the ISR stopped running is still reported.
 */
bool nora_uart_dspic33ck_rx_isr_ready(
    nora_uart_instance_t inst);

/*
 * Pop one byte from the ring.
 *   NORA_UART_ERR_INVALID_ARG  data == NULL
 *   NORA_UART_ERR_RX_EMPTY     ring empty
 *   NORA_UART_OK               one byte written to *data
 */
nora_uart_status_t nora_uart_dspic33ck_rx_isr_read_byte(
    nora_uart_instance_t inst,
    uint8_t *data);

/* Drop buffered ring contents and drain the hardware RX FIFO. */
void nora_uart_dspic33ck_rx_isr_flush(
    nora_uart_instance_t inst);

/* Snapshot the RX ISR ring runtime status counters (atomic vs the ISR). */
void nora_uart_dspic33ck_rx_isr_status_get(
    nora_uart_instance_t inst,
    nora_uart_dspic33ck_rx_isr_status_t *status);

/* Zero the RX ISR ring runtime status counters (atomic vs the ISR). */
void nora_uart_dspic33ck_rx_isr_status_clear(
    nora_uart_instance_t inst);

/*
 * nora_uart_rx_irq_handler() -- the RX ISR body this file implements -- is
 * declared in nora_uart.h alongside nora_uart_tx_irq_handler(), because the
 * interrupt entry points are part of the portable surface. It clears the RX
 * interrupt flag, drains the RX FIFO into the ring, and counts/clears the latched
 * RX error flags. No printf / blocking.
 */

/*
 * Internal HAL hooks (glue to an optional asynchronous transfer engine). NOT
 * part of the application-facing API; do NOT call from application code.
 *
 * These carry weak no-op defaults in nora_uart_dspic33ck_rx_isr_ring.c (feed returns
 * false so the byte falls through to the ring; notify does nothing), so the ring links
 * whether or not the async engine is in the build. nora_uart_dspic33ck.c provides the
 * strong definitions that override them, which is why a build with both routes each
 * byte to an active async RX transfer first. (This note used to say no async engine
 * was shipped yet; it has been since the AK transfer model was ported.)
 */

/* Offer one freshly received byte to an active async RX transfer. Returns true
 * when the byte was consumed (caller must NOT also push it to the ring). */
bool nora_uart_dspic33ck_async_rx_feed(
    nora_uart_instance_t inst,
    uint8_t byte);

/* Forward RX-side event bits (errors / RX_READY) to the registered callback. */
void nora_uart_dspic33ck_async_notify_events(
    nora_uart_instance_t inst,
    uint32_t events);

#ifdef __cplusplus
}
#endif

#endif /* NORA_UART_DSPIC33CK_RX_ISR_RING_H */
