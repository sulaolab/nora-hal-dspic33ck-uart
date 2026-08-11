#ifndef NORA_UART_H
#define NORA_UART_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Minimal dsPIC33CK UART byte-stream HAL.
 *
 * API shape intentionally follows the dsPIC33AK UART HAL where practical, and
 * the names now sit in the shared NORA namespace (silicon side tagged
 * _dspic33ck). AK's asynchronous transfer API is ported (see the bottom of this
 * header); what is still CK-only is the BRGH / BCLKSEL configuration
 * (high_speed, clock_source), which AK's UART has no equivalent for.
 *
 * The RX backend IS selectable from this portable config (rx_mode and the
 * rx_ring_* / rx_irq_priority fields), so an application written against the
 * dsPIC33AK HAL's ISR-ring configuration compiles and runs here unchanged. The
 * chip-tagged nora_uart_dspic33ck_rx_isr_*() entry points stay available for
 * code that wants the ring's own diagnostics.
 */

typedef enum {
    NORA_UART_INST_1 = 0,
    NORA_UART_INST_2,
    NORA_UART_INST_3,
    NORA_UART_INST_COUNT
} nora_uart_instance_t;

typedef enum {
    NORA_UART_OK = 0,
    NORA_UART_ERR_INVALID_ARG,
    NORA_UART_ERR_NOT_PRESENT,
    NORA_UART_ERR_NOT_INITIALIZED,
    /* Placed here, not appended, so the enumerator VALUES stay identical to the
     * dsPIC33AK HAL's. Nothing in this repo indexes an array by this enum, so the
     * insertion is source-compatible; a consumer that PRINTS the raw number sees
     * _ERR_TIMEOUT and later codes shift by one. */
    NORA_UART_ERR_BUSY,
    NORA_UART_ERR_TIMEOUT,
    NORA_UART_ERR_RX_EMPTY,
    NORA_UART_ERR_TX_FULL,
    NORA_UART_ERR_OVERRUN,
    NORA_UART_ERR_FRAMING,
    NORA_UART_ERR_PARITY,
    NORA_UART_ERR_UNSUPPORTED
} nora_uart_status_t;

typedef uint32_t (*nora_uart_get_ms_fn)(void);

typedef enum {
    NORA_UART_PARITY_NONE = 0,
    NORA_UART_PARITY_EVEN,
    NORA_UART_PARITY_ODD
} nora_uart_parity_t;

/* RX backend for an instance. Selected through nora_uart_config_t.rx_mode and
 * reported back by nora_uart_rx_status_t.rx_mode, so the same enum both sets
 * and reports the backend -- as on the dsPIC33AK HAL. In ISR_RING mode
 * rx_ready() / read_byte() / rx_flush() operate on the ring instead of the
 * hardware FIFO. */
typedef enum {
    NORA_UART_RX_MODE_POLLING = 0,
    NORA_UART_RX_MODE_ISR_RING
} nora_uart_rx_mode_t;

typedef enum {
    NORA_UART_BCLKSEL_FOSC_DIV2 = 0,
    NORA_UART_BCLKSEL_FOSC = 1,
    NORA_UART_BCLKSEL_AFPLLO = 2,
    NORA_UART_BCLKSEL_REFCLK = 3
} nora_uart_bclksel_t;

typedef struct {
    uint32_t uart_clk_hz;
    uint32_t baudrate;
    uint32_t timeout_ms;

    /*
     * Optional millisecond tick callback for timeout handling.
     * If get_ms is NULL, timeout handling is disabled.
     */
    nora_uart_get_ms_fn get_ms;

    uint8_t data_bits;
    uint8_t stop_bits;
    nora_uart_parity_t parity;
    bool high_speed;
    nora_uart_bclksel_t clock_source;
    bool enable_tx;
    bool enable_rx;

    /* RX backend (see nora_uart_rx_mode_t). The rx_ring_* / rx_irq_priority
     * fields are used only when rx_mode == NORA_UART_RX_MODE_ISR_RING. The ring
     * buffer storage is caller-provided so the HAL holds no implicit RAM.
     *
     * Field names and semantics match the dsPIC33AK HAL, so a config literal is
     * portable in both directions. A zero-initialized config selects POLLING,
     * which is what every pre-existing CK caller gets.
     *
     * ISR_RING additionally requires enable_rx, a buffer of at least 2 bytes, a
     * non-zero irq_priority, an instance with an RX interrupt mapping, and the
     * ISR-ring object linked into the build; nora_uart_init() reports
     * NORA_UART_ERR_UNSUPPORTED (or _ERR_INVALID_ARG) instead of falling back to
     * polling, because a silent fallback would look like a working ring. */
    nora_uart_rx_mode_t rx_mode;
    uint8_t  *rx_ring_buffer;
    uint16_t  rx_ring_buffer_size;
    uint8_t   rx_irq_priority;

    /*
     * CPU interrupt priority for the TX interrupt, used only by the non-blocking
     * TX transfer engine (nora_uart_tx_start). It is programmed at init and is
     * independent of rx_irq_priority. Builds that never call the async TX API may
     * leave this at any value; the TX interrupt stays disabled until a transfer
     * starts. A value of 0 disables the TX interrupt on this CPU, so an async-TX
     * user must set a non-zero priority here.
     */
    uint8_t   tx_irq_priority;
} nora_uart_config_t;

/*
 * RX runtime status snapshot (backend-aware).
 *
 * This is different from nora_uart_status_t:
 *   - nora_uart_status_t      is a function return code.
 *   - nora_uart_rx_status_t   is runtime RX state / counters.
 *
 * In ISR ring mode the ISR-ring counters are copied from the existing backend.
 * Polling mode has no ISR-ring counters, but still reports the bytes handed to the
 * reader, the framing/parity errors it discarded, and the OERR recoveries performed
 * by nora_uart_rx_ready().
 */
typedef struct {
    nora_uart_rx_mode_t rx_mode;

    uint32_t rx_isr_count;
    uint32_t rx_byte_count;
    uint32_t rx_fifo_overflow_count;
    uint32_t framing_error_count;
    uint32_t parity_error_count;
    uint32_t autobaud_overflow_count;
    uint32_t tx_collision_count;
    uint32_t rx_ring_overflow_count;
    uint32_t rx_max_drain_count;

    uint32_t rx_stall_recovery_count;
    uint32_t rx_ie_lost_count;
    uint32_t rx_overrun_recovered_count;
} nora_uart_rx_status_t;

nora_uart_status_t nora_uart_init(
    nora_uart_instance_t inst,
    const nora_uart_config_t *config);

nora_uart_status_t nora_uart_deinit(
    nora_uart_instance_t inst);

bool nora_uart_is_present(
    nora_uart_instance_t inst);

bool nora_uart_is_initialized(
    nora_uart_instance_t inst);

/*
 * True when the RX FIFO holds at least one byte.
 *
 * It also ENDS A LATCHED OVERRUN when it finds one with the FIFO empty, and that is
 * not a side effect to be tidied away later: OERR halts the receiver, and on this
 * path only nora_uart_read_byte() clears it -- which a polled reader never
 * reaches once this function starts answering false.  Without the clear here, one
 * overrun ends reception for the life of the image while TX keeps working.  See the
 * comment on the implementation for the measured failure. Each recovery is
 * reported by nora_uart_rx_status_get().rx_overrun_recovered_count.
 */
bool nora_uart_rx_ready(
    nora_uart_instance_t inst);

bool nora_uart_tx_ready(
    nora_uart_instance_t inst);

bool nora_uart_tx_done(
    nora_uart_instance_t inst);

nora_uart_status_t nora_uart_write_byte(
    nora_uart_instance_t inst,
    uint8_t data);

nora_uart_status_t nora_uart_read_byte(
    nora_uart_instance_t inst,
    uint8_t *data);

size_t nora_uart_write(
    nora_uart_instance_t inst,
    const void *data,
    size_t len);

size_t nora_uart_read(
    nora_uart_instance_t inst,
    void *data,
    size_t len);

void nora_uart_rx_flush(
    nora_uart_instance_t inst);

/*
 * Backend-aware RX status snapshot / clear.
 *
 * ISR-ring mode reports and clears the existing RX ISR-ring counters; polling
 * mode reports zero for those counters. In both modes the snapshot includes
 * OERR recoveries performed by nora_uart_rx_ready().
 *
 * Returns NORA_UART_ERR_INVALID_ARG (status NULL), _ERR_NOT_PRESENT,
 * _ERR_NOT_INITIALIZED, or NORA_UART_OK.
 */
nora_uart_status_t nora_uart_rx_status_get(
    nora_uart_instance_t inst,
    nora_uart_rx_status_t *status);

nora_uart_status_t nora_uart_rx_status_clear(
    nora_uart_instance_t inst);

/*
 * RX REGISTER EVIDENCE -- the raw words, for the failure that cannot be asked about.
 *
 * Added because the counters above cannot answer it on a POLLING console, which is
 * what this project's console actually is: nothing calls the ISR-ring configure
 * entry point, so the ring-only counters are structurally zero and only
 * rx_byte_count, framing_error_count, parity_error_count and
 * rx_overrun_recovered_count are live -- and those four are live precisely because
 * the polled path was taught to count them. MEASURED on EV88G73A 2026-08-10, before
 * that: a `?tq` that was received AND echoed left rx_isr_count and rx_byte_count
 * at 0, which is why the byte count is now counted by the polled reader too.
 *
 * The console went deaf once and the only recovery is a reprogram, which resets
 * everything; so the state has to be printed continuously by the TX half, which
 * keeps working. These three words are what settle it, and each answers a
 * question no counter can:
 *   MODE -- UARTEN and URXEN. If URXEN has gone to 0 the receiver was switched
 *           off, and the whole search moves to whatever wrote MODE.
 *   STA  -- OERR / FERR / PERR. A latched OERR halts reception; the polling path
 *           clears it and counts the rescue, so OERR standing set in the print
 *           means the rescue itself is not running.
 *   STAH -- URXBE. Empty receive buffer while the host is sending says no byte
 *           is arriving at the pin at all -- the nEDBG CDC bridge or the PPS
 *           input mapping, not the UART.
 *
 * Raw words rather than decoded flags on purpose: decoding costs flash on a
 * 99 %-full part, and a raw word cannot be wrong about a bit this file did not
 * think to name.
 *
 * Returns NORA_UART_ERR_INVALID_ARG (out NULL), _ERR_NOT_PRESENT,
 * _ERR_NOT_INITIALIZED, or NORA_UART_OK.
 */
typedef struct {
    uint16_t mode;   /* UxMODE  */
    uint16_t sta;    /* UxSTA   */
    uint16_t stah;   /* UxSTAH  */
} nora_uart_rx_regs_t;

nora_uart_status_t nora_uart_rx_regs_get(
    nora_uart_instance_t inst,
    nora_uart_rx_regs_t *out);

nora_uart_status_t nora_uart_tx_enable(
    nora_uart_instance_t inst,
    bool enable);

nora_uart_status_t nora_uart_rx_enable(
    nora_uart_instance_t inst,
    bool enable);

/*
 * Read side of nora_uart_tx_enable() / nora_uart_rx_enable(): true when the
 * transmitter / receiver is currently enabled. False for an absent or
 * uninitialized instance, so a false answer means "not transmitting/receiving"
 * in every case and needs no separate presence check.
 *
 * The state is read back from the peripheral (MODE.UTXEN / MODE.URXEN), not from
 * a shadow variable, so it stays correct even if something outside this HAL
 * touched the enable bit. The dsPIC33AK HAL answers from its own shadow state;
 * the contract is the same.
 */
bool nora_uart_tx_is_enabled(
    nora_uart_instance_t inst);

bool nora_uart_rx_is_enabled(
    nora_uart_instance_t inst);

/* ----- Baud rate (re)configuration ---------------------------------------- */

/*
 * Recompute and apply the baud divisor from uart_clk_hz / baudrate and remember
 * both in the instance context. The sample-rate mode (BRGH) is NOT changed: the
 * divisor is recomputed against whatever mode nora_uart_init() left in place, so
 * the byte framing stays as configured.
 *
 * Rejected with NORA_UART_ERR_BUSY while a byte is still being shifted out or
 * the TX buffer is not empty, so an in-progress transmission is never silently
 * reconfigured mid-character. _ERR_INVALID_ARG on a zero clock or baud, or when
 * the requested pair needs a divisor this UART cannot express.
 *
 * Also rejected with _ERR_BUSY while an asynchronous TX or RX transfer is active
 * (matching dsPIC33AK). This note used to say the opposite -- that this HAL had no
 * asynchronous transfer API -- which stopped being true when the AK transfer model
 * was ported; the implementation has always checked both. An RX ISR ring, if
 * configured, keeps
 * receiving at the OLD rate until this call returns -- change the baud rate only
 * when the link is quiet in both directions.
 */
nora_uart_status_t nora_uart_set_baudrate(
    nora_uart_instance_t inst,
    uint32_t uart_clk_hz,
    uint32_t baudrate);

/* Last baud rate applied (0 if the instance is not initialized). */
uint32_t nora_uart_get_baudrate(
    nora_uart_instance_t inst);

/* ========================================================================== */
/* Asynchronous Transfer Model (event-driven, non-blocking)                   */
/* ========================================================================== */

/*
 * Optional asynchronous transfer layer for upper layers that want a non-blocking
 * Send/Receive model with completion/error events (for example a CMSIS-style
 * USART wrapper built on top of this HAL).
 *
 * This layer is purely additive and does NOT replace or change the byte-stream
 * API above. The blocking write byte path, the non-blocking read byte path and
 * the RX ISR ring keep working exactly as before; the async transfer engine is
 * inert until nora_uart_tx_start(), nora_uart_rx_start(), or
 * nora_uart_rx_start_clean() is called.
 *
 * Intentionally generic: the events and the API below describe a UART, not any
 * specific middleware. No ARM_USART_* / ARM_DRIVER_* names appear here.
 *
 * Backend requirements:
 *   - Async TX requires TX enabled and a non-zero tx_irq_priority; otherwise
 *     nora_uart_tx_start() returns NORA_UART_ERR_UNSUPPORTED (a transfer with no
 *     servicing interrupt would never complete). It also requires the device TX
 *     interrupt vector to reach nora_uart_tx_irq_handler(): unlike the RX
 *     vectors, this HAL does NOT own the TX vectors by default -- build
 *     nora_uart_dspic33ck_isr.c with NORA_UART_HAL_OWNS_TX_VECTORS=1, or supply
 *     your own _UxTXInterrupt that forwards into the handler. The default is off
 *     so a build that never uses async TX carries none of this code.
 *   - Async RX requires RX enabled and NORA_UART_RX_MODE_ISR_RING (the RX ISR
 *     feeds the async buffer); otherwise nora_uart_rx_start() returns
 *     NORA_UART_ERR_UNSUPPORTED.
 *   - nora_uart_tx_enable(false) / nora_uart_rx_enable(false) return
 *     NORA_UART_ERR_BUSY while an async transfer is active, so a transfer is
 *     never stranded by disabling its line mid-flight.
 */

/* Event bit-flags reported through the registered callback. Multiple bits may be
 * OR'd together in a single notification. Bit numbering matches the dsPIC33AK
 * UART HAL.
 *
 * SEND_COMPLETE means the driver has submitted every TX byte to the hardware
 * (FIFO/register) - the CMSIS ARM_USART_EVENT_SEND_COMPLETE sense, NOT physical
 * shift-register-empty. It is intentionally NOT named TX_COMPLETE to avoid being
 * read as the CMSIS ARM_USART_EVENT_TX_COMPLETE (line idle / shifter empty). Use
 * the existing nora_uart_tx_done() to confirm physical transmit completion. */
#define NORA_UART_EVENT_SEND_COMPLETE     (1u << 0)  /* all TX data submitted     */
#define NORA_UART_EVENT_RX_COMPLETE       (1u << 1)  /* requested RX length got   */
#define NORA_UART_EVENT_RX_READY          (1u << 2)  /* unsolicited RX -> ring    */
#define NORA_UART_EVENT_RX_OVERFLOW       (1u << 3)  /* software RX ring overflow */
#define NORA_UART_EVENT_RX_FRAMING_ERROR  (1u << 4)  /* UxSTA FERR                */
#define NORA_UART_EVENT_RX_PARITY_ERROR   (1u << 5)  /* UxSTA PERR                */
#define NORA_UART_EVENT_RX_OVERRUN_ERROR  (1u << 6)  /* hardware RX FIFO overrun  */

/*
 * Event callback. Invoked with the OR'd event bits for the instance and the
 * user_data pointer registered alongside it.
 *
 * The callback is invoked from interrupt context (TX/RX ISR). Keep it short and
 * non-blocking; do not call back into a blocking HAL API from inside it.
 */
typedef void (*nora_uart_event_callback_t)(
    nora_uart_instance_t inst,
    uint32_t events,
    void *user_data);

/*
 * Register (or clear, with callback == NULL) the event callback for an instance.
 * Valid before or after init; nora_uart_init()/deinit() clear it, so call this
 * after init. Returns _ERR_INVALID_ARG / _ERR_NOT_PRESENT or _OK.
 */
nora_uart_status_t nora_uart_set_callback(
    nora_uart_instance_t inst,
    nora_uart_event_callback_t callback,
    void *user_data);

/* ----- Non-blocking TX transfer ------------------------------------------- */

/*
 * Start a non-blocking TX transfer of length bytes from data. Returns
 * immediately; the bytes are pushed to the TX FIFO from the TX interrupt. When
 * the last byte has been submitted to the hardware, NORA_UART_EVENT_SEND_COMPLETE
 * is reported via the callback (SEND_COMPLETE sense, not physical shift-out; use
 * nora_uart_tx_done() for that). data must remain valid until completion or
 * abort.
 *
 *   _ERR_INVALID_ARG  data == NULL or length == 0
 *   _ERR_BUSY         a TX transfer is already active
 *   _ERR_UNSUPPORTED  TX disabled, tx_irq_priority == 0, or no TX interrupt
 *                     mapping for this instance
 *   _ERR_NOT_INITIALIZED / _ERR_NOT_PRESENT as usual
 *
 * This is independent of nora_uart_write()/_write_byte(); do not mix a blocking
 * write with an active async TX transfer on the same instance.
 */
nora_uart_status_t nora_uart_tx_start(
    nora_uart_instance_t inst,
    const uint8_t *data,
    size_t length);

/* Abort an active TX transfer. Already-submitted bytes still go out; no
 * SEND_COMPLETE event is reported. Safe to call when idle. */
nora_uart_status_t nora_uart_tx_abort(
    nora_uart_instance_t inst);

/* Number of bytes submitted by the current/last TX transfer. */
size_t nora_uart_tx_count_get(
    nora_uart_instance_t inst);

bool nora_uart_tx_is_busy(
    nora_uart_instance_t inst);

/* ----- Non-blocking RX transfer ------------------------------------------- */

/*
 * Register a non-blocking RX transfer: up to length bytes are stored into data as
 * they arrive (fed from the RX ISR, ISR ring mode only). Returns immediately.
 * When length bytes have been received, NORA_UART_EVENT_RX_COMPLETE is reported
 * via the callback. data must remain valid until completion or abort.
 *
 * While a transfer is active, incoming bytes go to the async buffer instead of
 * the RX ISR ring (so nora_uart_read_byte() will not see them).
 *
 * BY DESIGN, the transfer captures only bytes that arrive AFTER this call: bytes
 * already buffered in the RX ISR ring before rx_start() are NOT drained into the
 * async buffer and stay readable via the byte-stream API. A caller that wants the
 * async transfer to start from a clean slate should use
 * nora_uart_rx_start_clean(), which avoids a flush/start race window.
 *
 *   _ERR_INVALID_ARG  data == NULL or length == 0
 *   _ERR_BUSY         an RX transfer is already active
 *   _ERR_UNSUPPORTED  instance is not in ISR ring RX mode, or RX is disabled
 *   _ERR_NOT_INITIALIZED / _ERR_NOT_PRESENT as usual
 */
nora_uart_status_t nora_uart_rx_start(
    nora_uart_instance_t inst,
    uint8_t *data,
    size_t length);

/*
 * Start a clean non-blocking RX transfer. Bytes already buffered in the RX ISR
 * ring or hardware FIFO are discarded, then the async RX descriptor is armed
 * while the RX interrupt is held disabled.
 *
 * Bytes that arrive after the clean arm are captured by the new async transfer.
 * The exact boundary is the end of the FIFO drain / descriptor publication, not
 * the function entry point.
 *
 * This is intended for APIs such as CMSIS USART Receive(), where each receive
 * operation should observe only bytes that arrive for that operation.
 *
 * Return codes match nora_uart_rx_start(); _ERR_UNSUPPORTED is also returned if
 * the RX ISR is not currently enabled.
 */
nora_uart_status_t nora_uart_rx_start_clean(
    nora_uart_instance_t inst,
    uint8_t *data,
    size_t length);

/* Abort an active RX transfer. No RX_COMPLETE event is reported. Already-stored
 * bytes stay in the caller buffer (count readable via _rx_count_get). Safe when
 * idle. Subsequent incoming bytes resume going to the RX ISR ring. */
nora_uart_status_t nora_uart_rx_abort(
    nora_uart_instance_t inst);

/* Number of bytes stored by the current/last RX transfer. */
size_t nora_uart_rx_count_get(
    nora_uart_instance_t inst);

bool nora_uart_rx_is_busy(
    nora_uart_instance_t inst);

/* ----- Interrupt entry points --------------------------------------------- */

/*
 * RX / TX interrupt service routine bodies. Called from the interrupt vectors.
 * These are ordinary functions, NOT interrupt vector declarations.
 *
 * The RX vectors live in nora_uart_dspic33ck_isr.c and are owned by this HAL by
 * default; the TX vectors in the same file are opt-in
 * (NORA_UART_HAL_OWNS_TX_VECTORS=1) because only async TX needs them.
 */
void nora_uart_rx_irq_handler(
    nora_uart_instance_t inst);

/*
 * Refills the TX FIFO from the active transfer and, on the last byte, disables
 * the TX interrupt and reports NORA_UART_EVENT_SEND_COMPLETE.
 */
void nora_uart_tx_irq_handler(
    nora_uart_instance_t inst);

/* HAL-internal RX-ring and asynchronous-transfer glue is intentionally not part of
 * this public API. */

#ifdef __cplusplus
}
#endif

#endif /* NORA_UART_H */
