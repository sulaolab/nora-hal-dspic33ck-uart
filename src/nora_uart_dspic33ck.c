#include <xc.h>
#include <stdint.h>
#include <string.h>

#include "nora_uart.h"
#include "nora_uart_dspic33ck_device.h"
#include "nora_uart_dspic33ck_reg.h"
#include "nora_uart_dspic33ck_rx_isr_ring.h"

#define NORA_UART_BRG_MAX 0x000FFFFFUL

static bool initialized[NORA_UART_INST_COUNT];
static uint32_t timeout_ms[NORA_UART_INST_COUNT];
static nora_uart_get_ms_fn get_ms[NORA_UART_INST_COUNT];
static uint32_t baudrate_applied[NORA_UART_INST_COUNT];
/* The clock the divisor was computed against. Kept so nora_uart_set_baudrate()
 * leaves the instance context describing the rate that is actually on the wire,
 * as the dsPIC33AK HAL does. */
static uint32_t uart_clk_hz_applied[NORA_UART_INST_COUNT];
/* How many latched RX overruns the reader has had to end -- see
 * nora_uart_rx_ready(). Each one was a reception that used to stop for good, so
 * a non-zero value is a defect SURVIVED, and 0 is what a healthy board reports. */
static volatile uint32_t rx_overrun_recovered[NORA_UART_INST_COUNT];
/* Bytes this backend has handed to a POLLED reader. The ISR ring keeps its own
 * count, but a polled console never configures the ring, so without this the
 * rx_byte_count field is zero on a board where reception demonstrably works --
 * which makes it useless as evidence exactly when evidence is wanted. */
static volatile uint32_t rx_byte_polled[NORA_UART_INST_COUNT];
/* Framing/parity errors the POLLED reader has discarded. These used to be
 * ISR-ring-only fields, which meant they read zero on a polled console no matter
 * how many errored bytes arrived -- see nora_uart_read_byte() for what that cost. */
static volatile uint32_t rx_framing_polled[NORA_UART_INST_COUNT];
static volatile uint32_t rx_parity_polled[NORA_UART_INST_COUNT];

/* Asynchronous transfer model state. Inert until nora_uart_tx_start() /
 * _rx_start() / _rx_start_clean() is called, so a build that never uses it pays
 * only for these words (the code itself is dropped by isolate-each-function +
 * remove-unused-sections). */
static uint8_t tx_irq_priority_applied[NORA_UART_INST_COUNT];
static nora_uart_event_callback_t event_callback[NORA_UART_INST_COUNT];
static void *event_callback_user_data[NORA_UART_INST_COUNT];

static const uint8_t   *tx_async_buf[NORA_UART_INST_COUNT];
static size_t           tx_async_len[NORA_UART_INST_COUNT];
static volatile size_t  tx_async_count[NORA_UART_INST_COUNT];
static volatile bool    tx_async_busy[NORA_UART_INST_COUNT];

static uint8_t         *rx_async_buf[NORA_UART_INST_COUNT];
static size_t           rx_async_len[NORA_UART_INST_COUNT];
static volatile size_t  rx_async_count[NORA_UART_INST_COUNT];
static volatile bool    rx_async_busy[NORA_UART_INST_COUNT];

static bool uart_inst_is_valid(nora_uart_instance_t inst);
static nora_uart_status_t uart_get_regs(
    nora_uart_instance_t inst,
    const nora_uart_dspic33ck_regs_t **regs);
static nora_uart_status_t uart_require_initialized(
    nora_uart_instance_t inst,
    const nora_uart_dspic33ck_regs_t **regs);
static nora_uart_status_t uart_calc_brg(
    const nora_uart_config_t *config,
    uint32_t *brg);
static nora_uart_status_t uart_calc_brg_raw(
    uint32_t uart_clk_hz,
    uint32_t baudrate,
    bool high_speed,
    uint32_t *brg);
static bool uart_timeout_enabled(nora_uart_instance_t inst);
static uint32_t uart_timeout_start_ms(nora_uart_instance_t inst);
static bool uart_timeout_expired(
    nora_uart_instance_t inst,
    uint32_t start_ms);
static void uart_interrupts_disable(nora_uart_instance_t inst);
static void uart_async_reset(nora_uart_instance_t inst);
static void uart_async_rx_arm(
    nora_uart_instance_t inst,
    uint8_t *data,
    size_t length);
static void uart_notify(nora_uart_instance_t inst, uint32_t events);

/* Some existing CK configurations use only the polling backend and deliberately
 * omit the RX ISR-ring object. These weak fallbacks keep the public snapshot
 * available there; an included ISR-ring backend supplies strong definitions. */
bool __attribute__((weak)) nora_uart_dspic33ck_rx_isr_is_configured(
    nora_uart_instance_t inst)
{
    (void)inst;
    return false;
}

void __attribute__((weak)) nora_uart_dspic33ck_rx_isr_status_get(
    nora_uart_instance_t inst,
    nora_uart_dspic33ck_rx_isr_status_t *status)
{
    (void)inst;

    if (status != 0) {
        memset(status, 0, sizeof(*status));
    }
}

void __attribute__((weak)) nora_uart_dspic33ck_rx_isr_status_clear(
    nora_uart_instance_t inst)
{
    (void)inst;
}

/* Same reason as above, for the paths nora_uart_init() and the byte-stream API
 * now take when rx_mode == ISR_RING. A build without the ISR-ring object cannot
 * offer the ring, and says so (_ERR_UNSUPPORTED) instead of quietly running the
 * caller on the FIFO under a config that asked for a ring. */
nora_uart_status_t __attribute__((weak)) nora_uart_dspic33ck_rx_isr_config(
    nora_uart_instance_t inst,
    const nora_uart_dspic33ck_rx_isr_config_t *config)
{
    (void)inst;
    (void)config;
    return NORA_UART_ERR_UNSUPPORTED;
}

nora_uart_status_t __attribute__((weak)) nora_uart_dspic33ck_rx_isr_enable(
    nora_uart_instance_t inst)
{
    (void)inst;
    return NORA_UART_ERR_UNSUPPORTED;
}

nora_uart_status_t __attribute__((weak)) nora_uart_dspic33ck_rx_isr_disable(
    nora_uart_instance_t inst)
{
    (void)inst;
    return NORA_UART_ERR_UNSUPPORTED;
}

bool __attribute__((weak)) nora_uart_dspic33ck_rx_isr_ready(
    nora_uart_instance_t inst)
{
    (void)inst;
    return false;
}

nora_uart_status_t __attribute__((weak)) nora_uart_dspic33ck_rx_isr_read_byte(
    nora_uart_instance_t inst,
    uint8_t *data)
{
    (void)inst;
    (void)data;
    return NORA_UART_ERR_UNSUPPORTED;
}

void __attribute__((weak)) nora_uart_dspic33ck_rx_isr_flush(
    nora_uart_instance_t inst)
{
    (void)inst;
}

nora_uart_status_t nora_uart_init(
    nora_uart_instance_t inst,
    const nora_uart_config_t *config)
{
    const nora_uart_dspic33ck_regs_t *r;
    uint32_t brg;
    nora_uart_status_t st;

    if (!uart_inst_is_valid(inst) || config == 0) {
        return NORA_UART_ERR_INVALID_ARG;
    }

    if (config->uart_clk_hz == 0u || config->baudrate == 0u ||
        config->data_bits != 8u || config->stop_bits != 1u ||
        config->parity != NORA_UART_PARITY_NONE ||
        (unsigned)config->clock_source > (unsigned)NORA_UART_BCLKSEL_REFCLK) {
        return NORA_UART_ERR_INVALID_ARG;
    }

    if (config->rx_mode != NORA_UART_RX_MODE_POLLING &&
        config->rx_mode != NORA_UART_RX_MODE_ISR_RING) {
        return NORA_UART_ERR_INVALID_ARG;
    }

    /*
     * A CPU interrupt priority is a 3-bit field: 1..7 are the usable levels and 0
     * means "masked by CPU priority rules", i.e. this interrupt never runs. So a
     * value above 7 is not a slow interrupt, it is a truncated one -- writing 8
     * lands as 0 and silently disables the line. Refuse it for both directions
     * before anything is programmed.
     *
     * 0 is legal here and means "not using this interrupt": for TX it is what an
     * integration that never calls nora_uart_tx_start() passes, and the ISR-ring
     * block below rejects it for RX because a ring whose interrupt cannot fire is
     * exactly the silent-fallback shape this HAL refuses elsewhere.
     */
    if (config->rx_irq_priority > 7u || config->tx_irq_priority > 7u) {
        return NORA_UART_ERR_INVALID_ARG;
    }

    /* Rejected here rather than after the peripheral is running, so a config that
     * asks for a ring it cannot have leaves the instance untouched. enable_rx is
     * part of it: an RX interrupt on a disabled receiver never fires. */
    if (config->rx_mode == NORA_UART_RX_MODE_ISR_RING) {
        if (config->rx_ring_buffer == 0 || config->rx_ring_buffer_size < 2u ||
            config->rx_irq_priority == 0u || !config->enable_rx) {
            return NORA_UART_ERR_INVALID_ARG;
        }
    }

    st = uart_get_regs(inst, &r);
    if (st != NORA_UART_OK) {
        return st;
    }

    st = uart_calc_brg(config, &brg);
    if (st != NORA_UART_OK) {
        return st;
    }

    uart_interrupts_disable(inst);

    *r->MODE = 0u;
    *r->MODEH = 0u;
    *r->STA = 0u;
    *r->STAH = 0x0022u;
    *r->BRG = (uint16_t)(brg & 0xFFFFu);
    *r->BRGH = (uint16_t)((brg >> 16) & 0x000Fu);

    if (config->high_speed) {
        nora_uart_dspic33ck_reg_set(r->MODE, NORA_UART_DSPIC33CK_MODE_BRGH);
    }

    nora_uart_dspic33ck_reg_write_field(
        r->MODEH,
        NORA_UART_DSPIC33CK_MODEH_BCLKSEL_MASK,
        (uint16_t)((uint16_t)config->clock_source << 9));

    if (config->enable_tx) {
        nora_uart_dspic33ck_reg_set(r->MODE, NORA_UART_DSPIC33CK_MODE_UTXEN);
    }
    if (config->enable_rx) {
        nora_uart_dspic33ck_reg_set(r->MODE, NORA_UART_DSPIC33CK_MODE_URXEN);
    }

    nora_uart_dspic33ck_reg_set(r->MODE, NORA_UART_DSPIC33CK_MODE_UARTEN);

    timeout_ms[inst] = config->timeout_ms;
    get_ms[inst] = config->get_ms;
    baudrate_applied[inst] = config->baudrate;
    uart_clk_hz_applied[inst] = config->uart_clk_hz;
    initialized[inst] = true;

    /* Async transfer state starts clean, and the TX interrupt priority is
     * programmed here while its enable bit stays clear -- nora_uart_tx_start() is
     * what enables it. Also clears any callback a previous incarnation left. */
    uart_async_reset(inst);
    event_callback[inst] = 0;
    event_callback_user_data[inst] = 0;
    tx_irq_priority_applied[inst] = config->tx_irq_priority;
    if (config->tx_irq_priority != 0u &&
        !nora_uart_dspic33ck_device_set_tx_irq_priority(
            inst, config->tx_irq_priority)) {
        /*
         * The caller asked for async TX (a non-zero priority) on an instance whose
         * TX priority symbol this device does not name, so the priority was NOT
         * programmed. Discarding this result is how an async TX would later start
         * with its interrupt at whatever priority the reset left -- report instead,
         * and undo the init so a refused config leaves nothing half-configured.
         * A zero priority is not a request, so it is not an error.
         */
        (void)nora_uart_deinit(inst);
        return NORA_UART_ERR_UNSUPPORTED;
    }

    /* Set up after initialized[] is true because the ring's own config call
     * requires an initialized instance. A failure here undoes the whole init: the
     * caller asked for a ring, and half a UART is worse than none. */
    if (config->rx_mode == NORA_UART_RX_MODE_ISR_RING) {
        nora_uart_dspic33ck_rx_isr_config_t ring_config;

        ring_config.buffer = config->rx_ring_buffer;
        ring_config.buffer_size = config->rx_ring_buffer_size;
        ring_config.irq_priority = config->rx_irq_priority;

        st = nora_uart_dspic33ck_rx_isr_config(inst, &ring_config);
        if (st == NORA_UART_OK) {
            st = nora_uart_dspic33ck_rx_isr_enable(inst);
        }
        if (st != NORA_UART_OK) {
            (void)nora_uart_deinit(inst);
            return st;
        }
    }

    return NORA_UART_OK;
}

nora_uart_status_t nora_uart_deinit(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ck_regs_t *r;
    nora_uart_status_t st;

    st = uart_get_regs(inst, &r);
    if (st != NORA_UART_OK) {
        return st;
    }

    /* Ring first: it owns the RX interrupt enable, and taking the peripheral down
     * under a live RX ISR is the one order that can leave an interrupt pending on a
     * dead UART. */
    if (nora_uart_dspic33ck_rx_isr_is_configured(inst)) {
        (void)nora_uart_dspic33ck_rx_isr_disable(inst);
    }

    uart_interrupts_disable(inst);
    *r->MODE = 0u;
    *r->MODEH = 0u;
    *r->STA = 0u;
    *r->STAH = 0u;
    *r->BRG = 0u;
    *r->BRGH = 0u;

    timeout_ms[inst] = 0u;
    get_ms[inst] = 0;
    baudrate_applied[inst] = 0u;
    uart_clk_hz_applied[inst] = 0u;
    initialized[inst] = false;

    /* Drop async transfer state and the callback: a transfer cannot outlive the
     * UART that was carrying it, and a stale callback would fire from the next
     * incarnation's ISR. */
    uart_async_reset(inst);
    event_callback[inst] = 0;
    event_callback_user_data[inst] = 0;
    tx_irq_priority_applied[inst] = 0u;

    return NORA_UART_OK;
}

bool nora_uart_is_present(nora_uart_instance_t inst)
{
    return nora_uart_dspic33ck_instance_is_present(inst);
}

bool nora_uart_is_initialized(nora_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return false;
    }

    return initialized[inst];
}

bool nora_uart_rx_ready(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ck_regs_t *r;

    if (uart_require_initialized(inst, &r) != NORA_UART_OK) {
        return false;
    }

    /* ISR-ring mode answers from the ring, as the dsPIC33AK HAL does: the ISR has
     * already drained the FIFO, so asking the FIFO here would report "nothing to
     * read" with a ring full of bytes. The ring's own reader also ends a stalled
     * reception, which the FIFO path below cannot see. */
    if (nora_uart_dspic33ck_rx_isr_is_configured(inst)) {
        return nora_uart_dspic33ck_rx_isr_ready(inst);
    }

    /*
     * ENDING A LATCHED OVERRUN IS THIS FUNCTION'S JOB, and that is a defect fix, not
     * a feature.  OERR stops the receiver: no further character enters the FIFO
     * until software clears it.  The only place that cleared it on this path was
     * nora_uart_read_byte(), which a polled reader reaches ONLY WHEN THIS
     * FUNCTION SAYS TRUE -- so once the FIFO drained, this said "nothing to read",
     * read_byte() was never called, OERR was never cleared, and reception was over
     * for the life of the image.  TX is untouched by any of it, so the board goes on
     * printing and looks healthy: MEASURED on EV88G73A, a console that echoed
     * nothing while the audio ISR ran 885 000 blocks with miss = 0, and only a reset
     * ended it.  The reader was the one thing still running, and it was the one thing
     * that could not get out.
     *
     * Cleared only with the FIFO EMPTY, which is what makes this lossless under
     * either family's semantics: the classic dsPIC UART resets the receive buffer
     * when OERR is cleared, so clearing it with characters still queued would throw
     * away a command line instead of receiving it.  With the FIFO empty there is
     * nothing to throw away, and that is exactly the stuck state -- OERR set, FIFO
     * empty, receiver halted -- this exists to leave.
     */
    if (nora_uart_dspic33ck_reg_is_set(r->STAH, NORA_UART_DSPIC33CK_STAH_URXBE)) {
        if (nora_uart_dspic33ck_reg_is_set(r->STA, NORA_UART_DSPIC33CK_STA_OERR)) {
            nora_uart_dspic33ck_reg_clear(r->STA, NORA_UART_DSPIC33CK_STA_OERR);
            rx_overrun_recovered[inst]++;
        }
        return false;
    }

    return true;
}

bool nora_uart_tx_ready(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ck_regs_t *r;

    if (uart_require_initialized(inst, &r) != NORA_UART_OK) {
        return false;
    }

    return !nora_uart_dspic33ck_reg_is_set(r->STAH, NORA_UART_DSPIC33CK_STAH_UTXBF);
}

bool nora_uart_tx_done(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ck_regs_t *r;

    if (uart_require_initialized(inst, &r) != NORA_UART_OK) {
        return false;
    }

    return nora_uart_dspic33ck_reg_is_set(r->STA, NORA_UART_DSPIC33CK_STA_TRMT);
}

nora_uart_status_t nora_uart_write_byte(
    nora_uart_instance_t inst,
    uint8_t data)
{
    const nora_uart_dspic33ck_regs_t *r;
    nora_uart_status_t st;
    uint32_t start_ms;

    st = uart_require_initialized(inst, &r);
    if (st != NORA_UART_OK) {
        return st;
    }

    start_ms = uart_timeout_start_ms(inst);
    while (nora_uart_dspic33ck_reg_is_set(r->STAH, NORA_UART_DSPIC33CK_STAH_UTXBF)) {
        if (uart_timeout_enabled(inst) && uart_timeout_expired(inst, start_ms)) {
            return NORA_UART_ERR_TIMEOUT;
        }
    }

    *r->TXREG = data;
    return NORA_UART_OK;
}

nora_uart_status_t nora_uart_read_byte(
    nora_uart_instance_t inst,
    uint8_t *data)
{
    const nora_uart_dspic33ck_regs_t *r;
    nora_uart_status_t st;

    if (data == 0) {
        return NORA_UART_ERR_INVALID_ARG;
    }

    st = uart_require_initialized(inst, &r);
    if (st != NORA_UART_OK) {
        return st;
    }

    /* ISR-ring mode reads the ring (AK contract). The error flags handled below
     * belong to the FIFO path; in ring mode the ISR is what clears them and counts
     * them into the ring's counters. */
    if (nora_uart_dspic33ck_rx_isr_is_configured(inst)) {
        return nora_uart_dspic33ck_rx_isr_read_byte(inst, data);
    }

    /*
     * EVERY ERROR EXIT HERE MUST ALSO END THE CONDITION IT REPORTS, or the reader is
     * the thing that keeps the receiver stopped. FERR and PERR belong to the byte at
     * the head of the receive FIFO and the ONLY way to clear either one is to READ
     * RXREG -- so returning without that read left the errored byte in place, the flag
     * set, and this function returning the same error for the life of the image.
     *
     * MEASURED on EV88G73A 2026-08-11, deaf console reproduced on demand:
     * STA=000E (OERR|FERR) with STAH=001D (URXBF=1, URXBE=0 -- FIFO holding bytes),
     * rx byte count 0 since boot, and nothing the host typed was ever echoed. The
     * FIFO had filled ~8 s after boot, BEFORE the host sent anything: on the Nano
     * mounted to the motherboard the MCU is powered by the board while the debugger's
     * USB is unplugged, so the un-driven RX pin framed garbage into the FIFO. The
     * owner's sequence settles it -- power ON then plug USB reproduces it 100 % of the
     * time, plug USB then power ON never does.
     *
     * Note also what did NOT rescue it: nora_uart_rx_ready() clears OERR only with the
     * FIFO EMPTY (rightly -- see its comment), and this FIFO was full, so the rescue
     * never ran and its counter stayed 0. Discarding here is what makes progress, and
     * it discards nothing real: a framing or parity error means that byte was never a
     * valid character.
     */
    if (nora_uart_dspic33ck_reg_is_set(r->STA, NORA_UART_DSPIC33CK_STA_OERR)) {
        nora_uart_dspic33ck_reg_clear(r->STA, NORA_UART_DSPIC33CK_STA_OERR);
        /* Counted here too, not just in rx_ready(): this path was clearing a latched
         * overrun silently, so `?du` reported 0 on a board that had had one. */
        rx_overrun_recovered[inst]++;
        return NORA_UART_ERR_OVERRUN;
    }
    if (nora_uart_dspic33ck_reg_is_set(r->STA, NORA_UART_DSPIC33CK_STA_FERR)) {
        (void)*r->RXREG;
        rx_framing_polled[inst]++;
        return NORA_UART_ERR_FRAMING;
    }
    if (nora_uart_dspic33ck_reg_is_set(r->STA, NORA_UART_DSPIC33CK_STA_PERR)) {
        (void)*r->RXREG;
        rx_parity_polled[inst]++;
        return NORA_UART_ERR_PARITY;
    }
    if (nora_uart_dspic33ck_reg_is_set(r->STAH, NORA_UART_DSPIC33CK_STAH_URXBE)) {
        return NORA_UART_ERR_RX_EMPTY;
    }

    *data = (uint8_t)(*r->RXREG & 0x00FFu);
    rx_byte_polled[inst]++;
    return NORA_UART_OK;
}

size_t nora_uart_write(
    nora_uart_instance_t inst,
    const void *data,
    size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    size_t i;

    if (len != 0u && data == 0) {
        return 0u;
    }

    for (i = 0u; i < len; i++) {
        if (nora_uart_write_byte(inst, p[i]) != NORA_UART_OK) {
            break;
        }
    }

    return i;
}

size_t nora_uart_read(
    nora_uart_instance_t inst,
    void *data,
    size_t len)
{
    uint8_t *p = (uint8_t *)data;
    size_t i;

    if (len != 0u && data == 0) {
        return 0u;
    }

    for (i = 0u; i < len; i++) {
        if (nora_uart_read_byte(inst, &p[i]) != NORA_UART_OK) {
            break;
        }
    }

    return i;
}

void nora_uart_rx_flush(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ck_regs_t *r;

    if (uart_require_initialized(inst, &r) != NORA_UART_OK) {
        return;
    }

    /* The ring's own flush is the whole job in ring mode -- it drops the ring AND
     * drains the FIFO AND clears OERR, all with the RX interrupt held off. Draining
     * the FIFO again from here would be a foreground RXREG read racing the ISR. */
    if (nora_uart_dspic33ck_rx_isr_is_configured(inst)) {
        nora_uart_dspic33ck_rx_isr_flush(inst);
        return;
    }

    while (!nora_uart_dspic33ck_reg_is_set(r->STAH, NORA_UART_DSPIC33CK_STAH_URXBE)) {
        (void)*r->RXREG;
    }

    if (nora_uart_dspic33ck_reg_is_set(r->STA, NORA_UART_DSPIC33CK_STA_OERR)) {
        nora_uart_dspic33ck_reg_clear(r->STA, NORA_UART_DSPIC33CK_STA_OERR);
    }
}

nora_uart_status_t nora_uart_rx_status_get(
    nora_uart_instance_t inst,
    nora_uart_rx_status_t *status)
{
    const nora_uart_dspic33ck_regs_t *r;
    nora_uart_status_t st;

    if (status == 0) {
        return NORA_UART_ERR_INVALID_ARG;
    }

    st = uart_require_initialized(inst, &r);
    if (st != NORA_UART_OK) {
        return st;
    }

    memset(status, 0, sizeof(*status));
    status->rx_mode = NORA_UART_RX_MODE_POLLING;
    status->rx_overrun_recovered_count = rx_overrun_recovered[inst];
    /* Polled figures first, so an ISR-ring console overwrites them below with its
     * own. Without these a polling console reports zero errored bytes however
     * many it discarded -- and zero is also what a healthy board reports. The
     * byte count is here for the same reason: it is the number that shows
     * reception has stopped, and the ring's own count replaces it below. */
    status->framing_error_count = rx_framing_polled[inst];
    status->parity_error_count  = rx_parity_polled[inst];
    status->rx_byte_count       = rx_byte_polled[inst];

    if (nora_uart_dspic33ck_rx_isr_is_configured(inst)) {
        nora_uart_dspic33ck_rx_isr_status_t isr_status;

        status->rx_mode = NORA_UART_RX_MODE_ISR_RING;
        nora_uart_dspic33ck_rx_isr_status_get(inst, &isr_status);

        status->rx_isr_count            = isr_status.rx_isr_count;
        status->rx_byte_count           = isr_status.rx_byte_count;
        status->rx_fifo_overflow_count  = isr_status.rx_fifo_overflow_count;
        status->framing_error_count     = isr_status.framing_error_count;
        status->parity_error_count      = isr_status.parity_error_count;
        status->rx_ring_overflow_count  = isr_status.rx_ring_overflow_count;
        status->rx_max_drain_count      = isr_status.rx_max_drain_count;
        status->rx_stall_recovery_count = isr_status.rx_stall_recovery_count;
        status->rx_ie_lost_count        = isr_status.rx_ie_lost_count;
    }

    return NORA_UART_OK;
}

nora_uart_status_t nora_uart_rx_status_clear(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ck_regs_t *r;
    nora_uart_status_t st;

    st = uart_require_initialized(inst, &r);
    if (st != NORA_UART_OK) {
        return st;
    }

    if (nora_uart_dspic33ck_rx_isr_is_configured(inst)) {
        nora_uart_dspic33ck_rx_isr_status_clear(inst);
    }
    rx_overrun_recovered[inst] = 0u;
    rx_framing_polled[inst] = 0u;
    rx_parity_polled[inst] = 0u;
    rx_byte_polled[inst] = 0u;

    return NORA_UART_OK;
}

nora_uart_status_t nora_uart_rx_regs_get(
    nora_uart_instance_t inst,
    nora_uart_rx_regs_t *out)
{
    const nora_uart_dspic33ck_regs_t *r;
    nora_uart_status_t st;

    if (out == 0) {
        return NORA_UART_ERR_INVALID_ARG;
    }

    st = uart_require_initialized(inst, &r);
    if (st != NORA_UART_OK) {
        return st;
    }

    /* Read only -- no clearing, no rescue. A diagnostic that repairs what it is
     * measuring cannot testify about it, and nora_uart_rx_ready() is already the
     * place that repairs OERR. */
    out->mode = *r->MODE;
    out->sta  = *r->STA;
    out->stah = *r->STAH;

    return NORA_UART_OK;
}

nora_uart_status_t nora_uart_tx_enable(
    nora_uart_instance_t inst,
    bool enable)
{
    const nora_uart_dspic33ck_regs_t *r;
    nora_uart_status_t st;

    st = uart_require_initialized(inst, &r);
    if (st != NORA_UART_OK) {
        return st;
    }

    /* Disabling TX under an active async transfer would strand it: no interrupt
     * would drain the rest and no SEND_COMPLETE would ever be reported. */
    if (!enable && tx_async_busy[inst]) {
        return NORA_UART_ERR_BUSY;
    }

    if (enable) {
        nora_uart_dspic33ck_reg_set(r->MODE, NORA_UART_DSPIC33CK_MODE_UTXEN);
    } else {
        nora_uart_dspic33ck_reg_clear(r->MODE, NORA_UART_DSPIC33CK_MODE_UTXEN);
    }

    return NORA_UART_OK;
}

nora_uart_status_t nora_uart_rx_enable(
    nora_uart_instance_t inst,
    bool enable)
{
    const nora_uart_dspic33ck_regs_t *r;
    nora_uart_status_t st;

    st = uart_require_initialized(inst, &r);
    if (st != NORA_UART_OK) {
        return st;
    }

    /* Same reason as nora_uart_tx_enable(): no bytes would arrive to finish the
     * transfer, so it would wait for an RX_COMPLETE that cannot come. */
    if (!enable && rx_async_busy[inst]) {
        return NORA_UART_ERR_BUSY;
    }

    if (enable) {
        nora_uart_dspic33ck_reg_set(r->MODE, NORA_UART_DSPIC33CK_MODE_URXEN);
    } else {
        nora_uart_dspic33ck_reg_clear(r->MODE, NORA_UART_DSPIC33CK_MODE_URXEN);
    }

    return NORA_UART_OK;
}

bool nora_uart_tx_is_enabled(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ck_regs_t *r;

    if (uart_require_initialized(inst, &r) != NORA_UART_OK) {
        return false;
    }

    return nora_uart_dspic33ck_reg_is_set(r->MODE, NORA_UART_DSPIC33CK_MODE_UTXEN);
}

bool nora_uart_rx_is_enabled(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ck_regs_t *r;

    if (uart_require_initialized(inst, &r) != NORA_UART_OK) {
        return false;
    }

    return nora_uart_dspic33ck_reg_is_set(r->MODE, NORA_UART_DSPIC33CK_MODE_URXEN);
}

nora_uart_status_t nora_uart_set_baudrate(
    nora_uart_instance_t inst,
    uint32_t uart_clk_hz,
    uint32_t baudrate)
{
    const nora_uart_dspic33ck_regs_t *r;
    nora_uart_status_t st;
    uint32_t brg;
    bool high_speed;

    st = uart_require_initialized(inst, &r);
    if (st != NORA_UART_OK) {
        return st;
    }

    if (uart_clk_hz == 0u || baudrate == 0u) {
        return NORA_UART_ERR_INVALID_ARG;
    }

    /* Never reconfigure the divisor while the shifter still holds a character or
     * the write buffer is loaded: the byte in flight would finish at a different
     * rate than it started, which the receiver sees as a framing error rather
     * than as a rate change. */
    if (!nora_uart_dspic33ck_reg_is_set(r->STA, NORA_UART_DSPIC33CK_STA_TRMT) ||
        nora_uart_dspic33ck_reg_is_set(r->STAH, NORA_UART_DSPIC33CK_STAH_UTXBF)) {
        return NORA_UART_ERR_BUSY;
    }

    /* An async transfer spans many characters, so an idle shifter says nothing
     * about it -- refuse for the whole transfer, as the dsPIC33AK HAL does. */
    if (tx_async_busy[inst] || rx_async_busy[inst]) {
        return NORA_UART_ERR_BUSY;
    }

    /* Recompute in the mode the hardware is in, not in a mode the caller assumed. */
    high_speed =
        nora_uart_dspic33ck_reg_is_set(r->MODE, NORA_UART_DSPIC33CK_MODE_BRGH);

    st = uart_calc_brg_raw(uart_clk_hz, baudrate, high_speed, &brg);
    if (st != NORA_UART_OK) {
        return st;
    }

    *r->BRG = (uint16_t)(brg & 0xFFFFu);
    *r->BRGH = (uint16_t)((brg >> 16) & 0x000Fu);

    uart_clk_hz_applied[inst] = uart_clk_hz;
    baudrate_applied[inst] = baudrate;

    return NORA_UART_OK;
}

uint32_t nora_uart_get_baudrate(nora_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return 0u;
    }

    return baudrate_applied[inst];
}

/* ========================================================================== */
/* Asynchronous Transfer Model                                                */
/* ========================================================================== */

nora_uart_status_t nora_uart_set_callback(
    nora_uart_instance_t inst,
    nora_uart_event_callback_t callback,
    void *user_data)
{
    if (!uart_inst_is_valid(inst)) {
        return NORA_UART_ERR_INVALID_ARG;
    }
    if (!nora_uart_dspic33ck_instance_is_present(inst)) {
        return NORA_UART_ERR_NOT_PRESENT;
    }

    event_callback[inst] = callback;
    event_callback_user_data[inst] = user_data;

    return NORA_UART_OK;
}

nora_uart_status_t nora_uart_tx_start(
    nora_uart_instance_t inst,
    const uint8_t *data,
    size_t length)
{
    const nora_uart_dspic33ck_regs_t *r;
    nora_uart_status_t st;

    st = uart_require_initialized(inst, &r);
    if (st != NORA_UART_OK) {
        return st;
    }
    if (data == 0 || length == 0u) {
        return NORA_UART_ERR_INVALID_ARG;
    }
    if (tx_async_busy[inst]) {
        return NORA_UART_ERR_BUSY;
    }

    /* Both of these would produce a transfer that never completes, so they are
     * refused up front rather than reported as started. */
    if (!nora_uart_dspic33ck_reg_is_set(r->MODE, NORA_UART_DSPIC33CK_MODE_UTXEN)) {
        return NORA_UART_ERR_UNSUPPORTED;
    }
    if (tx_irq_priority_applied[inst] == 0u) {
        return NORA_UART_ERR_UNSUPPORTED;
    }

    /* Publish the descriptor before arming the interrupt. */
    tx_async_buf[inst]   = data;
    tx_async_len[inst]   = length;
    tx_async_count[inst] = 0u;
    tx_async_busy[inst]  = true;

    (void)nora_uart_dspic33ck_device_tx_irq_flag_clear(inst);
    if (!nora_uart_dspic33ck_device_tx_irq_enable(inst, true)) {
        tx_async_busy[inst] = false;
        return NORA_UART_ERR_UNSUPPORTED;
    }
    /* Raise the flag by hand: the TX interrupt means "the FIFO has room", which is
     * already true here, so nothing would latch it and the engine would wait for a
     * condition that had already passed. */
    (void)nora_uart_dspic33ck_device_tx_irq_raise_flag(inst);

    return NORA_UART_OK;
}

nora_uart_status_t nora_uart_tx_abort(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ck_regs_t *r;
    nora_uart_status_t st;

    st = uart_require_initialized(inst, &r);
    if (st != NORA_UART_OK) {
        return st;
    }

    (void)nora_uart_dspic33ck_device_tx_irq_enable(inst, false);
    (void)nora_uart_dspic33ck_device_tx_irq_flag_clear(inst);
    tx_async_busy[inst] = false;

    return NORA_UART_OK;
}

size_t nora_uart_tx_count_get(nora_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return 0u;
    }

    return tx_async_count[inst];
}

bool nora_uart_tx_is_busy(nora_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return false;
    }

    return tx_async_busy[inst];
}

nora_uart_status_t nora_uart_rx_start(
    nora_uart_instance_t inst,
    uint8_t *data,
    size_t length)
{
    const nora_uart_dspic33ck_regs_t *r;
    nora_uart_status_t st;

    st = uart_require_initialized(inst, &r);
    if (st != NORA_UART_OK) {
        return st;
    }
    if (data == 0 || length == 0u) {
        return NORA_UART_ERR_INVALID_ARG;
    }
    /* Async RX is fed from the RX ISR, which only runs in ISR-ring mode. */
    if (!nora_uart_dspic33ck_rx_isr_is_configured(inst)) {
        return NORA_UART_ERR_UNSUPPORTED;
    }
    if (!nora_uart_dspic33ck_reg_is_set(r->MODE, NORA_UART_DSPIC33CK_MODE_URXEN)) {
        return NORA_UART_ERR_UNSUPPORTED;
    }
    if (rx_async_busy[inst]) {
        return NORA_UART_ERR_BUSY;
    }

    /* The RX interrupt is already enabled by the ring backend;
     * nora_uart_dspic33ck_async_rx_feed() picks bytes up as soon as busy is true. */
    uart_async_rx_arm(inst, data, length);

    return NORA_UART_OK;
}

nora_uart_status_t nora_uart_rx_start_clean(
    nora_uart_instance_t inst,
    uint8_t *data,
    size_t length)
{
    const nora_uart_dspic33ck_regs_t *r;
    nora_uart_status_t st;
    uint8_t rx_irq_enabled;

    st = uart_require_initialized(inst, &r);
    if (st != NORA_UART_OK) {
        return st;
    }
    if (data == 0 || length == 0u) {
        return NORA_UART_ERR_INVALID_ARG;
    }
    if (!nora_uart_dspic33ck_rx_isr_is_configured(inst)) {
        return NORA_UART_ERR_UNSUPPORTED;
    }
    if (!nora_uart_dspic33ck_reg_is_set(r->MODE, NORA_UART_DSPIC33CK_MODE_URXEN)) {
        return NORA_UART_ERR_UNSUPPORTED;
    }
    if (rx_async_busy[inst]) {
        return NORA_UART_ERR_BUSY;
    }

    rx_irq_enabled = nora_uart_dspic33ck_device_rx_irq_get_enable(inst);
    if (rx_irq_enabled == 0u) {
        return NORA_UART_ERR_UNSUPPORTED;
    }
    if (!nora_uart_dspic33ck_device_rx_irq_enable(inst, false)) {
        return NORA_UART_ERR_UNSUPPORTED;
    }

    /* Drop what predates this call, then arm while the RX ISR cannot move a
     * just-arrived byte into the regular ring. Doing the flush and the arm under
     * one disable is the whole point of this entry point existing. */
    nora_uart_dspic33ck_rx_isr_flush(inst);
    uart_async_rx_arm(inst, data, length);

    (void)nora_uart_dspic33ck_device_rx_irq_enable(inst, true);

    return NORA_UART_OK;
}

nora_uart_status_t nora_uart_rx_abort(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ck_regs_t *r;
    nora_uart_status_t st;

    st = uart_require_initialized(inst, &r);
    if (st != NORA_UART_OK) {
        return st;
    }

    /* Single volatile flag write; the RX feed hook re-checks it before each store. */
    rx_async_busy[inst] = false;

    return NORA_UART_OK;
}

size_t nora_uart_rx_count_get(nora_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return 0u;
    }

    return rx_async_count[inst];
}

bool nora_uart_rx_is_busy(nora_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return false;
    }

    return rx_async_busy[inst];
}

void nora_uart_tx_irq_handler(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ck_regs_t *r;

    if (uart_get_regs(inst, &r) != NORA_UART_OK) {
        return;
    }

    (void)nora_uart_dspic33ck_device_tx_irq_flag_clear(inst);

    /* Spurious or aborted: nothing to send, so make sure the interrupt is off
     * instead of re-entering forever. */
    if (!tx_async_busy[inst]) {
        (void)nora_uart_dspic33ck_device_tx_irq_enable(inst, false);
        return;
    }

    while ((tx_async_count[inst] < tx_async_len[inst]) &&
           !nora_uart_dspic33ck_reg_is_set(r->STAH, NORA_UART_DSPIC33CK_STAH_UTXBF)) {
        *r->TXREG = tx_async_buf[inst][tx_async_count[inst]];
        tx_async_count[inst]++;
    }

    /* All bytes submitted to the FIFO: stop the interrupt and report complete.
     * SEND_COMPLETE is the "driver accepted and submitted all data" sense;
     * physical shift-register-empty stays observable via nora_uart_tx_done(). */
    if (tx_async_count[inst] >= tx_async_len[inst]) {
        (void)nora_uart_dspic33ck_device_tx_irq_enable(inst, false);
        tx_async_busy[inst] = false;
        uart_notify(inst, NORA_UART_EVENT_SEND_COMPLETE);
    }
}

/* ----- Internal hooks called from the RX ISR ------------------------------ */
/* Strong definitions overriding the weak no-ops in
 * nora_uart_dspic33ck_rx_isr_ring.c: this file IS the async layer that header
 * said could be added later. */

bool nora_uart_dspic33ck_async_rx_feed(
    nora_uart_instance_t inst,
    uint8_t byte)
{
    if (!uart_inst_is_valid(inst) || !rx_async_busy[inst]) {
        return false;
    }

    rx_async_buf[inst][rx_async_count[inst]] = byte;
    rx_async_count[inst]++;

    if (rx_async_count[inst] >= rx_async_len[inst]) {
        rx_async_busy[inst] = false;
        uart_notify(inst, NORA_UART_EVENT_RX_COMPLETE);
    }

    return true;
}

void nora_uart_dspic33ck_async_notify_events(
    nora_uart_instance_t inst,
    uint32_t events)
{
    if (!uart_inst_is_valid(inst) || events == 0u) {
        return;
    }

    uart_notify(inst, events);
}

static bool uart_inst_is_valid(nora_uart_instance_t inst)
{
    return ((unsigned)inst < (unsigned)NORA_UART_INST_COUNT);
}

static nora_uart_status_t uart_get_regs(
    nora_uart_instance_t inst,
    const nora_uart_dspic33ck_regs_t **regs)
{
    const nora_uart_dspic33ck_device_t *dev;

    if (regs == 0) {
        return NORA_UART_ERR_INVALID_ARG;
    }

    if (!uart_inst_is_valid(inst)) {
        return NORA_UART_ERR_INVALID_ARG;
    }

    dev = nora_uart_dspic33ck_get_device(inst);
    if (dev == 0) {
        return NORA_UART_ERR_NOT_PRESENT;
    }

    *regs = &dev->regs;
    return NORA_UART_OK;
}

static nora_uart_status_t uart_require_initialized(
    nora_uart_instance_t inst,
    const nora_uart_dspic33ck_regs_t **regs)
{
    nora_uart_status_t st;

    st = uart_get_regs(inst, regs);
    if (st != NORA_UART_OK) {
        return st;
    }

    if (!initialized[inst]) {
        return NORA_UART_ERR_NOT_INITIALIZED;
    }

    return NORA_UART_OK;
}

static nora_uart_status_t uart_calc_brg(
    const nora_uart_config_t *config,
    uint32_t *brg)
{
    return uart_calc_brg_raw(
        config->uart_clk_hz, config->baudrate, config->high_speed, brg);
}

/*
 * The divisor maths itself, without a config object.
 *
 * nora_uart_set_baudrate() has no config to pass: it must recompute against the
 * sample-rate mode the hardware is ALREADY in, read back from MODE.BRGH. Keeping
 * one implementation means the two paths cannot disagree about rounding.
 */
static nora_uart_status_t uart_calc_brg_raw(
    uint32_t uart_clk_hz,
    uint32_t baudrate,
    bool high_speed,
    uint32_t *brg)
{
    const uint32_t sample_div = high_speed ? 4u : 16u;
    const uint64_t denominator = (uint64_t)sample_div * baudrate;
    uint64_t div;

    if (denominator == 0u || brg == 0) {
        return NORA_UART_ERR_INVALID_ARG;
    }

    div = ((uint64_t)uart_clk_hz + (denominator / 2u)) / denominator;
    if (div == 0u || (div - 1u) > NORA_UART_BRG_MAX) {
        return NORA_UART_ERR_INVALID_ARG;
    }

    *brg = (uint32_t)(div - 1u);
    return NORA_UART_OK;
}

static bool uart_timeout_enabled(nora_uart_instance_t inst)
{
    return uart_inst_is_valid(inst) &&
           get_ms[inst] != 0 &&
           timeout_ms[inst] != 0u;
}

static uint32_t uart_timeout_start_ms(nora_uart_instance_t inst)
{
    if (!uart_timeout_enabled(inst)) {
        return 0u;
    }

    return get_ms[inst]();
}

static bool uart_timeout_expired(
    nora_uart_instance_t inst,
    uint32_t start_ms)
{
    uint32_t now;

    if (!uart_timeout_enabled(inst)) {
        return false;
    }

    now = get_ms[inst]();
    return ((uint32_t)(now - start_ms) >= timeout_ms[inst]);
}

static void uart_interrupts_disable(nora_uart_instance_t inst)
{
    switch (inst) {
#if defined(_U1RXIE)
    case NORA_UART_INST_1:
        _U1RXIE = 0;
        _U1TXIE = 0;
        _U1EIE = 0;
        _U1EVTIE = 0;
        _U1RXIF = 0;
        _U1TXIF = 0;
        _U1EIF = 0;
        _U1EVTIF = 0;
        break;
#endif
#if defined(_U2RXIE)
    case NORA_UART_INST_2:
        _U2RXIE = 0;
        _U2TXIE = 0;
        _U2EIE = 0;
        _U2EVTIE = 0;
        _U2RXIF = 0;
        _U2TXIF = 0;
        _U2EIF = 0;
        _U2EVTIF = 0;
        break;
#endif
#if defined(_U3RXIE)
    case NORA_UART_INST_3:
        _U3RXIE = 0;
        _U3TXIE = 0;
        _U3EIE = 0;
        _U3EVTIE = 0;
        _U3RXIF = 0;
        _U3TXIF = 0;
        _U3EIF = 0;
        _U3EVTIF = 0;
        break;
#endif
    default:
        break;
    }
}

static void uart_async_reset(nora_uart_instance_t inst)
{
    tx_async_buf[inst]   = 0;
    tx_async_len[inst]   = 0u;
    tx_async_count[inst] = 0u;
    tx_async_busy[inst]  = false;

    rx_async_buf[inst]   = 0;
    rx_async_len[inst]   = 0u;
    rx_async_count[inst] = 0u;
    rx_async_busy[inst]  = false;
}

static void uart_async_rx_arm(
    nora_uart_instance_t inst,
    uint8_t *data,
    size_t length)
{
    rx_async_buf[inst]   = data;
    rx_async_len[inst]   = length;
    rx_async_count[inst] = 0u;
    /* Published last: the feed hook keys off this flag, so the buffer and the
     * length must already be visible when it becomes true. */
    rx_async_busy[inst]  = true;
}

static void uart_notify(nora_uart_instance_t inst, uint32_t events)
{
    nora_uart_event_callback_t cb = event_callback[inst];

    if (cb != 0) {
        cb(inst, events, event_callback_user_data[inst]);
    }
}
