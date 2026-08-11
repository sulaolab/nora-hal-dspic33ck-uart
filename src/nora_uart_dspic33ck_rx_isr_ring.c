#include <string.h>
#include <stddef.h>

#include "nora_uart_dspic33ck_rx_isr_ring.h"
#include "nora_uart_dspic33ck_device.h"
#include "nora_uart_dspic33ck_reg.h"

/*
 * UART RX interrupt-driven ring HAL - implementation.
 *
 * The RX FIFO -> software-ring drain logic, the RX error-flag accounting, and
 * the RX ISR ring runtime counters live here. The ring buffer storage is
 * caller-provided; this module only keeps a pointer to it plus the read/write
 * indices. Register/IRQ access goes through the device register-pointer table
 * and the reg.h helpers, so no UxSTAbits / _UxRXIF / _UxRXIE symbols appear here.
 */

/*
 * Single-producer (ISR) / single-consumer (reader) ring, per instance.
 *   - g_rx_write_idx is advanced only by nora_uart_rx_irq_handler().
 *   - g_rx_read_idx is advanced only by nora_uart_dspic33ck_rx_isr_read_byte().
 */
static uint8_t *g_rx_ring[NORA_UART_INST_COUNT];
static uint16_t g_rx_ring_size[NORA_UART_INST_COUNT];
static volatile uint16_t g_rx_read_idx[NORA_UART_INST_COUNT];
static volatile uint16_t g_rx_write_idx[NORA_UART_INST_COUNT];
static volatile nora_uart_dspic33ck_rx_isr_status_t g_rx_status[NORA_UART_INST_COUNT];
static bool g_rx_isr_configured[NORA_UART_INST_COUNT];

static bool uart_inst_is_valid(nora_uart_instance_t inst);
static const nora_uart_dspic33ck_regs_t *uart_regs(nora_uart_instance_t inst);

static bool uart_rx_irq_set_priority(nora_uart_instance_t inst, uint8_t prio);
static bool uart_rx_irq_clear_flag(nora_uart_instance_t inst);
static bool uart_rx_irq_enable(nora_uart_instance_t inst, bool enable);
static uint8_t uart_rx_irq_get_enable(nora_uart_instance_t inst);

static void uart_rx_ring_push(nora_uart_instance_t inst, uint8_t b);
static uint16_t uart_rx_drain_fifo(nora_uart_instance_t inst,
                                   const nora_uart_dspic33ck_regs_t *r,
                                   bool *ring_got_data);

nora_uart_status_t nora_uart_dspic33ck_rx_isr_config(
    nora_uart_instance_t inst,
    const nora_uart_dspic33ck_rx_isr_config_t *config)
{
    const nora_uart_dspic33ck_regs_t *r;

    if (!uart_inst_is_valid(inst)) {
        return NORA_UART_ERR_INVALID_ARG;
    }
    if (config == 0 || config->buffer == 0 || config->buffer_size < 2u) {
        return NORA_UART_ERR_INVALID_ARG;
    }

    r = uart_regs(inst);
    if (r == 0) {
        return NORA_UART_ERR_NOT_PRESENT;
    }
    if (!nora_uart_is_initialized(inst)) {
        return NORA_UART_ERR_NOT_INITIALIZED;
    }

    /* Reject instances with no RX interrupt mapping before changing any state. */
    if (!uart_rx_irq_set_priority(inst, config->irq_priority)) {
        return NORA_UART_ERR_UNSUPPORTED;
    }

    /* Bind the caller's buffer and reset ring + status counters. */
    g_rx_ring[inst]      = config->buffer;
    g_rx_ring_size[inst] = config->buffer_size;
    g_rx_read_idx[inst]  = 0u;
    g_rx_write_idx[inst] = 0u;
    memset(config->buffer, 0x00, config->buffer_size);
    memset((void *)&g_rx_status[inst], 0x00, sizeof(g_rx_status[inst]));

    /* Interrupt when >= 1 char is in the RX FIFO (URXISEL = 0). */
    nora_uart_dspic33ck_reg_write_field(r->STAH, NORA_UART_DSPIC33CK_STAH_URXISEL_MASK, 0u);

    /* Priority already set above; clear any stale flag, leave disabled. */
    (void)uart_rx_irq_clear_flag(inst);

    g_rx_isr_configured[inst] = true;

    return NORA_UART_OK;
}

bool nora_uart_dspic33ck_rx_isr_is_configured(nora_uart_instance_t inst)
{
    return uart_inst_is_valid(inst) && g_rx_isr_configured[inst];
}

nora_uart_status_t nora_uart_dspic33ck_rx_isr_enable(
    nora_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return NORA_UART_ERR_INVALID_ARG;
    }
    if (!g_rx_isr_configured[inst]) {
        /* No "not configured" status code exists; the ring is unusable until
         * _rx_isr_config() succeeds, so report not-initialized. */
        return NORA_UART_ERR_NOT_INITIALIZED;
    }
    if (!nora_uart_is_initialized(inst)) {
        return NORA_UART_ERR_NOT_INITIALIZED;
    }

    (void)uart_rx_irq_clear_flag(inst);
    if (!uart_rx_irq_enable(inst, true)) {
        return NORA_UART_ERR_UNSUPPORTED;
    }

    return NORA_UART_OK;
}

nora_uart_status_t nora_uart_dspic33ck_rx_isr_disable(
    nora_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst)) {
        return NORA_UART_ERR_INVALID_ARG;
    }

    /* Disable is the safe direction: allowed even when not configured. */
    if (!uart_rx_irq_enable(inst, false)) {
        return NORA_UART_ERR_UNSUPPORTED;
    }
    (void)uart_rx_irq_clear_flag(inst);

    return NORA_UART_OK;
}

bool nora_uart_dspic33ck_rx_isr_service(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ck_regs_t *r;
    uint8_t ie;
    bool stalled;

    if (!uart_inst_is_valid(inst) || !g_rx_isr_configured[inst]) {
        return false;
    }
    r = uart_regs(inst);
    if (r == 0) {
        return false;
    }

    /*
     * The cheap half, and the reason this can sit in the reader's path: two reads
     * and no writes. OERR set means reception has already stopped. A non-empty FIFO
     * with an empty ring means the bytes did not reach the ring, which in normal
     * operation is a window of a few instructions between the byte landing and the
     * ISR running -- and in a stall never ends.
     *
     * Treating that brief normal window as a stall is harmless and deliberately not
     * filtered out: whoever gets there first drains the SAME FIFO into the SAME ring
     * in the same order, under a critical section that keeps them from interleaving.
     * A test for "and it has not ended for a while" would need a clock in the HAL to
     * buy nothing.
     */
    stalled = nora_uart_dspic33ck_reg_is_set(r->STA, NORA_UART_DSPIC33CK_STA_OERR) ||
              (!nora_uart_dspic33ck_reg_is_set(r->STAH, NORA_UART_DSPIC33CK_STAH_URXBE) &&
               (g_rx_read_idx[inst] == g_rx_write_idx[inst]));
    if (!stalled) {
        return false;
    }

    ie = uart_rx_irq_get_enable(inst);
    (void)uart_rx_irq_enable(inst, false);   /* the ISR's own critical section */

    uart_rx_drain_fifo(inst, r, 0);
    if (nora_uart_dspic33ck_reg_is_set(r->STA, NORA_UART_DSPIC33CK_STA_OERR)) {
        g_rx_status[inst].rx_fifo_overflow_count++;
        nora_uart_dspic33ck_reg_clear(r->STA, NORA_UART_DSPIC33CK_STA_OERR);
    }
    g_rx_status[inst].rx_stall_recovery_count++;
    if (ie == 0u) {
        /* Nothing in this HAL leaves a configured instance with RX disabled, so
         * this is the write-back race named in the header, caught rather than
         * inferred. Counted separately BECAUSE it is a different defect. */
        g_rx_status[inst].rx_ie_lost_count++;
    }

    /* Re-enabled unconditionally, not restored: if the enable bit was the thing
     * that went missing, putting the old value back is putting the stall back. */
    (void)uart_rx_irq_clear_flag(inst);
    (void)uart_rx_irq_enable(inst, true);

    return true;
}

bool nora_uart_dspic33ck_rx_isr_ready(nora_uart_instance_t inst)
{
    if (!uart_inst_is_valid(inst) || !g_rx_isr_configured[inst]) {
        return false;
    }

    if (g_rx_read_idx[inst] == g_rx_write_idx[inst]) {
        (void)nora_uart_dspic33ck_rx_isr_service(inst);
    }

    return (g_rx_read_idx[inst] != g_rx_write_idx[inst]);
}

nora_uart_status_t nora_uart_dspic33ck_rx_isr_read_byte(
    nora_uart_instance_t inst,
    uint8_t *data)
{
    uint16_t read_idx;
    uint16_t next;

    if (data == 0) {
        return NORA_UART_ERR_INVALID_ARG;
    }
    if (!uart_inst_is_valid(inst) || !g_rx_isr_configured[inst]) {
        return NORA_UART_ERR_RX_EMPTY;
    }
    if (g_rx_read_idx[inst] == g_rx_write_idx[inst]) {
        /* Same recovery as _ready(), so a caller that only ever calls this one is
         * not the caller that stays deaf. */
        (void)nora_uart_dspic33ck_rx_isr_service(inst);
    }
    if (g_rx_read_idx[inst] == g_rx_write_idx[inst]) {
        return NORA_UART_ERR_RX_EMPTY;
    }

    read_idx = g_rx_read_idx[inst];
    *data = g_rx_ring[inst][read_idx];

    next = (uint16_t)(read_idx + 1u);
    if (next >= g_rx_ring_size[inst]) {
        next = 0u;
    }
    g_rx_read_idx[inst] = next;

    return NORA_UART_OK;
}

void nora_uart_dspic33ck_rx_isr_flush(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ck_regs_t *r;
    uint8_t ie;

    if (!uart_inst_is_valid(inst)) {
        return;
    }
    r = uart_regs(inst);
    if (r == 0) {
        return;
    }

    ie = uart_rx_irq_get_enable(inst);
    (void)uart_rx_irq_enable(inst, false);   /* brief critical section vs the ISR */

    g_rx_read_idx[inst] = g_rx_write_idx[inst];   /* drop buffered ring contents */

    while (!nora_uart_dspic33ck_reg_is_set(r->STAH, NORA_UART_DSPIC33CK_STAH_URXBE)) {
        (void)*r->RXREG;                      /* drain the hardware RX FIFO too */
    }
    /* Not uart_rx_drain_fifo(): flush DISCARDS, and that helper delivers. */
    if (nora_uart_dspic33ck_reg_is_set(r->STA, NORA_UART_DSPIC33CK_STA_OERR)) {
        nora_uart_dspic33ck_reg_clear(r->STA, NORA_UART_DSPIC33CK_STA_OERR);
    }

    (void)uart_rx_irq_enable(inst, (ie != 0u));
}

void nora_uart_dspic33ck_rx_isr_status_get(
    nora_uart_instance_t inst,
    nora_uart_dspic33ck_rx_isr_status_t *status)
{
    uint8_t ie;

    if (status == 0 || !uart_inst_is_valid(inst)) {
        return;
    }

    ie = uart_rx_irq_get_enable(inst);
    (void)uart_rx_irq_enable(inst, false);   /* atomic snapshot vs the ISR */

    status->rx_isr_count           = g_rx_status[inst].rx_isr_count;
    status->rx_byte_count          = g_rx_status[inst].rx_byte_count;
    status->rx_fifo_overflow_count = g_rx_status[inst].rx_fifo_overflow_count;
    status->framing_error_count    = g_rx_status[inst].framing_error_count;
    status->parity_error_count     = g_rx_status[inst].parity_error_count;
    status->rx_ring_overflow_count = g_rx_status[inst].rx_ring_overflow_count;
    status->rx_max_drain_count     = g_rx_status[inst].rx_max_drain_count;
    status->rx_stall_recovery_count = g_rx_status[inst].rx_stall_recovery_count;
    status->rx_ie_lost_count        = g_rx_status[inst].rx_ie_lost_count;

    (void)uart_rx_irq_enable(inst, (ie != 0u));
}

void nora_uart_dspic33ck_rx_isr_status_clear(nora_uart_instance_t inst)
{
    uint8_t ie;

    if (!uart_inst_is_valid(inst)) {
        return;
    }

    ie = uart_rx_irq_get_enable(inst);
    (void)uart_rx_irq_enable(inst, false);
    memset((void *)&g_rx_status[inst], 0x00, sizeof(g_rx_status[inst]));
    (void)uart_rx_irq_enable(inst, (ie != 0u));
}

void nora_uart_rx_irq_handler(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ck_regs_t *r;
    uint16_t drain_count = 0u;
    uint32_t events = 0u;            /* async event bits to report after drain  */
    uint32_t ring_overflow_before;  /* ring-overflow counter snapshot          */
    bool ring_got_data = false;     /* at least one byte routed to the ring     */

    if (!uart_inst_is_valid(inst)) {
        return;
    }
    r = uart_regs(inst);
    if (r == 0) {
        return;
    }

    (void)uart_rx_irq_clear_flag(inst);   /* clear RX interrupt flag first */
    g_rx_status[inst].rx_isr_count++;
    ring_overflow_before = g_rx_status[inst].rx_ring_overflow_count;

    /*
     * Drain all available bytes from the RX FIFO (reading RXREG advances it). An
     * active async RX transfer takes priority: each byte is offered to it first
     * and only pushed to the software ring when no transfer is consuming bytes.
     * Shared with the reader's stall recovery -- see uart_rx_drain_fifo().
     */
    drain_count = uart_rx_drain_fifo(inst, r, &ring_got_data);

    g_rx_status[inst].rx_byte_count += drain_count;
    if (drain_count > g_rx_status[inst].rx_max_drain_count) {
        g_rx_status[inst].rx_max_drain_count = drain_count;
    }

    /*
     * Count the latched RX error flags and collect the async event bits. OERR is
     * R/W and must be cleared to resume reception; FERR/PERR on dsPIC33CK are
     * read-only status tied to the character drained above and clear as the FIFO
     * is read, so they are only counted here.
     */
    if (nora_uart_dspic33ck_reg_is_set(r->STA, NORA_UART_DSPIC33CK_STA_OERR)) {
        g_rx_status[inst].rx_fifo_overflow_count++;
        nora_uart_dspic33ck_reg_clear(r->STA, NORA_UART_DSPIC33CK_STA_OERR);
        events |= NORA_UART_EVENT_RX_OVERRUN_ERROR;
    }
    if (nora_uart_dspic33ck_reg_is_set(r->STA, NORA_UART_DSPIC33CK_STA_FERR)) {
        g_rx_status[inst].framing_error_count++;
        events |= NORA_UART_EVENT_RX_FRAMING_ERROR;
    }
    if (nora_uart_dspic33ck_reg_is_set(r->STA, NORA_UART_DSPIC33CK_STA_PERR)) {
        g_rx_status[inst].parity_error_count++;
        events |= NORA_UART_EVENT_RX_PARITY_ERROR;
    }

    /* Software ring overflow (a byte was dropped during the drain above). */
    if (g_rx_status[inst].rx_ring_overflow_count != ring_overflow_before) {
        events |= NORA_UART_EVENT_RX_OVERFLOW;
    }

    /* Unsolicited byte-stream data landed in the ring this pass. */
    if (ring_got_data) {
        events |= NORA_UART_EVENT_RX_READY;
    }

    if (events != 0u) {
        nora_uart_dspic33ck_async_notify_events(inst, events);
    }
}

/*
 * Weak no-op defaults for the async glue, so this ring links with or without the
 * async engine: feed returns false so every byte falls through to the ring, and
 * notify discards the events. nora_uart_dspic33ck.c overrides both with strong
 * definitions. (This note used to say no async engine was shipped yet.)
 */
__attribute__((weak)) bool nora_uart_dspic33ck_async_rx_feed(
    nora_uart_instance_t inst,
    uint8_t byte)
{
    (void)inst;
    (void)byte;
    return false;
}

__attribute__((weak)) void nora_uart_dspic33ck_async_notify_events(
    nora_uart_instance_t inst,
    uint32_t events)
{
    (void)inst;
    (void)events;
}

static bool uart_inst_is_valid(nora_uart_instance_t inst)
{
    return ((unsigned)inst < (unsigned)NORA_UART_INST_COUNT);
}

static const nora_uart_dspic33ck_regs_t *uart_regs(nora_uart_instance_t inst)
{
    const nora_uart_dspic33ck_device_t *dev = nora_uart_dspic33ck_get_device(inst);

    if (dev == 0) {
        return 0;
    }
    return &dev->regs;
}

/*
 * Single-producer push (ISR context). If the next write slot would collide with
 * the read index, the byte is dropped and rx_ring_overflow_count is incremented.
 */
/*
 * Drain every byte the RX FIFO holds, offering each to an active async transfer
 * first and pushing the rest to the ring. Returns how many bytes were taken and,
 * when ring_got_data is non-NULL, whether any of them landed in the ring.
 *
 * ONE COPY, called from the ISR and from the reader's stall recovery: they must
 * agree byte for byte about priority and order, and two copies of a FIFO drain
 * would be two chances to disagree.  The caller owns the critical section -- the
 * ISR is one by definition, the reader makes one by disabling the interrupt.
 */
static uint16_t uart_rx_drain_fifo(nora_uart_instance_t inst,
                                   const nora_uart_dspic33ck_regs_t *r,
                                   bool *ring_got_data)
{
    uint16_t drain_count = 0u;

    while (!nora_uart_dspic33ck_reg_is_set(r->STAH, NORA_UART_DSPIC33CK_STAH_URXBE)) {
        uint8_t b = (uint8_t)(*r->RXREG & 0x00FFu);
        if (!nora_uart_dspic33ck_async_rx_feed(inst, b)) {
            uart_rx_ring_push(inst, b);
            if (ring_got_data != 0) {
                *ring_got_data = true;
            }
        }
        drain_count++;
    }

    return drain_count;
}

static void uart_rx_ring_push(nora_uart_instance_t inst, uint8_t b)
{
    uint16_t write_idx = g_rx_write_idx[inst];
    uint16_t next = (uint16_t)(write_idx + 1u);

    if (next >= g_rx_ring_size[inst]) {
        next = 0u;
    }

    if (next == g_rx_read_idx[inst]) {
        g_rx_status[inst].rx_ring_overflow_count++;
        return;
    }

    g_rx_ring[inst][write_idx] = b;
    g_rx_write_idx[inst] = next;
}

static bool uart_rx_irq_set_priority(nora_uart_instance_t inst, uint8_t prio)
{
    return nora_uart_dspic33ck_device_set_rx_irq_priority(inst, prio);
}

/*
 * These three are the whole reason the RX interrupt bits are not in the register
 * descriptor: each is now one atomic bclr/bset, so the ISR clearing its own flag
 * cannot erase T1IF, DMA0/1IF, CCP1IF or SPI1RX/TXIF out of the same IFS0 word.
 * The get_device() call they used to make was only there to reach the descriptor.
 */
static bool uart_rx_irq_clear_flag(nora_uart_instance_t inst)
{
    return nora_uart_dspic33ck_device_rx_irq_flag_clear(inst);
}

static bool uart_rx_irq_enable(nora_uart_instance_t inst, bool enable)
{
    return nora_uart_dspic33ck_device_rx_irq_enable(inst, enable);
}

static uint8_t uart_rx_irq_get_enable(nora_uart_instance_t inst)
{
    return nora_uart_dspic33ck_device_rx_irq_get_enable(inst);
}
