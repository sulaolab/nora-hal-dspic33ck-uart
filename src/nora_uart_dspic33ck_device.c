#include <xc.h>

#include "nora_uart_dspic33ck_device.h"

/*
 * Per-instance UART register pointers.
 *
 * The RX/TX interrupt flag and enable bits are deliberately NOT here. They used to
 * be, as (IFSx pointer, IECx pointer, mask) triples chosen by a per-bank #ifdef
 * ladder -- because the banks differ per instance on this part (U1 in IFS0/IEC0,
 * U2 in IFS1/IEC1, U3 in IFS3/IEC3) where dsPIC33AK keeps them all in IFS2/IFS3.
 * They are now instance switches over the DFP's own _UxRXIF / _UxRXIE bit aliases
 * (see nora_uart_dspic33ck_device_rx_irq_* below), which is both atomic and free of the
 * bank question entirely: the DFP defines _U1RXIF wherever the bit actually lives,
 * so there is no bank for this file to get right or to re-derive when porting.
 *
 * The #error guards that used to protect the ladder moved with the bits: a part
 * with a UART but no _UxRXIF/_UxRXIE alias still fails to compile rather than
 * shipping an RX ISR that cannot clear its own flag.
 */
static const nora_uart_dspic33ck_device_t uart_devices[NORA_UART_INST_COUNT] = {
#if defined(U1MODE)
    [NORA_UART_INST_1] = {
        .present = true,
        .regs = {
            .MODE = &U1MODE,
            .MODEH = &U1MODEH,
            .STA = &U1STA,
            .STAH = &U1STAH,
            .BRG = &U1BRG,
            .BRGH = &U1BRGH,
            .TXREG = &U1TXREG,
            .RXREG = &U1RXREG,
        },
    },
#else
    [NORA_UART_INST_1] = { .present = false },
#endif

#if defined(U2MODE)
    [NORA_UART_INST_2] = {
        .present = true,
        .regs = {
            .MODE = &U2MODE,
            .MODEH = &U2MODEH,
            .STA = &U2STA,
            .STAH = &U2STAH,
            .BRG = &U2BRG,
            .BRGH = &U2BRGH,
            .TXREG = &U2TXREG,
            .RXREG = &U2RXREG,
        },
    },
#else
    [NORA_UART_INST_2] = { .present = false },
#endif

#if defined(U3MODE)
    [NORA_UART_INST_3] = {
        .present = true,
        .regs = {
            .MODE = &U3MODE,
            .MODEH = &U3MODEH,
            .STA = &U3STA,
            .STAH = &U3STAH,
            .BRG = &U3BRG,
            .BRGH = &U3BRGH,
            .TXREG = &U3TXREG,
            .RXREG = &U3RXREG,
        },
    },
#else
    [NORA_UART_INST_3] = { .present = false },
#endif
};

const nora_uart_dspic33ck_device_t *nora_uart_dspic33ck_get_device(
    nora_uart_instance_t inst)
{
    if ((unsigned)inst >= (unsigned)NORA_UART_INST_COUNT) {
        return 0;
    }

    if (!uart_devices[inst].present) {
        return 0;
    }

    return &uart_devices[inst];
}

bool nora_uart_dspic33ck_instance_is_present(nora_uart_instance_t inst)
{
    return (nora_uart_dspic33ck_get_device(inst) != 0);
}

bool nora_uart_dspic33ck_device_set_rx_irq_priority(
    nora_uart_instance_t inst,
    uint8_t priority)
{
    switch (inst) {
#if defined(_U1RXIP)
    case NORA_UART_INST_1: _U1RXIP = priority; return true;
#endif
#if defined(_U2RXIP)
    case NORA_UART_INST_2: _U2RXIP = priority; return true;
#endif
#if defined(_U3RXIP)
    case NORA_UART_INST_3: _U3RXIP = priority; return true;
#endif
    default: break;
    }

    return false;
}

bool nora_uart_dspic33ck_device_set_tx_irq_priority(
    nora_uart_instance_t inst,
    uint8_t priority)
{
    switch (inst) {
#if defined(_U1TXIP)
    case NORA_UART_INST_1: _U1TXIP = priority; return true;
#endif
#if defined(_U2TXIP)
    case NORA_UART_INST_2: _U2TXIP = priority; return true;
#endif
#if defined(_U3TXIP)
    case NORA_UART_INST_3: _U3TXIP = priority; return true;
#endif
    default: break;
    }

    return false;
}

/*
 * A present UART whose RX interrupt bits the DFP does not name would compile to a
 * switch with no arm for it, i.e. an RX ISR unable to clear its own flag and a ring
 * that never fills. That is what the old bank ladder's #error protected against, so
 * it is kept here rather than dropped with the ladder.
 */
#if defined(U1MODE) && !(defined(_U1RXIF) && defined(_U1RXIE))
#error "UART1 is present but the DFP names no _U1RXIF/_U1RXIE bit alias"
#endif
#if defined(U2MODE) && !(defined(_U2RXIF) && defined(_U2RXIE))
#error "UART2 is present but the DFP names no _U2RXIF/_U2RXIE bit alias"
#endif
#if defined(U3MODE) && !(defined(_U3RXIF) && defined(_U3RXIE))
#error "UART3 is present but the DFP names no _U3RXIF/_U3RXIE bit alias"
#endif

bool nora_uart_dspic33ck_device_rx_irq_flag_clear(nora_uart_instance_t inst)
{
    switch (inst) {
#if defined(_U1RXIF)
    case NORA_UART_INST_1: _U1RXIF = 0; return true;
#endif
#if defined(_U2RXIF)
    case NORA_UART_INST_2: _U2RXIF = 0; return true;
#endif
#if defined(_U3RXIF)
    case NORA_UART_INST_3: _U3RXIF = 0; return true;
#endif
    default: break;
    }

    return false;
}

bool nora_uart_dspic33ck_device_rx_irq_enable(
    nora_uart_instance_t inst,
    bool enable)
{
    /* The ternary is a runtime value even though it can only be 0 or 1, and a runtime value
     * is only atomic here because dsPIC33C has BFINS -- the same line is a whole-word
     * read-modify-write on dsPIC33A (see the interrupt-atomicity analysis in the
     * development tree, section 8). Storing
     * a literal in each arm holds on both. */
    if (enable) {
        switch (inst) {
#if defined(_U1RXIE)
        case NORA_UART_INST_1: _U1RXIE = 1; return true;
#endif
#if defined(_U2RXIE)
        case NORA_UART_INST_2: _U2RXIE = 1; return true;
#endif
#if defined(_U3RXIE)
        case NORA_UART_INST_3: _U3RXIE = 1; return true;
#endif
        default: break;
        }
        return false;
    }

    switch (inst) {
#if defined(_U1RXIE)
    case NORA_UART_INST_1: _U1RXIE = 0; return true;
#endif
#if defined(_U2RXIE)
    case NORA_UART_INST_2: _U2RXIE = 0; return true;
#endif
#if defined(_U3RXIE)
    case NORA_UART_INST_3: _U3RXIE = 0; return true;
#endif
    default: break;
    }

    return false;
}

uint8_t nora_uart_dspic33ck_device_rx_irq_get_enable(nora_uart_instance_t inst)
{
    switch (inst) {
#if defined(_U1RXIE)
    case NORA_UART_INST_1: return (_U1RXIE != 0u) ? 1u : 0u;
#endif
#if defined(_U2RXIE)
    case NORA_UART_INST_2: return (_U2RXIE != 0u) ? 1u : 0u;
#endif
#if defined(_U3RXIE)
    case NORA_UART_INST_3: return (_U3RXIE != 0u) ? 1u : 0u;
#endif
    default: break;
    }

    return 0u;
}

/*
 * Same guard as the RX one above, for the TX side: a present UART whose TX
 * interrupt bits the DFP does not name would compile to a switch with no arm for
 * it, i.e. an async TX transfer that arms nothing and never reports
 * SEND_COMPLETE. The async engine reports _ERR_UNSUPPORTED when these helpers
 * return false, but a build-time error is the better place to find out.
 */
#if defined(U1MODE) && !(defined(_U1TXIF) && defined(_U1TXIE))
#error "UART1 is present but the DFP names no _U1TXIF/_U1TXIE bit alias"
#endif
#if defined(U2MODE) && !(defined(_U2TXIF) && defined(_U2TXIE))
#error "UART2 is present but the DFP names no _U2TXIF/_U2TXIE bit alias"
#endif
#if defined(U3MODE) && !(defined(_U3TXIF) && defined(_U3TXIE))
#error "UART3 is present but the DFP names no _U3TXIF/_U3TXIE bit alias"
#endif

bool nora_uart_dspic33ck_device_tx_irq_flag_clear(nora_uart_instance_t inst)
{
    switch (inst) {
#if defined(_U1TXIF)
    case NORA_UART_INST_1: _U1TXIF = 0; return true;
#endif
#if defined(_U2TXIF)
    case NORA_UART_INST_2: _U2TXIF = 0; return true;
#endif
#if defined(_U3TXIF)
    case NORA_UART_INST_3: _U3TXIF = 0; return true;
#endif
    default: break;
    }

    return false;
}

bool nora_uart_dspic33ck_device_tx_irq_raise_flag(nora_uart_instance_t inst)
{
    switch (inst) {
#if defined(_U1TXIF)
    case NORA_UART_INST_1: _U1TXIF = 1; return true;
#endif
#if defined(_U2TXIF)
    case NORA_UART_INST_2: _U2TXIF = 1; return true;
#endif
#if defined(_U3TXIF)
    case NORA_UART_INST_3: _U3TXIF = 1; return true;
#endif
    default: break;
    }

    return false;
}

bool nora_uart_dspic33ck_device_tx_irq_enable(
    nora_uart_instance_t inst,
    bool enable)
{
    /* Literal per arm, not a ternary: see the note on
     * nora_uart_dspic33ck_device_rx_irq_enable(). */
    if (enable) {
        switch (inst) {
#if defined(_U1TXIE)
        case NORA_UART_INST_1: _U1TXIE = 1; return true;
#endif
#if defined(_U2TXIE)
        case NORA_UART_INST_2: _U2TXIE = 1; return true;
#endif
#if defined(_U3TXIE)
        case NORA_UART_INST_3: _U3TXIE = 1; return true;
#endif
        default: break;
        }
        return false;
    }

    switch (inst) {
#if defined(_U1TXIE)
    case NORA_UART_INST_1: _U1TXIE = 0; return true;
#endif
#if defined(_U2TXIE)
    case NORA_UART_INST_2: _U2TXIE = 0; return true;
#endif
#if defined(_U3TXIE)
    case NORA_UART_INST_3: _U3TXIE = 0; return true;
#endif
    default: break;
    }

    return false;
}
