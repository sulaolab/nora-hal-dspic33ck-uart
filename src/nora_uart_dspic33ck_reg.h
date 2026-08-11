#ifndef NORA_UART_DSPIC33CK_REG_H
#define NORA_UART_DSPIC33CK_REG_H

#include <stdint.h>
#include <stdbool.h>

/*
 * No interrupt flag/enable descriptor lives here any more.
 *
 * There used to be a nora_uart_dspic33ck_irq_t of (IFSx pointer, IECx pointer, mask) per
 * instance and direction, with reg_irq_enable/disable/clear/raise operating on it.
 * Every one of those was a read-modify-write on a register the UART does not own:
 * IFS0 on this part also carries T1IF, DMA0/1IF, CCP1/CCT1IF and SPI1RX/TXIF, so a
 * flag the hardware set between the read and the write was erased by the write --
 * and the RX ISR did exactly that on every received byte.
 *
 * The bit operations are now instance switches over the DFP's _UxRXIF/_UxRXIE
 * aliases in nora_uart_dspic33ck_device.c, which the compiler emits as a single atomic
 * bclr/bset. Anything that needs to touch a UART interrupt bit calls those; nothing
 * should reintroduce a pointer+mask path to a shared register.
 *
 * The helpers below stay because MODE/MODEH/STA/STAH/BRG belong to one UART: a
 * read-modify-write there can only race that UART's own hardware-set status bits,
 * which is a different and much smaller problem (see the header's OERR note).
 */

typedef struct {
    volatile uint16_t *MODE;
    volatile uint16_t *MODEH;
    volatile uint16_t *STA;
    volatile uint16_t *STAH;
    volatile uint16_t *BRG;
    volatile uint16_t *BRGH;
    volatile uint16_t *TXREG;
    volatile uint16_t *RXREG;
} nora_uart_dspic33ck_regs_t;

#define NORA_UART_DSPIC33CK_MODE_URXEN       (1u << 4)
#define NORA_UART_DSPIC33CK_MODE_UTXEN       (1u << 5)
#define NORA_UART_DSPIC33CK_MODE_BRGH        (1u << 7)
#define NORA_UART_DSPIC33CK_MODE_UARTEN      (1u << 15)

#define NORA_UART_DSPIC33CK_MODEH_STSEL_MASK (3u << 4)
#define NORA_UART_DSPIC33CK_MODEH_BCLKSEL_MASK (3u << 9)

#define NORA_UART_DSPIC33CK_STA_OERR         (1u << 1)
#define NORA_UART_DSPIC33CK_STA_FERR         (1u << 3)
#define NORA_UART_DSPIC33CK_STA_PERR         (1u << 6)
#define NORA_UART_DSPIC33CK_STA_TRMT         (1u << 7)

#define NORA_UART_DSPIC33CK_STAH_URXBE       (1u << 1)
#define NORA_UART_DSPIC33CK_STAH_UTXBF       (1u << 4)
#define NORA_UART_DSPIC33CK_STAH_UTXBE       (1u << 5)
#define NORA_UART_DSPIC33CK_STAH_URXISEL_MASK (7u << 8)  /* RX interrupt watermark select */

static inline void nora_uart_dspic33ck_reg_set(volatile uint16_t *reg, uint16_t mask)
{
    *reg = (uint16_t)(*reg | mask);
}

static inline void nora_uart_dspic33ck_reg_clear(volatile uint16_t *reg, uint16_t mask)
{
    *reg = (uint16_t)(*reg & (uint16_t)~mask);
}

static inline bool nora_uart_dspic33ck_reg_is_set(volatile uint16_t *reg, uint16_t mask)
{
    return ((*reg & mask) != 0u);
}

static inline void nora_uart_dspic33ck_reg_write_field(
    volatile uint16_t *reg,
    uint16_t mask,
    uint16_t value)
{
    *reg = (uint16_t)((*reg & (uint16_t)~mask) | (value & mask));
}

#endif /* NORA_UART_DSPIC33CK_REG_H */
