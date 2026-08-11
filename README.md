# nora-hal-dspic33ck-uart

**NORA-HAL** — *Native On-chip Resource Assistant*

Small, readable UART HAL for Microchip dsPIC33CK devices — part of **NORA-HAL**, a HAL
family whose public API is namespaced `nora_*` / `NORA_*`. It provides a byte-stream API
with a polling RX backend, an optional interrupt-driven RX software ring, and an optional
non-blocking (async) TX/RX transfer model with completion events.

## Naming

The public API is `nora_*` / `NORA_*`. This repository has carried that namespace since its
first commit — there is no earlier `dspic33ck_*` public API here and therefore no
compatibility aliases to remove.

The chip name survives in exactly two places, both deliberate:

* **Implementation file names** carry a backend tag: `nora_uart_dspic33ck.c` is the
  dsPIC33CK backend of the processor-neutral `nora_uart.h`.
* **Backend-private identifiers** inside those files — plus the chip-tagged
  `nora_uart_dspic33ck_rx_isr_*()` entry points, which stay available for code that wants
  the RX ring's own diagnostics rather than the portable status snapshot.

The tag is `_dspic33ck` — a different silicon family from dsPIC33AK (dsPIC33**C** vs
dsPIC33**A**), and never abbreviated to `_dspic33c`.

### Relationship to the dsPIC33AK HAL of the same name

[nora-hal-dspic33ak-uart](https://github.com/sulaolab/nora-hal-dspic33ak-uart) is the same
API for the dsPIC33AK family, and the alignment is close enough to be worth stating
precisely:

- **The async transfer model is ported.** `tx_start` / `rx_start` / `rx_start_clean` /
  `tx_abort` / `rx_abort` / the `*_count_get` / `*_is_busy` pair / `set_callback` and the
  completion events all exist here under the same names.
- **The RX backend is selectable from the portable config** (`rx_mode`, `rx_ring_buffer`,
  `rx_ring_buffer_size`, `rx_irq_priority`), with the same field names and semantics, so a
  config literal is portable in both directions and an application written against the AK
  ISR-ring configuration compiles and runs here unchanged.
- **`nora_uart_status_t` keeps the AK enumerator *values*.** `NORA_UART_ERR_BUSY` was
  inserted in the middle rather than appended for exactly that reason. Nothing indexes an
  array by this enum, so the insertion is source-compatible — but a consumer that *prints
  the raw number* sees `_ERR_TIMEOUT` and everything after it shift by one.
- **What is CK-only: the baud clock configuration.** `high_speed` (BRGH) and
  `clock_source` (`BCLKSEL`: `FOSC/2`, `FOSC`, `AFPLLO`, `REFCLK`) have no equivalent on
  the AK UART, which instead takes its clock from CLKGEN8 — a block this family does not
  have at all. So an AK integration's clock setup does *not* port; the rest of the config
  does.

The dsPIC33AK and dsPIC33CK fleets are **not** symmetric, and nothing here should be read
as a claim that they are.

## Status

Validation target:

* Devices: dsPIC33CK64MC105 (EV88G73A Curiosity Nano), dsPIC33CK256MP508 (DM330030)
* Compiler: XC-DSC v3.31.01
* DFP: dsPIC33CK-MC_DFP 1.10.386 / dsPIC33CK-MP_DFP 1.15.423 or compatible

**How to read the evidence below.** These HALs are built for evaluation, FAE demos and
architecture experiments, so exhaustive per-function coverage was never the goal — there is
no unit-test suite, and what exists is integration testing on real hardware. Three tiers,
used across the seven sibling repositories: **integration-verified** (it ran as part of the
working system, and something observable would have broken if it had not),
**hardware-observed, not a matrix** (it worked in the configuration actually run; other
combinations are untried rather than known-good), and **compiled, not executed**.

**Hardware-exercised (EV88G73A, UART1 at 230400 8N1, polling RX, 2026-08-11).** This is
the board's interactive console, so it is the most heavily used HAL in the tree: it
answered commands across a drag-and-drop reprogram, three forced CPU traps, a software
reset, a peripheral restart and a cold power cycle — **six reset paths** — while audio ran
with `miss = 0`. Specifically covered:

* `nora_uart_init()` with `clock_source` / `high_speed`, TX and RX enabled
* blocking byte-stream TX (`write` / `write_byte`) as the console's printer
* polling RX (`rx_ready` / `read_byte`) as the console's reader
* **RX error recovery.** A `FERR` / `PERR` / `OERR` that is *reported but not ended* leaves
  reception dead forever; the polling path clears the condition instead. The counter for
  overrun recoveries read **0** throughout the session, and the whole test for this failure
  mode is "the console still answers after every reset path", which it did.

**Compiled, not executed — read this before relying on any of it:**

| what | why not |
|---|---|
| the **async** TX/RX engine (`tx_start`, `rx_start`, `rx_start_clean`, `tx_abort`, `rx_abort`, `*_count_get`, `*_is_busy`, `set_callback`, `tx_irq_handler`) | no caller in the validating application. Measured on the tested build: `--gc-sections` **discarded all of them from the firmware image**, together with `nora_uart_set_baudrate()`, `nora_uart_tx_is_enabled()` and `nora_uart_rx_regs_get()`. They compile; they were not in the binary, so no amount of hardware exercise covers them |
| the **ISR-ring** RX backend (`NORA_UART_RX_MODE_ISR_RING`, `nora_uart_dspic33ck_rx_isr_ring.c`) | nothing in the tree selects it. It is additionally **excluded from the EV88G73A build** and compiled only in the `CK256MP508_DM330030` configuration, which is itself compile-only |
| `nora_uart_dspic33ck_isr.c` (the `_UxRXInterrupt` vector wrappers) | **excluded from both validated configurations, so it has never been compiled there**. It is the opt-in half of vector ownership (see below), and the standalone syntax check in this repository is the first thing that has ever compiled it |
| UART2 / UART3 | only UART1 is used on either board. The instance enum reaches three, and presence is a device question (`nora_uart_is_present()`) |
| dsPIC33CK256MP508 | that configuration is compile-only |

## Interrupt vector ownership

Two mutually exclusive arrangements, and this is the one place where an integration must
make a choice:

- **The application owns the vectors** (what both upstream boards do, and what the AK HAL
  always does). `nora_uart_rx_irq_handler()` and `nora_uart_tx_irq_handler()` are ordinary
  functions, not vector declarations; define `_UxRXInterrupt` / `_UxTXInterrupt` yourself
  and forward. Do not compile `nora_uart_dspic33ck_isr.c`.
- **The HAL owns the vectors.** Compile `nora_uart_dspic33ck_isr.c`. It defines
  `_UxRXInterrupt` for each instance whose RX flag symbol exists on the device and forwards
  into the ring handler, and it can define `_UxTXInterrupt` the same way. Two independent
  switches, with **different defaults**:

  | macro | default | effect |
  |---|---|---|
  | `NORA_UART_HAL_OWNS_RX_VECTORS` | **1** | the file's RX vectors are compiled; set `0` when you supply your own |
  | `NORA_UART_HAL_OWNS_TX_VECTORS` | **0** | set `1` to get the file's TX vectors; otherwise supply your own `_UxTXInterrupt` forwarding to `nora_uart_tx_irq_handler()` |

  The asymmetry is deliberate rather than an oversight: defining a TX vector references
  `nora_uart_tx_irq_handler()`, which anchors the whole async TX engine against
  `--gc-sections`, so a build that never calls `nora_uart_tx_start()` would pay ROM for
  code it cannot reach. On a part sitting at 97 % of flash that is the difference that
  matters.

Two warnings about that file, both worth knowing before you add it to a project:

1. **It has never been compiled upstream** (see Status). The standalone syntax check here
   passes for both devices, which is evidence of nothing beyond syntax.
2. **It also carries the `_UxTXInterrupt` wrappers**, off by default — see the table above.
   Turning them on is what makes the async TX engine reachable without writing a vector.

So async TX has two valid arrangements too — `NORA_UART_HAL_OWNS_TX_VECTORS=1`, or your own
vector — and the default is the second one.

## Design policy

* The public API exposes no XC-DSC / DFP bitfield types.
* PPS routing, GPIO setup, and clock bring-up stay outside this HAL.
* `printf()` / `read()` / `write()` retargeting and console command parsing stay outside
  this HAL.
* No dynamic memory. **RX ring storage is caller-provided**, so the HAL holds no implicit
  RAM.
* Timeout handling is opt-in: it exists only if the caller supplies `get_ms`.
* Device registers and the scattered `_UxRXIF` / `_UxRXIE` / `_UxRXIP` (and TX) bits are
  isolated in the device layer, reached through per-instance accessors that write the DFP
  bit aliases in a `switch` over the instance rather than through an `IFSx`/`IECx`
  pointer-and-mask table. A hand-maintained pointer table can name the wrong register for
  one instance and silently kill that instance's RX with nothing to catch it at build time.
* **No silent fallbacks.** `ISR_RING` requires `enable_rx`, a buffer of at least 2 bytes, a
  non-zero IRQ priority, an instance with an RX interrupt mapping, and the ring object
  linked in; `nora_uart_init()` returns `ERR_UNSUPPORTED` / `ERR_INVALID_ARG` rather than
  quietly running in polling mode, because a silent fallback looks exactly like a working
  ring.
* **Interrupt priorities are range-checked.** A CPU priority is a 3-bit field: 1..7 are the
  usable levels and 0 means *masked by CPU priority rules*. A value above 7 is therefore not
  a slow interrupt but a truncated one — 8 lands as 0 and silently disables the line — so
  `nora_uart_init()` refuses `rx_irq_priority > 7` or `tx_irq_priority > 7` with
  `ERR_INVALID_ARG` before programming anything. A **non-zero** `tx_irq_priority` on an
  instance whose TX priority symbol this device does not name is refused with
  `ERR_UNSUPPORTED` and the init is undone, rather than letting a later async TX start at
  whatever priority reset left. Zero is not a request, so it is not an error: it is what an
  integration that never calls `nora_uart_tx_start()` passes.

## Files

```text
src/
  nora_uart.h                          public API (byte stream, RX backends, async model)
  nora_uart_dspic33ck.c                HAL implementation
  nora_uart_dspic33ck_device.{c,h}     device register + RX/TX IRQ mapping
  nora_uart_dspic33ck_reg.h            internal register / bit-mask helpers
  nora_uart_dspic33ck_rx_isr_ring.{c,h}  RX ISR ring backend        -- unexercised
  nora_uart_dspic33ck_isr.c            optional _UxRXInterrupt wrappers -- never compiled upstream
```

## Basic usage (the validated path)

```c
#include "nora_uart.h"

static uint32_t app_get_ms(void) { return app_millisecond_tick; }

const nora_uart_config_t cfg = {
    .uart_clk_hz  = 100000000u,                  /* the clock reaching the BRG */
    .baudrate     = 230400u,
    .timeout_ms   = 10u,
    .get_ms       = app_get_ms,                  /* NULL disables timeouts entirely */
    .data_bits    = 8u,
    .stop_bits    = 1u,
    .parity       = NORA_UART_PARITY_NONE,
    .high_speed   = true,                        /* BRGH */
    .clock_source = NORA_UART_BCLKSEL_FOSC_DIV2, /* BCLKSEL */
    .enable_tx    = true,
    .enable_rx    = true,
    .rx_mode      = NORA_UART_RX_MODE_POLLING,   /* a zeroed config selects this */
};   /* rx_irq_priority / tx_irq_priority left 0: neither interrupt is used here */

if (nora_uart_init(NORA_UART_INST_1, &cfg) != NORA_UART_OK) { /* ... */ }

nora_uart_write_byte(NORA_UART_INST_1, 'x');

if (nora_uart_rx_ready(NORA_UART_INST_1)) {
    uint8_t b;
    (void)nora_uart_read_byte(NORA_UART_INST_1, &b);
}
```

`uart_clk_hz` must be the frequency that actually reaches the baud-rate generator for the
selected `clock_source`. The HAL cannot detect a mismatch — it only computes from what it is
told. `nora_uart_get_baudrate()` returns the **last baud rate requested** and stored, not a
rate measured back from the divisor: the BRG divisor is an integer, so the bits on the wire
are generally close to the requested rate rather than equal to it.

**On the RX error path:** `nora_uart_rx_ready()` is what clears an overrun condition, so
polling it is not optional bookkeeping — a receiver that only reports `OERR` and returns
never receives again. `nora_uart_rx_status_get()` reports the backend in use, the bytes
handed to the reader, framing/parity errors discarded, and the overrun recoveries
performed; `nora_uart_rx_status_clear()` resets the counters.

## RX ISR ring mode (unexercised)

```c
static uint8_t rx_ring[256];

nora_uart_config_t cfg = /* as above, plus: */ {
    .rx_mode             = NORA_UART_RX_MODE_ISR_RING,
    .rx_ring_buffer      = rx_ring,
    .rx_ring_buffer_size = sizeof rx_ring,   /* >= 2 */
    .rx_irq_priority     = 4u,               /* non-zero */
};

/* and either compile nora_uart_dspic33ck_isr.c, or define the vector yourself: */
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{
    nora_uart_rx_irq_handler(NORA_UART_INST_1);
}
```

In ring mode `rx_ready()` / `read_byte()` / `rx_flush()` operate on the software ring
instead of the hardware FIFO. The ring is single-producer (ISR) / single-consumer (reader).

## Async TX/RX (unexercised)

`nora_uart_set_callback()` registers the completion/error callback; `nora_uart_tx_start()`
and `nora_uart_rx_start()` / `nora_uart_rx_start_clean()` begin a non-blocking transfer, and
the TX interrupt vector must forward to `nora_uart_tx_irq_handler()`. `tx_irq_priority` is
programmed at init and must be non-zero for async TX; the TX interrupt stays disabled until
a transfer starts. `tx_abort()` / `rx_abort()` stop an active transfer, and the
`*_count_get()` calls report exact byte counts. The byte-stream API and the RX ring keep
working unchanged — the async model is additive.

## Notes

* This repository does not include Microchip DFP header files.
* An instance whose `UxCON` exists but whose interrupt flag/enable aliases do not is a
  compile-time `#error`, not a runtime surprise.
* Sibling repositories for this family:
  [dma](https://github.com/sulaolab/nora-hal-dspic33ck-dma) ·
  [timer](https://github.com/sulaolab/nora-hal-dspic33ck-timer) ·
  [gpio](https://github.com/sulaolab/nora-hal-dspic33ck-gpio) ·
  [clock](https://github.com/sulaolab/nora-hal-dspic33ck-clock) ·
  [i2c](https://github.com/sulaolab/nora-hal-dspic33ck-i2c) ·
  [spi-i2s-tdm](https://github.com/sulaolab/nora-hal-dspic33ck-spi-i2s-tdm)

## License

MIT No Attribution License (MIT-0). See [LICENSE](LICENSE).

Attribution is appreciated but not required.
