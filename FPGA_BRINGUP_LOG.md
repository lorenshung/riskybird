# riskybird v3 — FPGA / RISC-V SoC bring-up log

Chronological record of bringing up the Trenz TE0712 (Artix-7 **XC7A200T**) FPGA and the
**RV64GC Rocket** SoC it hosts: flashing bitstreams, loading ELFs, and reproducing the
sensor tests. Newest entries at the bottom. Companion to `samples/rose_flight_controller/docs/FLIGHT_TUNING_LOG.md`
(the ESP32 flight-controller log) and memory `riskybird-v3-fpga-bringup`.

## Setup (hardware + tools)
- **FPGA:** Trenz TE0712 XC7A200T (FBG484), on the `riskybirdv3_connector` programmer → `riskybirdv3_base` drone board.
- **Two FTDIs (distinct roles):** the **bitstream FTDI** (USB path 3-3) = FPGA-config JTAG (`openFPGALoader`);
  the **binary FTDI** (path 3-1) = Rocket **debug JTAG on ch 0** + **UART console on ch 1 = `/dev/ttyUSB1` @ 115200**.
  Both enumerate `0403:6010` after the EEPROM program → use **`-c ft2232`**.
- **ESP32-C6** (flight controller) on `/dev/ttyACM0` — shares the sensor I2C bus with the FPGA.
- **Tools:** `openFPGALoader v1.1.1`, `openocd 0.12`, `riscv64-elf-gdb`, Zephyr SDK 1.0.0-beta1 (LLVM → use `llvm-nm`).
- **Collaborator repos** (cloned to `scratchpad/collab/`): `github.com/lorenshung/{bitstream,elf}` — bitstreams, prebuilt ELFs, DT overlays.

## Log

| # | date | step | result |
|---|------|------|--------|
| 1 | 09-01 | `openFPGALoader --detect -c ft4232` | XC7A200T detected — `idcode 0x03636093`, artix a7 200t, irlen 6, single tap |
| 2 | 09-01 | read config `STAT` | `0x5000190c` — healthy + unconfigured (DONE=0, INIT_Complete=1, no CRC/ID/temp errors) |
| 3 | 09-01 | flash `RocketArty200TMotorSpadConfig.bit` to SPI flash (`-c ft4232 --fpga-part xc7a200tfbg484 -f`) | wrote 9.76 MB to Spansion **S25FL256S** @0x3b0000; booted from flash — `STAT=0x501079fc`, **DONE=1**, EOS=1, no CRC error |
| 4 | 09-01 | program bitstream-FTDI EEPROM (93LC66C) via `ftdi_eeprom` | now enumerates FT2232H **`0x6010`** → switch to `-c ft2232`. ID strings (`riskybird` / `riskybird bitstream` / `RBV3-BIT-01`) written (confirmed in raw dump) but **enumerate blank** — libftdi 1.5 string-offset bug on the 128/256B size mis-detect for this remarked chip. Recover with `--erase-eeprom`. |
| 5 | 09-02 | clone collaborator `bitstream` + `elf` repos | 271 MB + 65 MB; match ELF↔shell via `elf/manifest.json` `ram_bytes` (32 KiB spad vs 1 GiB DDR) |
| 6 | 09-02 | flash `RocketArty200TDroneDDRConfig.bit` (`-c ft2232 --fpga-part xc7a200tfbg484 -f`) | write OK (100%, exit 0). Post-write auto-reload **didn't fire over ft2232** (PROGRAM_B on the wrong channel → DONE=0). SRAM-load verified bitstream good (`DONE=1`). **Power-cycle to boot from flash.** |
| 7 | 09-02 | RISC-V bring-up: `openocd` (binary FTDI) → Rocket; `load_image ddr_stress.default.elf` + `resume 0x80000000` | **RV64GC Rocket alive** (`misa 0x800000000094112d`, 1 hart). **DDR PASSED — 0 errors** (64 MiB, addr-in-addr + 0x55/0xAA), ~27–37 MB/s, on `ttyUSB1`. |
| 8 | 09-02 | **build hello_world ourselves** (board `chipyard_riscv64` + repo overlays `fpga-common;arty200t`) | boots: `Hello World! chipyard_riscv64/rocketchip_virt_riscv64`. Needed 3 Kconfigs (from the `chipyard/dsp24` board — the real-HW reference): `UART_SIFIVE_PORT_0=y`, `UART_HTIF=n`, **`SYS_CLOCK_HW_CYCLES_PER_SEC=50000`** (→ 50 MHz peripheral clock → 115200 baud; the board default 1 GHz gave ~5760 baud = whitespace). |
| 9 | 09-02 | **ESP32 → dummy firmware** (release the shared I2C bus before scanning) | **done** — bare hello_world (144 KB) flashed to ESP32-C6 (`ttyACM0`), hash verified. FC no longer drives the sensor pins; bus free for the FPGA. |
| 10 | 09-02 | build + load `i2c_scanner` on the Rocket (on `DroneDDR`) | **DroneDDR has no I2C** — faulted `mcause 7 (store access fault) @ 0x10040008` (i2c0 base, no slave). Confirmed by `VERIFIED.md`: DroneDDR is DDR-only; sensor tests need a shell with the I2C/SPI/PWM periphery. |
| 11 | 09-02 | **build a full-peripheral DDR bitstream** on `dima-garden` (Vivado 2023.1) | *in progress* — clone `lorenshung/chipyard@ku040-codesign-cnn` → `/scratch/dima/riskybird_chipyard`; added **`RocketArty200TDroneFullDDRConfig`** (= MotorSpad periphery: DDR3 + I2C + SPI + PWM×2 + GPIO + camera OSPI, `ddr=true`) **+ a 2nd peripheral UART for the ESP** (`WithUART(0x10021000)`, bound to **E13=rxd←ESP-TX / F14=txd→ESP-RX**; gave `WithArty200TUART` a `uartNo` filter so uart0=console E21/D21, uart1=ESP). chipyard `build-setup --use-lean-conda --skip-toolchain` running; then `make -C fpga SUB_PROJECT=arty200t CONFIG=RocketArty200TDroneFullDDRConfig`. Also **corrected the entire camera DVP pinout** in `WithArty200TOspi` (the fork's H-region defaults were unvalidated placeholders). Derived from the `riskybirdv3` netlist (camera net → JB3 pin) × Trenz `misc/trenz_pinouts_info.xlsx` (`RAW_m_TE0712_REV02`: JM3 pin → ball) with the **hermaphroditic B2B swap** (base JB3.N ↔ module JM3.N±1, i.e. 57↔58) — all bank-13: **PCLK=W11** (user-confirmed), MCLK=W15, LVLD=T14, FVLD=W10, INT=T15, TRIG=V10, D0..7=W16,Y16,AA16,AA15,AB15,AB17,AB16,V13. No DDR conflict (on-module DDR3 uses on-module balls, not these B2B pins). SDC keeps 36 MHz `ospi_pclk` + permissive route. (Camera SCCB I2C pins A15/A16 still the fork default — recheck via the same method when doing camera I2C.) |

## Recipes
- **Detect / status:** `openFPGALoader -c ft2232 --detect` ; `... --read-register STAT` (DONE=1 + no CRC error = configured).
- **Flash a bitstream:** `openFPGALoader -c ft2232 --fpga-part xc7a200tfbg484 -f <bit>` (SRAM-load: drop `-f`). Power-cycle to boot from flash.
- **Load an ELF onto the Rocket:** `openocd` with `ftdi vid_pid 0x0403 0x6010`, `ftdi channel 0`, `ftdi layout_init 0x0B00 0x0B0B`, `jtag newtap riscv cpu -irlen 5`, `target create riscv.cpu riscv`; `init; halt; load_image <elf>; resume 0x80000000`. Console: `ttyUSB1 @ 115200`.
- **Build a Zephyr ELF:** `west build -b chipyard_riscv64 -d <bd> samples/<app> -- -DDTC_OVERLAY_FILE="<repo>/elf/overlays/fpga-common.overlay;.../arty200t.overlay" -DCONFIG_UART_HTIF=n -DCONFIG_UART_SIFIVE=y -DCONFIG_UART_SIFIVE_PORT_0=y -DCONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=50000`. (`arty200t.overlay` = 1 GiB DDR; `arty200t-scratchpad.overlay` = 32 KiB spad.)
