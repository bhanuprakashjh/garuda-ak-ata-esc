# garuda-ak-ata-esc

6-step trapezoidal sensorless BLDC ESC firmware for the
**dsPIC33AK128MC106 + ATA6847L** on the Microchip **EV43F54A**
evaluation board.

> Bench-verified to **225 k eRPM** on a bare 2810 outrunner @ 24 V
> (Profile 2). Block-commutation engages cleanly above ~210 k.

---

## Hardware

| Part | Role |
|---|---|
| **dsPIC33AK128MC106** | Motor-control MCU. Fcy = Fosc = 200 MHz, hardware FPU, 128 KB flash, 16 KB RAM. |
| **ATA6847L** | Integrated 3-phase gate driver with per-phase digital BEMF comparators (output is **inverted**: HIGH = BEMF below virtual neutral). |
| **EV43F54A** | Microchip evaluation board hosting the MCU module + ATA6847L driver. |
| **2810 / A2212 / Hurst DMB2424B** | Bench motors covering 150–1400 KV range. |

The ATA6847L's per-phase comparators give us **digital ZC detection**
without analog BEMF sampling. The comparator output is **inverted**
(`comp = 1` ↔ BEMF < neutral); this convention is load-bearing
throughout the firmware.

---

## Repository layout

```
.
├── docs/                              ← architecture + tuning history
│   ├── garuda_6step_ak_port_plan.md   ← original CK→AK port plan
│   ├── v4_ak_port_scheduler.md        ← deep architecture reference
│   └── v4_motor_tuning_guide.md       ← symptom → knob lookup
├── firmware/                          ← MPLAB X project root
│   ├── README.md                      ← canonical architecture + API doc (READ FIRST)
│   ├── garuda_config.h                ← compile-time config (profiles, flags, knobs)
│   ├── garuda_service.{c,h}           ← state machine, ISRs, BEMF sample classifier
│   ├── main.c                         ← boot + main loop
│   ├── motor/                         ← sector PI synchronizer + commutation table
│   ├── hal/                           ← PWM, ATA6847 SPI, PTG, ADC, timers, etc.
│   ├── gsp/                           ← Garuda Serial Protocol (UART1 @ 115200)
│   ├── tools/                         ← Python host monitor + diagnostic scripts
│   ├── nbproject/                     ← MPLAB X project metadata (synced with source)
│   ├── setup-build.sh                 ← bootstrap script for first CLI build
│   └── Makefile                       ← CLI wrapper around nbproject/Makefile-default.mk
└── .gitignore
```

The `firmware/README.md` is the authoritative reference for the
control algorithm, ISR map, telemetry layout, and runtime parameters.
The `docs/` folder contains older architecture and tuning material
from the port's V4 era; pieces of it are superseded by the current
firmware README — treat as historical reference unless explicitly
updated.

---

## Build

### MPLAB X (GUI)

1. Open `firmware/` as an MPLAB X project (`File → Open Project`).
2. Toolchain: **XC-DSC v3.30** or later. DFP: **dsPIC33AK-MC v1.4.172** or later.
3. Pick the bench motor in `firmware/garuda_config.h`:
   ```c
   #define MOTOR_PROFILE   2   // 0 = Hurst, 1 = A2212, 2 = 2810, 3 = HiZ1460
   ```
4. **Build & Program** with PKoB 4 / ICD attached.

MPLAB X regenerates the `.generated_files/flags/default/` placeholders
automatically on first project open. No setup needed.

### CLI (Makefile)

A fresh clone needs the bootstrap step once before `make` works,
because the MPLAB-generated flag-file stubs are gitignored:

```bash
cd firmware
./setup-build.sh                  # creates .generated_files/flags placeholders
make CONF=default                 # production build, ~3 s
# .hex lands in dist/default/production/
```

Subsequent builds just run `make CONF=default`.

### Flash via MDB

```bash
/path/to/MPLABX/bin/mdb.sh
> program dist/default/production/garuda_ata6847_ak.X.production.elf
> run
> quit
```

---

## Bench workflow

Host-side Python tool monitors telemetry over UART1 at 115200 baud:

```bash
python3 firmware/tools/gsp_ak_test.py mon /dev/ttyACM1 115200 run.csv
```

Each printed row shows ESC state, sector index, throttle, duty,
amplitude, Vbus, peak currents, eRPM, period (`Tp`), PI delta, capture
counters (`cR` / `cF`), per-polarity PI feed rates (`pR` / `pF`),
the post-ZC shadow ratio (`postF`), and the main-loop heartbeat. On
Ctrl-C the run CSV is closed with a summary footer.

The `postF` percentage (typically 80–95% in steady state) is the
primary BEMF-detection health indicator on this hardware. The `pR`
rising-feed rate stays near 0% by physics — rising-sector samples
consistently land POST-ZC of rising on this MCU + driver combo. See
`firmware/README.md` § BEMF detection for the full explanation.

---

## What works today

| Item | Status |
|---|---|
| ALIGN → CL direct startup | ✓ (OL ramp removed — wasn't actually syncing the rotor) |
| 225 k eRPM bare 2810 @ 24 V, 60 kHz PWM | ✓ |
| Block commutation above `BLOCK_ENTER_ERPM` | ✓ |
| Real-time GUI telemetry over GSP | ✓ |
| Polarity-symmetric BEMF gate (CK-matched) | ✓ |
| Stall recovery / restart without board reset | ✓ |
| 40 kHz PWM operation | partial (motor walls around 137 k; sample-quantization latency dominates) |
| Hardware ZC edge capture (SCCP IC) | not used — comparator outputs not routed to an IC-capable PPS on this MCU; experimental path was tried and removed (chatter trap) |

The current state machine is **`IDLE → ARMED → ALIGN → CLOSED_LOOP`**.
The `ESC_OL_RAMP` enum value still exists for ABI stability but no
code transitions into it — the legacy OL ramp branch was removed when
bench data showed it didn't actually sync the rotor.

---

## Critical hardware lessons

Before changing anything in the control or BEMF paths, internalize:

1. **ATA6847 comparator is INVERTED.** `comp = 1` ↔ BEMF below virtual
   neutral. Rising ZC produces a falling comparator edge; falling ZC
   produces a rising one. This convention is load-bearing in
   `ProcessBemfSample` (garuda_service.c) and the polarity-symmetric
   acceptance gate.

2. **Atomic per-sector snapshot.** The `sectorSnap` u16 packs
   `currentSector`, `floatingPhase`, and `ptgExpectedComp` and is
   written **back-to-back** in `SectorPI_Commutate` so the PTG ISR
   (lower IPL) sees a coherent triple. Adding any non-atomic writes
   in Commutate to those globals breaks BEMF detection.

3. **MPLAB build reads `MOTOR_PROFILE` from `garuda_config.h`**, *not*
   from a command-line `-D` flag. Edit the file.

4. **AK has SCCP1–4 only** (no SCCP5). All SCCP IC and CLC code from
   the CK board was either removed or not ported.

5. **`__builtin_disi` is not supported on XC-DSC.** Use seqlocks or
   double-read consistency for atomic snapshots.

6. **`UPDREQ = 1` must be written to every PG generator** in 6-step
   mode (not just the master).

---

## Status

Active development. Bench platform: EV43F54A + 2810 @ 24 V.
Verified peak as of `2026-05-20`: **225 k eRPM**, sustained with block
commutation, clean restart-without-reset path. Cleanup pass on
`2026-05-20` removed ~1000 lines of disproved-experimental scaffolding
(`hal_capture` stubs, `FEATURE_HW_ZC_*` flags, `piFeedPolarity` runtime
knob, multi-phase BEMF probe, dead OL ramp branch).

See `firmware/README.md` for the full architecture, telemetry, and
tuning workflow.
