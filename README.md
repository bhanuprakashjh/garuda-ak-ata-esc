# garuda-ak-ata-esc

6-step trapezoidal sensorless BLDC ESC firmware for the
**dsPIC33AK128MC106 + ATA6847L** on the Microchip **EV43F54A**
evaluation board.

> AK port of the Garuda 6-step ESC. Targets drone-class outrunners
> (1000–2000 KV, 7 PP) at 12–24 V. Bench-verified to **226 k eRPM**
> bare 2810 @ 24 V.

---

## Hardware

| Part | Role |
|---|---|
| **dsPIC33AK128MC106** | Motor control MCU (200 MHz Fp, hardware FPU, 128 KB flash, 16 KB RAM) |
| **ATA6847L** | Integrated 3-phase gate driver with digital BEMF comparators (inverted output) |
| **EV43F54A** | Microchip evaluation board hosting the MCU + driver |
| **2810 / A2212** | Bench motors (1000–1400 KV outrunners) |

The ATA6847L's per-phase comparators give us **digital ZC detection**
without ADC sampling of analog BEMF. The comparator output is
**inverted** — comparator HIGH means BEMF below the virtual neutral.
This convention is load-bearing throughout the firmware; see
`docs/v4_ak_port_scheduler.md` §3.

---

## Repository layout

```
.
├── docs/
│   ├── v4_ak_port_scheduler.md      ← deep architecture + tuning reference
│   ├── garuda_6step_ak_port_plan.md ← original CK→AK port plan
│   └── v4_motor_tuning_guide.md     ← symptom→knob lookup
├── firmware/
│   ├── garuda_config.h        ← ALL compile-time config (profiles, flags, knobs)
│   ├── garuda_service.c       ← Main loop, ISRs, V4_ProcessBemfSample
│   ├── garuda_service.h
│   ├── garuda_types.h
│   ├── main.c                 ← Init + service loop entry
│   ├── Makefile               ← CLI build wrapper
│   ├── nbproject/             ← MPLAB X project metadata
│   ├── motor/
│   │   ├── sector_pi.c        ← V4 set-point PI scheduler, OL ramp, CL handoff
│   │   ├── commutation.c      ← 6-step commutation table
│   │   ├── startup.c
│   │   └── ...
│   ├── hal/
│   │   ├── hal_pwm.c          ← Center-aligned complementary PWM, 6-step phase mux
│   │   ├── hal_ata6847.c      ← SPI gate-driver register config
│   │   ├── hal_ptg.c          ← PTG step IRQ → V4_ProcessBemfSample
│   │   ├── hal_com_timer.c    ← SCCP3 one-shot Commutate scheduler
│   │   ├── hal_capture.c      ← SCCP IC for hardware ZC capture (alt path)
│   │   ├── hal_adc.c          ← Vbus + currents
│   │   └── port_config.h      ← GPIO/PPS macros for EV43F54A
│   ├── gsp/                   ← Garuda Serial Protocol (binary, 115200 baud)
│   ├── input/                 ← Pot/throttle input
│   ├── scope/                 ← On-chip ring-buffer scope
│   └── tools/                 ← Python host tools (bench/telemetry)
└── .gitignore
```

---

## Build

### MPLAB X (GUI)

1. Open `firmware/` as an MPLAB X project.
2. Toolchain: XC-DSC v3.30 or later. DFP: dsPIC33AK-MC v1.4.172 or later.
3. Edit `firmware/garuda_config.h` line `#define MOTOR_PROFILE` to pick
   the bench motor (0 = Hurst, 1 = A2212, 2 = 2810, 3 = HiZ1460).
4. Build and program.

### CLI (Makefile)

```bash
cd firmware
make -f nbproject/Makefile-default.mk SUBPROJECTS= .build-conf
# .hex lands in dist/default/production/
```

Programming via MDB (PKoB 4):

```bash
/path/to/mplab_platform/bin/mdb.sh
> program dist/default/production/garuda_ata6847_ak.X.production.elf
> run
```

---

## Bench workflow

Host-side Python tool monitors telemetry over UART1:

```bash
python3 firmware/tools/gsp_ak_test.py mon /dev/ttyACM1 115200 run.csv
```

Each line prints sector hits, PI period (`Tp`), measured eRPM, peak
currents, capture counters (`cR`, `cF`), and shadow diagnostics
(`preR`, `postF`, `fpStale`). On Ctrl-C the tool emits an
end-of-run multi-phase BEMF tally showing the per-(sector, phase)
comp=1 ratios — useful for catching phase-mapping bugs.

See `docs/v4_ak_port_scheduler.md` §9 for what every counter means.

---

## What works today

| Item | Status |
|---|---|
| Open-loop ramp → closed-loop handoff | ✓ |
| 226 k eRPM bare 2810 @ 24 V | ✓ |
| Block commutation above ~210 k eRPM | ✓ |
| Real-time GUI telemetry over GSP | ✓ |
| Per-sector multi-phase BEMF tally diagnostic | ✓ |
| Rising-sector PI feed | partial (4 captures at CL entry, then 2T:ε pacing only feeds falling) |
| Both-polarity continuous PI feed | requires V5.3 scheduler — current code is gated off behind `FEATURE_V5_SCHEDULER` |

The 2T:ε pacing of the V4 reactive scheduler is documented in
`docs/v4_ak_port_scheduler.md` §7. It's not a bug — it's the load-
bearing mechanism that gets us 226 k eRPM with the current advance
schedule. A future 1:1 scheduler rewrite would change this.

---

## Critical hardware lessons (read these before changing anything)

1. **ATA6847 comparator is INVERTED.** Output HIGH = BEMF below
   neutral = pre-ZC for rising sectors. This convention is
   load-bearing in `V4_ProcessBemfSample` and the SP-mode CCP ISR.
2. **PTG ISR priority < Commutate ISR priority** (5 vs 6). Commutate
   can preempt PTG mid-flight. The three per-sector globals
   (`v4_currentSector`, `v4_floatingPhase`, `v5_ptgExpectedComp`)
   MUST be written back-to-back in Commutate or PTG sees an
   incoherent snapshot. See sector_pi.c step 3 and the doc.
3. **`measuredCommPeriod` reads 2× the true sector period** because
   of the V4 scheduler's ASAP-pair. Anything that consumes it
   (blanking, fallback schedule, PI scale) inherits that scaling.
4. **MPLAB build uses `MOTOR_PROFILE` from `garuda_config.h`**, NOT
   a command-line `-D` flag. Edit the file.

---

## Reference materials

- Microchip dsPIC33AK128MC106 datasheet (DS70005527)
- Microchip ATA6847L datasheet
- Microchip AN6285: ATA6847L → dsPIC33CK connections (the EV43F54A wiring is derived from this)
- Microchip AVR motor-control high-speed reference (250 k+ eRPM with ~200 lines — the design intent V4 follows)

---

## Status

Active development branch. Bench platform: EV43F54A + 2810 @ 24 V.
Verified peak as of `2026-05-15`: **226 k eRPM** clean across a full
pot sweep, both directions.
