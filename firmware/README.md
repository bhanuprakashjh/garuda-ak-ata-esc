# Firmware tree

MPLAB X project for the dsPIC33AK128MC106 + ATA6847L on the EV43F54A
board. Toolchain: XC-DSC v3.30, DFP dsPIC33AK-MC v1.4.172.

## Quick layout

| Path | Role |
|---|---|
| `garuda_config.h` | All compile-time configuration (motor profile, feature flags, tuning knobs). |
| `garuda_service.c/h` | Main loop, ISR dispatch, `V4_ProcessBemfSample`. |
| `garuda_types.h` | Shared types (`COMMUTATION_STEP_T`, etc.). |
| `main.c` | Init order + service loop entry. |
| `Makefile` | CLI build wrapper. |
| `motor/` | V4 set-point PI scheduler, OL ramp, CL handoff, commutation table. |
| `hal/` | PWM, ATA6847 SPI, PTG, SCCP3 commutation timer, SCCP IC, ADC, GPIO. |
| `gsp/` | Garuda Serial Protocol (binary, 115200 baud) — parser, snapshot encoder, command dispatch. |
| `input/` | Pot/throttle input. |
| `scope/` | On-chip ring-buffer scope (12 channels, 4 trigger modes). |
| `tools/` | Python host tools — telemetry monitor, GSP test harness. |
| `nbproject/` | MPLAB X project metadata. |

## Build (CLI)

```bash
make -f nbproject/Makefile-default.mk SUBPROJECTS= .build-conf
# Output: dist/default/production/garuda_ata6847_ak.X.production.{hex,elf}
```

## Build (MPLAB X GUI)

Open this directory as an MPLAB X project. Edit `garuda_config.h`
line `#define MOTOR_PROFILE` to pick the bench motor before Build &
Program.

## Architecture

See `../docs/v4_ak_port_scheduler.md` for the full architecture +
every load-bearing variable + every tuning knob.
