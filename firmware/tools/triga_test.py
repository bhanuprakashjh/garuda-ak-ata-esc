#!/usr/bin/env python3
"""Test a specific PG1TRIGA position with live telemetry monitor.

Theory: AK's pR_pct=90% is because we sample mid-ON (CAHALF=1, TRIGA=0 =
cycle 1/2 boundary = pulse center). CK samples mid-OFF (PG1TRIGA=0 with
CAHALF=0 = period boundary = midway between pulses) which gives clean
BEMF reading with no di/dt coupling.

Usage:
    python3 triga_test.py [--pos midoff|midon|<encoded>] [/dev/ttyACM1]

Encodings:
    midoff  → 0x0000  (CAHALF=0, TRIGA=0 — period boundary, CK-equivalent)
    midon   → 0x8000  (CAHALF=1, TRIGA=0 — cycle boundary, current AK)

What to watch:
    pR%  — if it flips from 90% → ~50%, theory confirmed; comp is now reading
           pure BEMF and transitions cleanly mid-sector
    pF%  — if it climbs from 0% → similar range, falling sectors are now
           seeing usable comp readings too
    cR / cF — actual capture counters (should track pR/pF behaviour)
"""
import sys, time, struct, serial, argparse

STX = 0x02
GSP_CMD_SET_PARAM    = 0x11
GSP_CMD_TELEM_START  = 0x14
GSP_CMD_TELEM_STOP   = 0x15
V4_PARAM_TRIGA_POS   = 0xF6

POSITIONS = {
    'midoff': 0x0000,   # CAHALF=0, TRIGA=0  — period boundary = mid-OFF (CK)
    'midon':  0x8000,   # CAHALF=1, TRIGA=0  — cycle boundary  = mid-ON  (current AK)
}


def crc16_ccitt(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        c ^= (b << 8) & 0xFFFF
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if (c & 0x8000) else (c << 1) & 0xFFFF
    return c


def frame(cmd: int, payload: bytes = b'') -> bytes:
    body = bytes([len(payload) + 1, cmd]) + payload
    return bytes([STX]) + body + struct.pack('>H', crc16_ccitt(body))


_rx_buf = bytearray()


def read_one_frame(ser, timeout_s: float = 0.5):
    ser.timeout = 0.05
    end = time.time() + timeout_s
    while time.time() < end:
        chunk = ser.read(256)
        if chunk:
            _rx_buf.extend(chunk)
        while len(_rx_buf) >= 5:
            if _rx_buf[0] != STX:
                _rx_buf.pop(0)
                continue
            length = _rx_buf[1]
            if len(_rx_buf) < 2 + length + 2:
                break
            cmd = _rx_buf[2]
            payload = bytes(_rx_buf[3:2 + length])
            crc_rx = struct.unpack('>H', _rx_buf[2 + length:4 + length])[0]
            crc_calc = crc16_ccitt(bytes(_rx_buf[1:2 + length]))
            del _rx_buf[:4 + length]
            if crc_rx == crc_calc:
                return (cmd, payload)
    return None


def decode_line(payload: bytes) -> str:
    seq = payload[0] | (payload[1] << 8)
    state = payload[2]
    potRaw = payload[6] | (payload[7] << 8)
    dutyPct = payload[8]
    vbusRaw = payload[10] | (payload[11] << 8)
    timerPer = payload[20] | (payload[21] << 8)
    eRpm = struct.unpack('<I', payload[24:28])[0]
    capSet = struct.unpack('<I', payload[58:62])[0]
    capRise = struct.unpack('<I', payload[62:66])[0]
    capFall = capSet - capRise
    pR_acc = pR_rej = pF_acc = pF_rej = 0
    if len(payload) >= 110:
        pR_acc = struct.unpack('<I', payload[94:98])[0]
        pR_rej = struct.unpack('<I', payload[98:102])[0]
        pF_acc = struct.unpack('<I', payload[102:106])[0]
        pF_rej = struct.unpack('<I', payload[106:110])[0]
    pR_pct = (100 * pR_acc // (pR_acc + pR_rej)) if (pR_acc + pR_rej) else 0
    pF_pct = (100 * pF_acc // (pF_acc + pF_rej)) if (pF_acc + pF_rej) else 0
    ESC_STATE = {0: "IDLE", 1: "ARM ", 2: "ALN ", 3: "RAMP",
                 4: "CL  ", 5: "RCV ", 6: "FLT "}
    name = ESC_STATE.get(state, f"S{state}")
    vbusV = vbusRaw / 100.0
    def kfmt(n):
        if n >= 1_000_000: return f"{n/1e6:4.1f}M"
        if n >= 10_000:    return f"{n//1000:4d}k"
        return f"{n:5d}"
    return (f"#{seq:5d} {name} pot={potRaw:4d} d={dutyPct:3d}% "
            f"V={vbusV:5.2f} RPM={eRpm:7d} Tp={timerPer:5d}  "
            f"cR={kfmt(capRise)} cF={kfmt(capFall)}  "
            f"pR={pR_pct:3d}% pF={pF_pct:3d}%")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pos', default='midoff',
                    help='midoff | midon | <0xNNNN hex>')
    ap.add_argument('port', nargs='?', default='/dev/ttyACM1')
    ap.add_argument('--baud', type=int, default=115200)
    args = ap.parse_args()

    if args.pos in POSITIONS:
        value = POSITIONS[args.pos]
        label = args.pos
    else:
        value = int(args.pos, 0)
        label = f"0x{value:04X}"

    cahalf = (value >> 15) & 1
    triga = value & 0x7FFF
    print(f"Setting PG1TRIGA: {label} → CAHALF={cahalf}, TRIGA={triga} (encoded 0x{value:04X})")
    print("Watch the pR% / pF% columns as you ramp the pot:")
    print("  midoff theory: pR should drop from 90% to ~50% and pF should climb.")
    print("  Ctrl-C to stop.\n")

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.2)
    ser.reset_input_buffer()
    _rx_buf.clear()

    # Start telemetry stream.
    ser.write(frame(GSP_CMD_TELEM_START))
    ser.flush()
    time.sleep(0.1)

    # Apply the TRIGA position.
    payload = struct.pack('<HI', V4_PARAM_TRIGA_POS, value & 0xFFFFFFFF)
    ser.write(frame(GSP_CMD_SET_PARAM, payload))
    ser.flush()
    time.sleep(0.1)

    try:
        while True:
            r = read_one_frame(ser, timeout_s=1.0)
            if r is None:
                continue
            cmd, payload = r
            if cmd == 0x80 and len(payload) >= 66:
                print(decode_line(payload))
    except KeyboardInterrupt:
        pass
    finally:
        # Restore the firmware baseline (mid-ON) before exit.
        baseline = POSITIONS['midon']
        payload = struct.pack('<HI', V4_PARAM_TRIGA_POS, baseline & 0xFFFFFFFF)
        try:
            ser.write(frame(GSP_CMD_SET_PARAM, payload))
            ser.write(frame(GSP_CMD_TELEM_STOP))
            ser.flush()
        except Exception:
            pass
        ser.close()
        print(f"\nRestored PG1TRIGA to baseline (mid-ON, 0x{baseline:04X}).")


if __name__ == '__main__':
    main()
