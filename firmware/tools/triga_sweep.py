#!/usr/bin/env python3
"""PG1TRIGA sweep — find ADC trigger position that lights up falling captures.

Bench protocol:
  1. Flash firmware (V4_PARAM_TRIGA_POS = 0xF6 must be wired up).
  2. Get motor running at steady CL — pot to mid-range, let eRpm settle.
  3. Run: python3 triga_sweep.py /dev/ttyACM1
  4. Don't move the pot during the sweep. Each step waits a settling window,
     captures cR/cF deltas, prints one row. Bad rows = motor lost CL at that
     TRIGA position; ignore them.

A position is "interesting" when cF_delta and pF% climb to be comparable to
cR_delta and pR%. Hardcode the winning TRIGA into hal_pwm.c.

Encoding: V4_PARAM_TRIGA_POS bit 15 = CAHALF, bits 0-14 = TRIGA value.
"""
import sys, time, struct, serial, argparse

STX = 0x02
GSP_CMD_SET_PARAM    = 0x11
GSP_CMD_GET_SNAPSHOT = 0x02
GSP_CMD_TELEM_START  = 0x14
GSP_CMD_TELEM_STOP   = 0x15
V4_PARAM_TRIGA_POS   = 0xF6

# AK: LOOPTIME_TCY = (8 * 100MHz / 60kHz) - 16 = 13317 at 60 kHz PWM
LOOPTIME_TCY = 13317


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


# Persistent receive buffer so frames spanning serial read boundaries
# aren't dropped. Indexed by id(ser) so multiple ports in one process work.
_RX_BUFS = {}


def _rx_buf(ser):
    return _RX_BUFS.setdefault(id(ser), bytearray())


def _drain_rx(ser):
    _RX_BUFS[id(ser)] = bytearray()


def read_one_frame(ser, timeout_s: float = 0.5):
    """Return (cmd, payload) of the next valid frame, or None on timeout.
    Maintains a persistent buffer across calls so multi-chunk frames are
    reassembled correctly."""
    ser.timeout = 0.05
    end = time.time() + timeout_s
    buf = _rx_buf(ser)
    while time.time() < end:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)
        while len(buf) >= 5:
            if buf[0] != STX:
                buf.pop(0)
                continue
            length = buf[1]
            if len(buf) < 2 + length + 2:
                break
            cmd = buf[2]
            payload = bytes(buf[3:2 + length])
            crc_rx = struct.unpack('>H', buf[2 + length:4 + length])[0]
            crc_calc = crc16_ccitt(bytes(buf[1:2 + length]))
            del buf[:4 + length]
            if crc_rx == crc_calc:
                return (cmd, payload)
            # Bad CRC — consumed those bytes, try next STX.
    return None


def set_param(ser, param_id: int, value: int):
    payload = struct.pack('<HI', param_id, value & 0xFFFFFFFF)
    ser.write(frame(GSP_CMD_SET_PARAM, payload))


def grab_snapshot(ser, timeout_s: float = 2.0):
    """Wait for the next telemetry snapshot frame from the broadcast stream."""
    end = time.time() + timeout_s
    while time.time() < end:
        r = read_one_frame(ser, timeout_s=min(0.5, end - time.time()))
        if r is None:
            continue
        cmd, payload = r
        if cmd == 0x80 and len(payload) >= 66:
            return payload
    return None


def decode_counters(payload: bytes):
    """Pull eRpm, capture counters, and V5 shadow raw counters out of a snapshot.

    V5 shadow semantics:
      pR_acc: rising-sector  samples where comp == 0 (POST state for rising)
      pR_rej: rising-sector  samples where comp == 1 (PRE  state for rising)
      pF_acc: falling-sector samples where comp == 1 (POST state for falling)
      pF_rej: falling-sector samples where comp == 0 (PRE  state for falling)

    pF_rej is the key counter for cF lockout diagnosis — it counts
    falling-sector PRE samples (comp==0) which the legacy V4 path SHOULD
    be turning into cF captures but isn't. If pF_rej climbs while cF stays
    pinned, the sample is being seen but mis-routed."""
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
    state = payload[2]
    return {
        'state': state, 'eRpm': eRpm,
        'cR': capRise, 'cF': capFall,
        'pR_acc': pR_acc, 'pR_rej': pR_rej,
        'pF_acc': pF_acc, 'pF_rej': pF_rej,
        'pR_pct': pR_pct, 'pF_pct': pF_pct,
    }


def sweep(port: str, baud: int, settle_s: float, step: int):
    ser = serial.Serial(port, baudrate=baud, timeout=0.05)
    # Kick the firmware's telemetry broadcast on — mirrors gsp_ak_test.py mon.
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.write(frame(GSP_CMD_TELEM_START))
    ser.flush()
    time.sleep(0.1)
    print(f"Opened {port} @ {baud}. Sweeping PG1TRIGA in steps of {step}.")
    print(f"LOOPTIME_TCY = {LOOPTIME_TCY}, so per-cycle range = [0, {LOOPTIME_TCY - 1}].")
    print("Hold pot steady. Each row = one TRIGA position, deltas over",
          f"{settle_s:.1f}s window.\n")
    # dcR/dcF: legacy V4 capture counters (isRising-routed).
    # dpRa/dpRr/dpFa/dpFr: V5 shadow raw counters (v5_ptgExpectedComp-routed,
    #   the polarity flag is volatile and known-good there).
    #
    # If dpFr (falling-sector comp==0 = PRE samples) is large but dcF=0,
    # the falling captures ARE on the wire — the legacy V4 path is just
    # mis-routing them through stuck-true isRising.
    print(f"{'CAHALF':>6} {'TRIGA':>6} {'eRpm':>7} "
          f"{'dcR':>6} {'dcF':>6}  "
          f"{'dpRa':>5} {'dpRr':>5} {'dpFa':>5} {'dpFr':>5}")

    # Sweep both cycles. Position 0..LOOPTIME_TCY-1 stepped by `step`.
    positions = []
    for cahalf in (0, 1):
        v = 0
        while v < LOOPTIME_TCY:
            positions.append((cahalf, v))
            v += step
        # also tag the very end of each cycle
        positions.append((cahalf, LOOPTIME_TCY - 1))

    results = []
    for cahalf, triga in positions:
        encoded = (triga & 0x7FFF) | (0x8000 if cahalf else 0)
        set_param(ser, V4_PARAM_TRIGA_POS, encoded)

        # Brief settling window after SET_PARAM lands.
        time.sleep(0.1)
        # Take the FIRST snapshot to anchor the counters at this position.
        snap0 = grab_snapshot(ser)
        if snap0 is None:
            print(f"{cahalf:>6} {triga:>6}   (no snap)")
            continue
        d0 = decode_counters(snap0)
        t0 = time.time()

        # Wait the settle window.
        while time.time() - t0 < settle_s:
            time.sleep(0.05)

        snap1 = grab_snapshot(ser)
        if snap1 is None:
            print(f"{cahalf:>6} {triga:>6}   (no snap)")
            continue
        d1 = decode_counters(snap1)

        dcR = max(0, d1['cR'] - d0['cR'])
        dcF = max(0, d1['cF'] - d0['cF'])
        dpRa = max(0, d1['pR_acc'] - d0['pR_acc'])
        dpRr = max(0, d1['pR_rej'] - d0['pR_rej'])
        dpFa = max(0, d1['pF_acc'] - d0['pF_acc'])
        dpFr = max(0, d1['pF_rej'] - d0['pF_rej'])
        ratio = (dcF / dcR) if dcR > 0 else 0.0

        results.append({
            'cahalf': cahalf, 'triga': triga, 'eRpm': d1['eRpm'],
            'dcR': dcR, 'dcF': dcF, 'ratio': ratio,
            'dpRa': dpRa, 'dpRr': dpRr, 'dpFa': dpFa, 'dpFr': dpFr,
            'pR': d1['pR_pct'], 'pF': d1['pF_pct'],
        })
        print(f"{cahalf:>6} {triga:>6} {d1['eRpm']:>7} "
              f"{dcR:>6} {dcF:>6}  "
              f"{dpRa:>5} {dpRr:>5} {dpFa:>5} {dpFr:>5}")

    # Summary 1: top 5 positions by cF/cR ratio (legacy V4 path success).
    print("\n=== Top 5 positions by cF/cR ratio (target ~1.0) ===")
    scored = [(min(r['ratio'], 1.0), r) for r in results if r['dcR'] > 100]
    scored.sort(key=lambda x: -x[0])
    for score, r in scored[:5]:
        print(f"  CAHALF={r['cahalf']} TRIGA={r['triga']:>6}: "
              f"dcR={r['dcR']:>6} dcF={r['dcF']:>6} "
              f"ratio={r['ratio']:.2f} pR={r['pR']}% pF={r['pF']}% eRpm={r['eRpm']}")

    # Summary 2: diagnostic — is the falling-sector PRE-comp sample (dpFr)
    # being seen at the silicon level, regardless of the legacy V4 routing?
    print("\n=== Falling-sector wire-level visibility (dpFr — V5 shadow) ===")
    print("If dpFr is large but dcF is small, the falling samples ARE on the")
    print("wire — the legacy V4 path is mis-routing them via stuck-true isRising.")
    by_dpFr = sorted([r for r in results if r['dcR'] > 100],
                     key=lambda r: -r['dpFr'])
    for r in by_dpFr[:5]:
        print(f"  CAHALF={r['cahalf']} TRIGA={r['triga']:>6}: "
              f"dpFr={r['dpFr']:>5} dcF={r['dcF']:>5}  "
              f"(rising: dpRa={r['dpRa']:>5} dpRr={r['dpRr']:>5}; "
              f"falling: dpFa={r['dpFa']:>5} dpFr={r['dpFr']:>5})")

    # Restore the committed baseline before exit (CAHALF=0, TRIGA=PER-1).
    set_param(ser, V4_PARAM_TRIGA_POS, (LOOPTIME_TCY - 1) & 0x7FFF)
    # Stop the telemetry broadcast we started.
    try:
        ser.write(frame(GSP_CMD_TELEM_STOP))
        ser.flush()
    except Exception:
        pass
    ser.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('port', nargs='?', default='/dev/ttyACM1')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--settle', type=float, default=0.8,
                    help='Seconds between snapshots at each position (default 0.8)')
    ap.add_argument('--step', type=int, default=800,
                    help='TRIGA step size per cycle (default 800, ~17 positions/cycle)')
    args = ap.parse_args()
    sweep(args.port, args.baud, args.settle, args.step)


if __name__ == '__main__':
    main()
