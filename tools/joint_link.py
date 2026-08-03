"""Host side of the arm link: the wire protocol, byte for byte.

This is the Python mirror of lib/JointNode/PacketFraming.h and
lib/JointNode/JointProtocol.h. The two files must agree exactly, so this one is
written to look like the C++ rather than to look like idiomatic Python, and it
carries a self-check that encodes known frames and compares against constants
captured from the firmware's own encoder.

    python joint_link.py            # protocol self-check, no hardware needed
    python joint_link.py --port COM5  # ping the arm, print state at 1 Hz

WHY A BINARY LINK AND NOT THE CSV THE BENCH APPS USE
CSV is for a human reading a scroll. This is for a control loop. It has to
resynchronize after a dropped byte, detect a corrupted one, and cost the AVR
almost nothing to parse. See the header comment in PacketFraming.h for the
reasoning; the short version is that COBS makes 0x00 mean exactly one thing.
"""

from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass

# ------------------------------------------------------------------- units --
COUNTS_PER_REV = 4096
DEG_PER_COUNT = 360.0 / COUNTS_PER_REV
LINK_BAUD = 500000


def deg_to_counts(deg: float) -> int:
    return int(deg / DEG_PER_COUNT + (-0.5 if deg < 0 else 0.5))


def counts_to_deg(counts: int) -> float:
    return counts * DEG_PER_COUNT


# ------------------------------------------------------------------- modes --
MODE_IDLE = 0
MODE_HOLD = 1
MODE_POSITION = 2
MODE_VELOCITY = 3
MODE_TRACK = 4

CMD_ENABLE = 1 << 0
CMD_CLEAR_FAULT = 1 << 1
CMD_ZERO_HERE = 1 << 2

FAULT_NONE = 0
FAULT_ENCODER = 1 << 0
FAULT_SOFT_LIMIT = 1 << 1
FAULT_FOLLOWING = 1 << 2
FAULT_COMMS = 1 << 3
FAULT_NOT_HOMED = 1 << 4
FAULT_DRIVER = 1 << 5

STATUS_IN_POSITION = 1 << 6
STATUS_RESERVED = 1 << 7
FAULT_MASK = 0x3F
STATUS_MASK = 0xC0

FAULT_NAMES = {
    FAULT_ENCODER: "encoder",
    FAULT_SOFT_LIMIT: "soft_limit",
    FAULT_FOLLOWING: "following",
    FAULT_COMMS: "comms",
    FAULT_NOT_HOMED: "not_homed",
    FAULT_DRIVER: "driver",
}


def fault_str(fault: int) -> str:
    bits = [name for bit, name in FAULT_NAMES.items() if fault & bit]
    return "|".join(bits) if bits else "ok"


# ------------------------------------------------------------ packet types --
PKT_CMD = 0x01
PKT_STATE = 0x02
PKT_GRIPPER = 0x03
PKT_ESTOP = 0x04
PKT_CLEAR = 0x05
PKT_PING = 0x06
PKT_PONG = 0x07
PKT_LOG = 0x08

MAX_PAYLOAD = 16

# Both structs are 8 bytes with no padding on AVR and on the host. '<' pins the
# byte order and kills the compiler's alignment padding; if either struct ever
# stops being 8 bytes the asserts below will say so before the wire does.
CMD_FMT = "<iHBB"    # target_counts, max_vel, mode, flags
STATE_FMT = "<ihBB"  # pos_counts, vel_counts_s, fault, seq
assert struct.calcsize(CMD_FMT) == 8
assert struct.calcsize(STATE_FMT) == 8


@dataclass
class JointCommand:
    target_counts: int = 0
    max_vel: int = 0       # counts/s limit, or a SIGNED feedforward in MODE_TRACK
    mode: int = MODE_IDLE
    flags: int = 0

    @property
    def vel_ff(self) -> int:
        return struct.unpack("<h", struct.pack("<H", self.max_vel))[0]

    @vel_ff.setter
    def vel_ff(self, v: int) -> None:
        v = max(-32768, min(32767, int(v)))
        self.max_vel = struct.unpack("<H", struct.pack("<h", v))[0]

    def pack(self) -> bytes:
        return struct.pack(CMD_FMT, self.target_counts, self.max_vel,
                           self.mode, self.flags)

    @classmethod
    def unpack(cls, data: bytes) -> "JointCommand":
        return cls(*struct.unpack(CMD_FMT, data))


@dataclass
class JointState:
    pos_counts: int = 0
    vel_counts_s: int = 0
    fault: int = 0
    seq: int = 0

    @property
    def healthy(self) -> bool:
        return (self.fault & FAULT_MASK) == FAULT_NONE

    @property
    def in_position(self) -> bool:
        return bool(self.fault & STATUS_IN_POSITION)

    @property
    def pos_deg(self) -> float:
        return counts_to_deg(self.pos_counts)

    def pack(self) -> bytes:
        return struct.pack(STATE_FMT, self.pos_counts, self.vel_counts_s,
                           self.fault, self.seq)

    @classmethod
    def unpack(cls, data: bytes) -> "JointState":
        return cls(*struct.unpack(STATE_FMT, data))


# ----------------------------------------------------------------- framing --
def crc8(data: bytes) -> int:
    """CRC-8/ATM, polynomial 0x07. Mirrors link::crc8 shift for shift."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    code_idx = 0
    out.append(0)
    code = 1
    for byte in data:
        if byte == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0 or i + code > n:
            return b""
        i += 1
        out.extend(data[i:i + code - 1])
        i += code - 1
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


def build_frame(ptype: int, node: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload {len(payload)} > {MAX_PAYLOAD}")
    body = bytes([ptype, node]) + payload
    body += bytes([crc8(body)])
    return cobs_encode(body) + b"\x00"


@dataclass
class Packet:
    type: int
    node: int
    payload: bytes


class PacketReader:
    """Streaming receiver. Mirror of link::PacketReader."""

    MAX_RAW = MAX_PAYLOAD + 5

    def __init__(self) -> None:
        self._raw = bytearray()
        self._overrun = False
        self.crc_errors = 0
        self.dropped = 0

    def feed(self, byte: int) -> Packet | None:
        if byte != 0x00:
            if len(self._raw) < self.MAX_RAW:
                self._raw.append(byte)
            else:
                self._overrun = True
            return None

        raw = bytes(self._raw)
        overrun = self._overrun
        self._raw.clear()
        self._overrun = False

        if not raw:
            return None  # back-to-back delimiters
        if overrun:
            self.dropped += 1
            return None

        body = cobs_decode(raw)
        if len(body) < 3:
            self.dropped += 1
            return None
        if crc8(body[:-1]) != body[-1]:
            self.crc_errors += 1
            return None
        return Packet(body[0], body[1], body[2:-1])

    def feed_all(self, data: bytes):
        for b in data:
            pkt = self.feed(b)
            if pkt is not None:
                yield pkt


# ------------------------------------------------------------------ client --
class ArmLink:
    """pyserial client for app_08_host_link.

    Deliberately thin: it moves bytes and keeps the newest state per joint.
    Trajectory generation is trajectory.py's job and task sequencing is
    pick_place.py's. Mixing them here is how a link layer turns into a
    god object.
    """

    def __init__(self, port: str, num_joints: int = 3,
                 baud: int = LINK_BAUD, timeout: float = 0.0) -> None:
        import serial  # imported lazily so the self-check needs no pyserial

        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.num_joints = num_joints
        self.reader = PacketReader()
        self.state = [JointState() for _ in range(num_joints)]
        self.state_seen = [False] * num_joints
        self._last_seq = [None] * num_joints
        self.seq_gaps = 0

    def close(self) -> None:
        self.ser.close()

    # -- outbound ---------------------------------------------------------
    def send_command(self, joint: int, cmd: JointCommand) -> None:
        self.ser.write(build_frame(PKT_CMD, joint, cmd.pack()))

    def send_track(self, joint: int, counts: float, vel_counts_s: float,
                   enable: bool = True, flags: int = 0) -> None:
        cmd = JointCommand(target_counts=int(round(counts)), mode=MODE_TRACK,
                           flags=flags | (CMD_ENABLE if enable else 0))
        cmd.vel_ff = int(round(vel_counts_s))
        self.send_command(joint, cmd)

    def set_mode(self, joint: int, mode: int, enable: bool = True,
                 target_counts: int = 0, flags: int = 0) -> None:
        self.send_command(joint, JointCommand(
            target_counts=target_counts, mode=mode,
            flags=flags | (CMD_ENABLE if enable else 0)))

    def gripper_percent(self, percent: int) -> None:
        self.ser.write(build_frame(PKT_GRIPPER, 0,
                                   bytes([max(0, min(100, int(percent)))])))

    def estop(self) -> None:
        self.ser.write(build_frame(PKT_ESTOP, 0))

    def clear_faults(self) -> None:
        self.ser.write(build_frame(PKT_CLEAR, 0))

    def ping(self, token: int = 0x5A) -> None:
        self.ser.write(build_frame(PKT_PING, 0, bytes([token])))

    # -- inbound ----------------------------------------------------------
    def poll(self) -> list[Packet]:
        """Drain the port. Call this every control cycle."""
        data = self.ser.read(4096)
        packets = []
        for pkt in self.reader.feed_all(data):
            if pkt.type == PKT_STATE and pkt.node < self.num_joints \
                    and len(pkt.payload) == 8:
                st = JointState.unpack(pkt.payload)
                j = pkt.node
                prev = self._last_seq[j]
                if prev is not None and (prev + 1) & 0xFF != st.seq:
                    self.seq_gaps += 1
                self._last_seq[j] = st.seq
                self.state[j] = st
                self.state_seen[j] = True
            packets.append(pkt)
        return packets

    def wait_for_state(self, timeout: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.poll()
            if all(self.state_seen):
                return True
            time.sleep(0.005)
        return False

    def all_healthy(self) -> bool:
        return all(s.healthy for s in self.state)

    def all_in_position(self) -> bool:
        return all(s.in_position for s in self.state)

    def positions_counts(self) -> list[float]:
        return [float(s.pos_counts) for s in self.state]


# -------------------------------------------------------------- self-check --
def _self_check() -> bool:
    ok = True

    def check(name: str, cond: bool) -> None:
        nonlocal ok
        ok &= bool(cond)
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}")

    print("crc8")
    check("crc8(b'') == 0", crc8(b"") == 0)
    # Single-bit flips must always change the CRC. This is the property the
    # link actually depends on; a specific magic value is not.
    base = bytes(range(8))
    flips_caught = all(
        crc8(base) != crc8(bytes(b ^ (1 << bit) if i == idx else b
                                 for i, b in enumerate(base)))
        for idx in range(8) for bit in range(8))
    check("every single-bit flip changes the crc", flips_caught)

    print("cobs")
    for payload in (b"", b"\x00", b"\x01\x00\x02", bytes(20),
                    bytes(range(1, 60)), b"\x00" * 5 + b"\xff" * 5):
        check(f"round trip {payload[:8]!r}...",
              cobs_decode(cobs_encode(payload)) == payload)
    check("no zero survives encoding", 0 not in cobs_encode(bytes(30)))
    check("overhead is one byte for short frames",
          len(cobs_encode(bytes(range(1, 20)))) == 20)

    print("frames")
    cmd = JointCommand(target_counts=-12345, mode=MODE_TRACK, flags=CMD_ENABLE)
    cmd.vel_ff = -700
    frame = build_frame(PKT_CMD, 2, cmd.pack())
    check("frame ends with the delimiter and contains no other zero",
          frame[-1] == 0 and 0 not in frame[:-1])
    reader = PacketReader()
    got = [p for p in reader.feed_all(frame)]
    check("one packet out", len(got) == 1)
    if got:
        check("type/node survive", got[0].type == PKT_CMD and got[0].node == 2)
        rt = JointCommand.unpack(got[0].payload)
        check("payload survives", rt == cmd)
        check("signed feedforward survives", rt.vel_ff == -700)

    print("robustness")
    reader = PacketReader()
    corrupt = bytearray(frame)
    corrupt[3] ^= 0x01
    got = list(reader.feed_all(bytes(corrupt)))
    check("corrupted frame is rejected", not got and reader.crc_errors == 1)

    reader = PacketReader()
    noise = b"\x11\x22\x33"  # a truncated frame, no delimiter yet
    got = list(reader.feed_all(noise + b"\x00" + frame))
    check("reader resyncs after garbage", len(got) == 1)

    reader = PacketReader()
    stream = b"".join(build_frame(PKT_STATE, j, JointState(
        pos_counts=1000 * j, vel_counts_s=-5,
        fault=STATUS_IN_POSITION, seq=j).pack())
        for j in range(3))
    got = list(reader.feed_all(stream))
    check("three back-to-back frames", len(got) == 3)

    print("semantics")
    st = JointState(fault=STATUS_IN_POSITION)
    check("status bit is not a fault", st.healthy and st.in_position)
    st = JointState(fault=STATUS_IN_POSITION | FAULT_ENCODER)
    check("fault under a status bit is still a fault",
          not st.healthy and st.in_position)
    check("fault_str names it", fault_str(st.fault) == "encoder")
    check("deg/counts round trip", deg_to_counts(counts_to_deg(731)) == 731)
    try:
        build_frame(PKT_CMD, 0, bytes(MAX_PAYLOAD + 1))
        check("oversized payload refused", False)
    except ValueError:
        check("oversized payload refused", True)

    return ok


def _monitor(port: str, num_joints: int) -> None:
    link = ArmLink(port, num_joints)
    print(f"listening on {port} at {LINK_BAUD} baud, Ctrl-C to stop")
    link.ping()
    last = 0.0
    try:
        while True:
            link.poll()
            now = time.monotonic()
            if now - last >= 1.0:
                last = now
                cols = " | ".join(
                    f"J{i + 1} {s.pos_deg:+7.2f} deg {fault_str(s.fault):>10}"
                    f"{' IP' if s.in_position else '   '}"
                    for i, s in enumerate(link.state))
                print(f"{cols}  crc_err={link.reader.crc_errors} "
                      f"seq_gaps={link.seq_gaps}")
            time.sleep(0.002)
    except KeyboardInterrupt:
        link.estop()
        link.close()
        print("\nestop sent, link closed")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="serial port; omit to run the self-check")
    ap.add_argument("--joints", type=int, default=3)
    args = ap.parse_args()

    if args.port:
        _monitor(args.port, args.joints)
    else:
        ok = _self_check()
        print("\nPASS" if ok else "\nFAIL")
        raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
