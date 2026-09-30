#!/usr/bin/env python3
"""Bluetooth headset microphone bridge: 16 kHz PCM for the voice node.

Works with any paired headset that has the hands-free (HFP) profile: the one
connected now, else each paired one in turn (the last one that worked
first). PipeWire 1.0.5 answers AT+NREC with an error when no modem is present,
so headsets dropped its link before any samples existed; this process acts as
the hands-free gateway itself, replies OK, and reads the SCO microphone.

Codecs: mSBC (wideband) when the headset negotiates it, else CVSD (8 kHz,
upsampled to 16 kHz). /tmp/hospital_hfp_mic.up exists only while headset
audio is flowing; without it the voice node uses the system microphone
(USB / wired), so any microphone works.
"""

import array
import ctypes
import os
import socket
import struct
import subprocess
import threading
import time

PCM_SOCK = "/tmp/hospital_hfp_mic.sock"
LINK_FLAG = "/tmp/hospital_hfp_mic.up"
LAST_GOOD = os.path.expanduser("~/.hospital_headset")
HANDSFREE_UUID = "0000111e"
SOL_BLUETOOTH = 274
BT_VOICE = 11
BT_VOICE_TRANSPARENT = 0x0003   # mSBC: raw frames, decoded here
BT_VOICE_CVSD_16BIT = 0x0060    # CVSD: the adapter delivers 16-bit 8 kHz PCM

# Indicator order shared by AT+CIND=? and AT+CIND?
CIND_DEF = (
    '("service",(0,1)),("call",(0,1)),("callsetup",(0,3)),'
    '("callheld",(0,2)),("signal",(0,5)),("roam",(0,1)),("battchg",(0,5))'
)
CIND_VAL = "1,0,0,0,5,0,5"


def log(msg):
    print(msg, flush=True)


class AudioOut:
    def __init__(self):
        self._clients = []
        self._lock = threading.Lock()
        self._rms_sq = 0
        self._rms_n = 0
        self._last_report = time.monotonic()

    def add(self, conn):
        with self._lock:
            self._clients.append(conn)

    def write(self, pcm):
        dead = []
        with self._lock:
            clients = list(self._clients)
        for conn in clients:
            try:
                conn.sendall(pcm)
            except OSError:
                dead.append(conn)
        if dead:
            with self._lock:
                for conn in dead:
                    if conn in self._clients:
                        self._clients.remove(conn)
                    try:
                        conn.close()
                    except OSError:
                        pass
        samples = array.array("h")
        samples.frombytes(pcm[: len(pcm) - (len(pcm) % 2)])
        if samples:
            self._rms_sq += sum(s * s for s in samples)
            self._rms_n += len(samples)
        now = time.monotonic()
        if now - self._last_report >= 2.0 and self._rms_n:
            rms = (self._rms_sq / self._rms_n) ** 0.5
            log(f"Headset mic rms={rms:.0f}")
            self._rms_sq = 0
            self._rms_n = 0
            self._last_report = now


AUDIO = AudioOut()


def serve_pcm():
    if os.path.exists(PCM_SOCK):
        os.unlink(PCM_SOCK)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(PCM_SOCK)
    os.chmod(PCM_SOCK, 0o666)
    server.listen(1)
    log(f"PCM socket {PCM_SOCK}")
    while True:
        conn, _ = server.accept()
        log("Voice node attached to the headset mic")
        AUDIO.add(conn)


def upsample_8k_to_16k(data):
    samples = array.array("h")
    samples.frombytes(data[: len(data) - (len(data) % 2)])
    out = array.array("h")
    for sample in samples:
        out.append(sample)
        out.append(sample)
    return out.tobytes()


def _msbc_decoder():
    lib = ctypes.CDLL("libsbc.so.1")
    lib.sbc_init_msbc.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    lib.sbc_init_msbc.restype = ctypes.c_int
    lib.sbc_decode.argtypes = [
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_size_t,
        ctypes.c_void_p,
        ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_size_t),
    ]
    lib.sbc_decode.restype = ctypes.c_ssize_t
    ctx = ctypes.create_string_buffer(1024)
    if lib.sbc_init_msbc(ctx, 0) != 0:
        raise RuntimeError("mSBC decoder failed to start")
    out = ctypes.create_string_buffer(512)

    def decode(frame):
        written = ctypes.c_size_t(0)
        consumed = lib.sbc_decode(
            ctx, frame, len(frame), out, len(out), ctypes.byref(written)
        )
        if consumed <= 0 or written.value == 0:
            return b""
        return out.raw[: written.value]

    return decode


LINK_UP = False


def set_link_flag(up):
    """Create / remove the link flag once per change, atomically (write a
    temp file, then rename): the voice node must never read a half-written
    flag. Rewriting it on every packet made it read empty now and then, so
    the voice node dropped the headset several times a second."""
    global LINK_UP
    if up == LINK_UP:
        return
    LINK_UP = up
    try:
        if up:
            tmp = LINK_FLAG + ".tmp"
            with open(tmp, "w") as f:
                f.write(str(os.getpid()))
            os.replace(tmp, LINK_FLAG)
            log("Headset audio flowing")
        elif os.path.exists(LINK_FLAG):
            os.unlink(LINK_FLAG)
            log("Headset audio stopped")
    except OSError:
        pass


def read_sco(sco, cvsd=False):
    log(f"SCO microphone open ({'CVSD' if cvsd else 'mSBC'})")
    if cvsd:
        read_sco_cvsd(sco)
        return
    global CURRENT_RFCOMM
    decode = _msbc_decoder()
    logged = False
    # Playing the microphone back into the earbuds makes them cancel
    # the wearer's voice. Send silence on the speaker side instead.
    silence = bytes.fromhex(
        "ad0000c500000000776db6dddb6db776db6dddb6db77"
        "6db6dddb6db776db6dddb6db776db6dddb6db776db6d"
        "ddb6db776db6dddb6db776db6c"
    )
    h2 = (0x08, 0x38, 0xC8, 0xF8)
    h2_i = 0
    raw_energy = 0
    raw_count = 0
    last_raw = time.monotonic()
    try:
        while True:
            data = sco.recv(512)
            if not data:
                break
            packet = bytes((0x01, h2[h2_i])) + silence
            if len(packet) < 60:
                packet += bytes(60 - len(packet))
            h2_i = (h2_i + 1) % 4
            try:
                sco.send(packet[:60])
            except OSError:
                pass
            if not logged:
                log(f"SCO first packet {len(data)} bytes {data[:16].hex()}")
                logged = True
            if len(data) > 2:
                raw_energy += sum(data[2:])
                raw_count += len(data) - 2
            now = time.monotonic()
            if now - last_raw >= 2.0 and raw_count:
                log(f"SCO payload avg={raw_energy / raw_count:.1f}")
                raw_energy = 0
                raw_count = 0
                last_raw = now
            # H2 header (2 bytes) + mSBC frame starting at the 0xAD sync.
            if len(data) >= 59 and data[0] == 0x01 and data[2] == 0xAD:
                pcm = decode(data[2:59])
            else:
                pcm = b""
            if pcm:
                set_link_flag(True)
                AUDIO.write(pcm)
    except OSError as exc:
        log(f"SCO microphone closed: {exc}")
    finally:
        close_link(sco)


def read_sco_cvsd(sco):
    """CVSD: 16-bit 8 kHz PCM both ways; answer each packet with silence."""
    try:
        while True:
            data = sco.recv(512)
            if not data:
                break
            try:
                sco.send(bytes(len(data)))
            except OSError:
                pass
            set_link_flag(True)
            AUDIO.write(upsample_8k_to_16k(data))
    except OSError as exc:
        log(f"SCO microphone closed: {exc}")
    finally:
        close_link(sco)


def close_link(sco):
    set_link_flag(False)
    try:
        sco.close()
    except OSError:
        pass
    link = CURRENT_RFCOMM
    if link is not None:
        try:
            link.close()
        except OSError:
            pass


class _ScoAddr(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("family", ctypes.c_ushort), ("bdaddr", ctypes.c_uint8 * 6)]


def _bdaddr(text):
    return [int(part, 16) for part in reversed(text.split(":"))]


def connect_sco(adapter, remote, cvsd=False):
    import select
    sco = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_SEQPACKET, socket.BTPROTO_SCO)
    libc = ctypes.CDLL(None, use_errno=True)
    try:
        local = _ScoAddr()
        local.family = socket.AF_BLUETOOTH
        local.bdaddr[:] = _bdaddr(adapter)
        if libc.bind(sco.fileno(), ctypes.byref(local), ctypes.sizeof(local)) != 0:
            err = ctypes.get_errno()
            raise OSError(err, os.strerror(err))
        voice = BT_VOICE_CVSD_16BIT if cvsd else BT_VOICE_TRANSPARENT
        sco.setsockopt(SOL_BLUETOOTH, BT_VOICE, struct.pack("<H", voice))
        peer = _ScoAddr()
        peer.family = socket.AF_BLUETOOTH
        peer.bdaddr[:] = _bdaddr(remote)
        sco.setblocking(False)
        rc = libc.connect(sco.fileno(), ctypes.byref(peer), ctypes.sizeof(peer))
        if rc != 0 and ctypes.get_errno() not in (0, 115):
            err = ctypes.get_errno()
            raise OSError(err, os.strerror(err))
        _, writable, _ = select.select([], [sco], [], 3.0)
        if not writable:
            raise OSError(110, "SCO connect timed out")
        err = sco.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
        if err:
            raise OSError(err, os.strerror(err))
        sco.setblocking(True)
    except OSError as exc:
        log(f"SCO connect failed: {exc}")
        sco.close()
        return False
    threading.Thread(target=read_sco, args=(sco, cvsd), daemon=True).start()
    return True


def send_lines(rfcomm, *lines):
    payload = "".join(f"\r\n{line}\r\n" for line in lines).encode()
    rfcomm.sendall(payload)


CURRENT_RFCOMM = None


def handle_rfcomm(rfcomm, adapter, remote):
    global CURRENT_RFCOMM
    CURRENT_RFCOMM = rfcomm
    buf = b""
    slc_ready = False
    sco_started = False
    # Headset features (AT+BRSF): bit 7 = codec negotiation. Codecs (AT+BAC):
    # 2 = mSBC. Without both, the call audio is CVSD.
    codec_negotiation = False
    msbc = False
    log(f"RFCOMM open for {remote}")
    try:
        while True:
            chunk = rfcomm.recv(256)
            if not chunk:
                break
            buf += chunk
            while b"\r" in buf:
                raw, buf = buf.split(b"\r", 1)
                if buf.startswith(b"\n"):
                    buf = buf[1:]
                cmd = raw.decode("utf-8", "replace").strip()
                if not cmd:
                    continue
                log(f"AT {cmd}")
                upper = cmd.upper()
                if upper.startswith("AT+BRSF"):
                    try:
                        codec_negotiation = bool(int(cmd.split("=", 1)[1]) & (1 << 7))
                    except (IndexError, ValueError):
                        codec_negotiation = False
                    # Our (gateway) features; bit 9 is codec negotiation.
                    send_lines(rfcomm, "+BRSF: 1023", "OK")
                elif upper.startswith("AT+BAC"):
                    msbc = "2" in cmd.split("=", 1)[-1].split(",")
                    send_lines(rfcomm, "OK")
                elif upper.startswith("AT+CIND=?"):
                    send_lines(rfcomm, f"+CIND: {CIND_DEF}", "OK")
                elif upper.startswith("AT+CIND?"):
                    send_lines(rfcomm, f"+CIND: {CIND_VAL}", "OK")
                elif upper.startswith("AT+CHLD=?"):
                    send_lines(rfcomm, "+CHLD: (0,1,2)", "OK")
                elif upper.startswith("AT+CMER"):
                    send_lines(rfcomm, "OK")
                    slc_ready = True
                    if not (codec_negotiation and msbc) and not sco_started:
                        # CVSD headset: no codec step follows. Signal an
                        # active call so it opens its mic, then open the audio.
                        time.sleep(0.2)
                        send_lines(rfcomm, "+VGM:15", "+CIEV: 3,2", "+CIEV: 2,1",
                                   "+CIEV: 3,0")
                        time.sleep(0.3)
                        sco_started = connect_sco(adapter, remote, cvsd=True)
                elif upper.startswith("AT+BCC"):
                    send_lines(rfcomm, "OK")
                    if slc_ready and not sco_started:
                        send_lines(rfcomm, "+BCS:2")
                elif upper.startswith("AT+BCS"):
                    send_lines(rfcomm, "OK")
                    if not sco_started:
                        sco_started = connect_sco(adapter, remote)
                elif upper.startswith("AT+NREC"):
                    send_lines(rfcomm, "OK")
                    if slc_ready and not sco_started and codec_negotiation and msbc:
                        time.sleep(0.2)
                        # Keep an active call so the headset leaves the mic open.
                        send_lines(
                            rfcomm,
                            "+VGM:15",
                            "+CIEV: 3,2",
                            "+CIEV: 2,1",
                            "+CIEV: 3,0",
                            "+BCS:2",
                        )
                else:
                    send_lines(rfcomm, "OK")
    except OSError as exc:
        log(f"RFCOMM closed: {exc}")
    finally:
        try:
            rfcomm.close()
        except OSError:
            pass
        set_link_flag(False)
        log("RFCOMM closed")


def adapter_address():
    out = subprocess.check_output(["bluetoothctl", "show"], text=True, timeout=5)
    for line in out.splitlines():
        if "Controller" in line:
            parts = line.split()
            if len(parts) >= 2:
                return parts[1]
    raise RuntimeError("Bluetooth adapter not found")


def _bt(*args, timeout=8):
    try:
        return subprocess.check_output(["bluetoothctl", *args], text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError):
        return ""


def _devices(kind):
    """Addresses from 'bluetoothctl devices <kind>' (Connected / Paired)."""
    return [line.split()[1] for line in _bt("devices", kind).splitlines()
            if line.startswith("Device ") and len(line.split()) >= 2]


def is_headset(addr):
    return HANDSFREE_UUID in _bt("info", addr).lower()


def headset_candidates():
    """Headsets to try, best first: connected now, last good, other paired."""
    try:
        with open(LAST_GOOD) as f:
            last = f.read().strip()
    except OSError:
        last = ""
    connected = [a for a in _devices("Connected") if is_headset(a)]
    paired = [a for a in _devices("Paired") if is_headset(a)]
    order = connected + ([last] if last in paired else []) + paired
    return list(dict.fromkeys(order))   # unique, keep order


def remember(addr):
    try:
        with open(LAST_GOOD, "w") as f:
            f.write(addr)
    except OSError:
        pass


def ensure_connected(addr):
    # Drop any existing A2DP session so the call link is the only one.
    subprocess.run(
        ["bluetoothctl", "disconnect", addr],
        check=False,
        timeout=10,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(0.8)


def hfp_channel(addr):
    """RFCOMM channel of the headset's hands-free service, or None."""
    try:
        out = subprocess.check_output(
            ["sdptool", "browse", addr], text=True, timeout=15
        )
    except (OSError, subprocess.SubprocessError):
        return None
    in_handsfree = False
    for line in out.splitlines():
        if "Hands-Free" in line or "Handsfree" in line:
            in_handsfree = True
            continue
        if in_handsfree and "Channel:" in line:
            return int(line.rsplit(":", 1)[-1].strip())
        if in_handsfree and line.startswith("Service Name:"):
            in_handsfree = False
    return None


def try_headset(adapter, remote):
    """Run one call link to this headset until it ends. True if it opened."""
    info = _bt("info", remote)
    name = info.split("Name: ", 1)[1].splitlines()[0] if "Name: " in info else "?"
    log(f"Trying headset {remote} ({name})")
    ensure_connected(remote)
    channel = hfp_channel(remote)
    if channel is None:
        log(f"{remote}: no hands-free service found (off, out of range or busy)")
        return False
    rfcomm = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    rfcomm.settimeout(8)
    log(f"Opening hands-free channel {channel}")
    try:
        rfcomm.connect((remote, channel))
    except OSError as exc:
        log(f"{remote}: {exc}")
        rfcomm.close()
        return False
    rfcomm.settimeout(None)
    remember(remote)
    handle_rfcomm(rfcomm, adapter, remote)
    return True


def main():
    set_link_flag(False)
    threading.Thread(target=serve_pcm, daemon=True).start()
    adapter = adapter_address()
    log(f"Headset mic bridge on {adapter}")
    quiet_since = 0.0
    while True:
        candidates = headset_candidates()
        if not candidates:
            if time.monotonic() - quiet_since > 30:
                log("No paired hands-free headset. Pair one (bluetoothctl: pair, trust, "
                    "connect) or use a USB microphone.")
                quiet_since = time.monotonic()
            time.sleep(3)
            continue
        for remote in candidates:
            try:
                if try_headset(adapter, remote):
                    break
            except Exception as exc:
                log(f"Hands-free link failed: {exc}")
        set_link_flag(False)
        time.sleep(2)


if __name__ == "__main__":
    main()
