import argparse
import ctypes
import struct
import sys
import time

import numpy as np
import soundcard as sc

# ================= CONFIG =================

HZ = 240
DURATION_SECONDS = 60
OUTPUT_FILE = "space_240hz.bin"

VK_SPACE = 0x20
DT_NS = 1_000_000_000 // HZ

# Audio trigger defaults.
AUDIO_SAMPLE_RATE = 48_000
AUDIO_BLOCK_SIZE = 256  # ~5.3 ms at 48 kHz
AUDIO_RMS_THRESHOLD = 0.01  # anything louder than ~ambient noise

# ================= CLI =================

parser = argparse.ArgumentParser(
    description="240Hz spacebar logger with audio-triggered start."
)
parser.add_argument("--output", default=OUTPUT_FILE, help="output .bin path")
parser.add_argument(
    "--duration", type=int, default=DURATION_SECONDS, help="seconds to record"
)
parser.add_argument(
    "--no-audio-trigger",
    action="store_true",
    help="skip audio wait (use 3-second countdown like before)",
)
parser.add_argument(
    "--audio-threshold",
    type=float,
    default=AUDIO_RMS_THRESHOLD,
    help="RMS threshold (0..1) that counts as 'music started'",
)
parser.add_argument(
    "--audio-device",
    default=None,
    help="name of the speaker to loopback. Defaults to the system default.",
)
args = parser.parse_args()

DURATION_SECONDS = args.duration
OUTPUT_FILE = args.output

# ================= WINDOWS SETUP =================

user32 = ctypes.windll.user32
winmm = ctypes.windll.winmm
kernel32 = ctypes.windll.kernel32

GetAsyncKeyState = user32.GetAsyncKeyState

winmm.timeBeginPeriod(1)

HIGH_PRIORITY_CLASS = 0x00000080
kernel32.SetPriorityClass(kernel32.GetCurrentProcess(), HIGH_PRIORITY_CLASS)

# ================= BIT PACKING =================

buffer = bytearray()
current_byte = 0
bit_index = 0


def write_bit(bit: int):
    global current_byte, bit_index

    if bit:
        current_byte |= 1 << bit_index

    bit_index += 1

    if bit_index == 8:
        buffer.append(current_byte)
        current_byte = 0
        bit_index = 0


def flush_bits():
    global current_byte, bit_index

    if bit_index > 0:
        buffer.append(current_byte)


# ================= AUDIO TRIGGER =================


def pick_loopback_mic():
    """Return a soundcard Microphone that captures the default speaker's output."""
    if args.audio_device:
        spk = sc.get_speaker(args.audio_device)
    else:
        spk = sc.default_speaker()
    # soundcard exposes every speaker as a loopback mic via include_loopback=True.
    return sc.get_microphone(id=str(spk.name), include_loopback=True), spk


def wait_for_audio():
    """Open a WASAPI loopback on the default speaker, block until RMS exceeds
    `--audio-threshold`. Returns the timestamp (perf_counter_ns) corresponding
    to the *start* of the first above-threshold block so t=0 of the bitstring
    lines up with the first audible sample, not the detection moment."""
    mic, spk = pick_loopback_mic()
    print(f"Listening on loopback of: {spk.name}")
    print(
        f"Waiting for audio (RMS > {args.audio_threshold}). "
        "Start the GD song now..."
    )
    block_duration_ns = int(1_000_000_000 * AUDIO_BLOCK_SIZE / AUDIO_SAMPLE_RATE)
    with mic.recorder(
        samplerate=AUDIO_SAMPLE_RATE, blocksize=AUDIO_BLOCK_SIZE
    ) as rec:
        while True:
            block_capture_end_ns = time.perf_counter_ns()
            data = rec.record(numframes=AUDIO_BLOCK_SIZE)
            # data shape: (frames, channels). Mix down to mono RMS.
            if data.size == 0:
                continue
            mono = data.mean(axis=1) if data.ndim > 1 else data
            rms = float(np.sqrt(np.mean(mono.astype(np.float64) ** 2)))
            if rms >= args.audio_threshold:
                # Align t=0 to the START of this block, not now.
                block_start_ns = block_capture_end_ns
                # (block_capture_end_ns was taken before record() returned;
                # in practice record() may have blocked for up to one block
                # duration, so the block's audible start is ~block_duration_ns
                # earlier than when record() returned. We use the pre-record
                # timestamp which is the best available lower bound.)
                _ = block_duration_ns  # informational; retained for clarity
                print(f"Audio detected (RMS={rms:.4f}). Starting capture...")
                return block_start_ns


# ================= LOGGER =================

total_samples = HZ * DURATION_SECONDS

print(f"Logging {HZ}Hz for {DURATION_SECONDS}s (missed ticks filled)")

if args.no_audio_trigger:
    print("Starting in 3 seconds...")
    time.sleep(1)
    print("Starting in 2 seconds...")
    time.sleep(1)
    print("Starting in 1 seconds...")
    time.sleep(1)
    print("Started")
    start_ns = time.perf_counter_ns()
else:
    try:
        start_ns = wait_for_audio()
    except KeyboardInterrupt:
        print("\nAborted before audio trigger.", file=sys.stderr)
        winmm.timeEndPeriod(1)
        sys.exit(1)

sample_index = 0
last_bit = 0

max_late_ns = 0
missed_events = 0

try:
    while sample_index < total_samples:
        target_ns = start_ns + sample_index * DT_NS

        while True:
            now_ns = time.perf_counter_ns()
            remaining_ns = target_ns - now_ns

            if remaining_ns <= 0:
                break

            if remaining_ns > 2_000_000:
                time.sleep(0.001)
            elif remaining_ns > 200_000:
                time.sleep(0)
            else:
                pass  # spin

        now_ns = time.perf_counter_ns()
        lateness_ns = now_ns - target_ns

        if lateness_ns > max_late_ns:
            max_late_ns = lateness_ns

        missed = int(lateness_ns // DT_NS)

        if missed > 0:
            missed_events += 1

            for _ in range(missed):
                if sample_index >= total_samples:
                    break
                write_bit(last_bit)
                sample_index += 1

        if sample_index >= total_samples:
            break

        pressed = bool(GetAsyncKeyState(VK_SPACE) & 0x8000)
        last_bit = 1 if pressed else 0

        write_bit(last_bit)
        sample_index += 1

finally:
    flush_bits()

    end_ns = time.perf_counter_ns()
    elapsed = (end_ns - start_ns) / 1e9

    unused_bits = (8 - (total_samples % 8)) % 8

    with open(OUTPUT_FILE, "wb") as f:
        f.write(b"SP240BIN")
        f.write(struct.pack("<IIIb", HZ, DURATION_SECONDS, total_samples, unused_bits))
        f.write(buffer)

    winmm.timeEndPeriod(1)

    print("\nDone.")
    print(f"Samples: {total_samples}")
    print(f"Bytes: {len(buffer)}")
    print(f"Elapsed: {elapsed:.6f}s")
    print(f"Max lateness: {max_late_ns / 1e6:.3f} ms")
    print(f"Missed events handled: {missed_events}")
    print(f"Output: {OUTPUT_FILE}")
