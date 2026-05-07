// 240Hz spacebar bitstring logger triggered by the GD player's x-coord.
//
// Mirrors `bitstring/get_bitstring.py` but, instead of waiting on a song-start
// audio cue, attaches to GeometryDash.exe (via HAPIH) and waits for the level
// to actually start moving the player: x sits at ~0 for a while, then jumps
// positive by a "normal" 240Hz step (vx*dt = ~1.44px at 1x speed, up to ~6px
// at 4x). That first positive frame is sample 0 of the bitstring, so the
// resulting .bin lines up exactly with the level's tick 0 - no manual
// tick-offset alignment needed downstream.
//
// Output format is identical to the Python script (`SP240BIN` magic + header
// + bit-packed payload), so any consumer that reads `space_240hz.bin` works
// unchanged.
//
// Build (MSVC, from repo root):
//   cl /std:c++17 /EHsc /O2 ^
//      bitstring\get_bitstring.cpp DashBot-3.0\HAPIH.cpp ^
//      /Fe:bitstring\get_bitstring.exe ^
//      /I DashBot-3.0
//
// Build (clang/g++):
//   g++ -std=c++17 -O2 \
//       bitstring/get_bitstring.cpp DashBot-3.0/HAPIH.cpp \
//       -IDashBot-3.0 -o bitstring/get_bitstring.exe -lwinmm

#include "HAPIH.h"
#include <windows.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#pragma comment(lib, "winmm.lib")

namespace {

// ---------------- defaults / constants ----------------

constexpr int    DEFAULT_HZ              = 240;
constexpr int    DEFAULT_DURATION_S      = 60;
constexpr int    VK_SPACE_KEY            = 0x20;
constexpr int    VK_ESCAPE_KEY           = 0x1B;
constexpr auto   SLEEP_GUARD             = std::chrono::microseconds(800);
constexpr auto   YIELD_THRESHOLD         = std::chrono::microseconds(200);

// "Normal" per-tick x delta band, in world units, at 240Hz.
// 1x speed in GD is ~5.77 vx and dt = 1/4 frame = 0.25 substep, so per-tick
// dx ~= 1.44. At 4x (1.6 player_speed) it tops out around ~5.0. Anything
// outside [0.5, 7.0] is either noise/jitter (too small) or a teleport/portal
// jump (too big) and we should keep waiting.
constexpr double DEFAULT_MIN_STEP        = 0.5;
constexpr double DEFAULT_MAX_STEP        = 7.0;

// Trigger upper bound on x. As soon as we see a finite x below this AND a
// dx in [min_step, max_step], we know the level is running and we're still
// near the start. We then back-compute how many ticks "x = FIRST_TICK_X" was
// based on the observed dx, and pad that many 0-bits in front so sample 0 of
// the output still aligns with tick 0.
//
// Default = 500px. Way bigger than any realistic startup latency between
// "level started moving" and "we caught the first tick", but way smaller
// than the typical level length, so post-restart catches still work.
constexpr double DEFAULT_X_TRIGGER_MAX   = 500.0;
constexpr double FIRST_TICK_X            = -1.0;

// Hard cap on how many leading 0-bits we'll synthesize from x/dx, just in
// case dx is unstable on the trigger frame. 5s worth at 240Hz = 1200 ticks.
constexpr int    MAX_PAD_TICKS           = 1200;

// Default GD pointer chain offsets (see DashBot-3.0/gd_position_logger.cpp).
// You can override with --x-addr / --y-addr if you've already resolved them
// (e.g. via Cheat Engine) since pointer chains can drift across GD versions.
constexpr std::size_t OFF_X = 0x67C;
constexpr std::size_t OFF_Y = 0x680;

volatile bool g_running = true;

BOOL WINAPI on_console_ctrl(DWORD ctrl_type) {
    switch (ctrl_type) {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_LOGOFF_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        g_running = false;
        return TRUE;
    default:
        return FALSE;
    }
}

void print_usage(const char* exe) {
    std::cerr
        << "Usage: " << exe << " <out.bin> [options]\n"
        << "\n"
        << "Options:\n"
        << "  --duration <seconds>   Recording length (default 60)\n"
        << "  --hz <int>             Sample rate in Hz (default 240)\n"
        << "  --x-addr <hex>         Direct x-coord address (skip pointer chain)\n"
        << "  --y-addr <hex>         Optional direct y-coord address (accepted for compatibility)\n"
        << "  --x-trigger-max <float> Max x at trigger frame (default 500). Trigger fires\n"
        << "                          on the first tick where x < this AND dx is in band;\n"
        << "                          (round((x-(-1))/dx)) leading 0-bits are prepended so\n"
        << "                          sample 0 aligns with tick 0 at x = -1.\n"
        << "  --min-step <float>     Min per-tick dx that arms trigger (default 0.5)\n"
        << "  --max-step <float>     Max per-tick dx that counts as a normal step (default 7.0)\n"
        << "\n"
        << "Example:\n"
        << "  " << exe << " pop.bin --duration 90 \\\n"
        << "    --x-addr 0x1C3545D688C --y-addr 0x1C328E5F3E0\n";
}

bool parse_u64(const std::string& text, uint64_t& out) {
    try {
        size_t consumed = 0;
        out = std::stoull(text, &consumed, 0);
        return consumed == text.size();
    } catch (...) {
        return false;
    }
}

bool parse_double(const std::string& text, double& out) {
    try {
        size_t consumed = 0;
        out = std::stod(text, &consumed);
        return consumed == text.size();
    } catch (...) {
        return false;
    }
}

bool parse_int(const std::string& text, int& out) {
    try {
        size_t consumed = 0;
        long v = std::stol(text, &consumed, 0);
        if (consumed != text.size()) return false;
        out = static_cast<int>(v);
        return true;
    } catch (...) {
        return false;
    }
}

// ---------------- bit packing (matches get_bitstring.py) ----------------

class BitBuffer {
public:
    void push(int bit) {
        if (bit) cur_ |= static_cast<uint8_t>(1u << bit_index_);
        ++bit_index_;
        if (bit_index_ == 8) {
            bytes_.push_back(cur_);
            cur_ = 0;
            bit_index_ = 0;
        }
    }
    void flush() {
        if (bit_index_ > 0) {
            bytes_.push_back(cur_);
            cur_ = 0;
            bit_index_ = 0;
        }
    }
    const std::vector<uint8_t>& bytes() const { return bytes_; }

private:
    std::vector<uint8_t> bytes_;
    uint8_t cur_ = 0;
    int     bit_index_ = 0;
};

// Read a float through either the direct address (--x-addr) or the GD
// pointer chain. Returns NaN on failure so the trigger logic can ignore it.
float read_coord(HackIH& gd, bool use_direct, uint64_t direct_addr, std::size_t chain_off) {
    float v = std::numeric_limits<float>::quiet_NaN();
    if (use_direct) {
        gd.ReadRaw(PointerIH(reinterpret_cast<void*>(direct_addr)), &v, sizeof(v));
    } else {
        v = gd.Read<float>({gd.BaseAddress, 0x3222D0, 0x164, 0x224, chain_off});
    }
    return v;
}

} // namespace

int main(int argc, const char* argv[]) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    std::string out_path        = argv[1];
    int         hz              = DEFAULT_HZ;
    int         duration_s      = DEFAULT_DURATION_S;
    uint64_t    x_addr          = 0;
    uint64_t    y_addr          = 0;
    bool        has_x_addr      = false;
    bool        has_y_addr      = false;
    bool        use_direct_xy   = false;
    double      x_trigger_max   = DEFAULT_X_TRIGGER_MAX;
    double      min_step        = DEFAULT_MIN_STEP;
    double      max_step        = DEFAULT_MAX_STEP;

    for (int i = 2; i < argc; ++i) {
        std::string flag = argv[i];
        auto need_value = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << name << " requires a value\n";
                std::exit(1);
            }
            return argv[++i];
        };
        if (flag == "--duration")            { if (!parse_int(need_value("--duration"), duration_s) || duration_s <= 0) { std::cerr << "Invalid --duration\n"; return 1; } }
        else if (flag == "--hz")             { if (!parse_int(need_value("--hz"), hz) || hz <= 0)              { std::cerr << "Invalid --hz\n"; return 1; } }
        else if (flag == "--x-addr")         { if (!parse_u64(need_value("--x-addr"), x_addr))                 { std::cerr << "Invalid --x-addr\n"; return 1; } has_x_addr = true; use_direct_xy = true; }
        else if (flag == "--y-addr")         { if (!parse_u64(need_value("--y-addr"), y_addr))                 { std::cerr << "Invalid --y-addr\n"; return 1; } has_y_addr = true; use_direct_xy = true; }
        else if (flag == "--x-trigger-max")  { if (!parse_double(need_value("--x-trigger-max"), x_trigger_max) || x_trigger_max <= 0) { std::cerr << "Invalid --x-trigger-max\n"; return 1; } }
        else if (flag == "--min-step")       { if (!parse_double(need_value("--min-step"), min_step))          { std::cerr << "Invalid --min-step\n"; return 1; } }
        else if (flag == "--max-step")       { if (!parse_double(need_value("--max-step"), max_step))          { std::cerr << "Invalid --max-step\n"; return 1; } }
        else if (flag == "--help" || flag == "-h") { print_usage(argv[0]); return 0; }
        else {
            std::cerr << "Unknown argument: " << flag << "\n";
            print_usage(argv[0]);
            return 1;
        }
    }
    if (use_direct_xy && !has_x_addr) {
        std::cerr << "Direct mode requires --x-addr.\n";
        return 1;
    }

    if (!SetConsoleCtrlHandler(on_console_ctrl, TRUE)) {
        std::cerr << "Warning: could not install Ctrl+C handler.\n";
    }

    // High-resolution sleep + boosted process/thread priority to reduce
    // scheduler jitter at 240Hz.
    timeBeginPeriod(1);
    SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);

    HackIH gd;
    if (!gd.bind("GeometryDash.exe")) {
        std::cerr << "Could not bind GeometryDash.exe (is it running?)\n";
        timeEndPeriod(1);
        return 3;
    }

    std::cout << "Attached to GeometryDash.exe.\n";
    std::cout << "Output: " << out_path << "  (" << hz << "Hz x " << duration_s << "s = "
              << static_cast<int64_t>(hz) * duration_s << " samples)\n";
    if (use_direct_xy) {
        std::cout << "Direct X mode: x=0x" << std::hex << x_addr;
        if (has_y_addr) {
            std::cout << " (y=0x" << y_addr << " ignored)";
        }
        std::cout << std::dec << "\n";
    } else {
        std::cout << "Pointer-chain XY mode (offsets 0x67C / 0x680).\n";
    }
    std::cout << "Trigger: first tick with x < " << x_trigger_max
              << " AND dx in [" << min_step << ", " << max_step << "] px/tick.\n"
              << "  Leading 0-bits will be synthesized so sample 0 == tick 0 at x = "
              << FIRST_TICK_X << ".\n";
    std::cout << "Start the level - capture begins on the first 240Hz tick where the player moves.\n";

    using clock = std::chrono::steady_clock;
    constexpr int64_t NS_PER_SEC = 1'000'000'000LL;
    const auto tick_offset = [hz](int64_t tick_index) -> std::chrono::nanoseconds {
        // Use absolute rational-time mapping instead of fixed integer step_ns so
        // rates like 240Hz stay phase-accurate over long captures.
        const long double ns =
            (static_cast<long double>(tick_index) * static_cast<long double>(NS_PER_SEC)) /
            static_cast<long double>(hz);
        return std::chrono::nanoseconds(static_cast<int64_t>(std::floor(ns)));
    };
    const auto tick_time = [&tick_offset](clock::time_point base, int64_t tick_index) -> clock::time_point {
        return base + tick_offset(tick_index);
    };

    // -------- Phase 1: catch the first in-band forward step near x=0. --------
    //
    // We don't gate on a "saw N consecutive zero ticks" precondition anymore,
    // because in practice GD's pointer chain often returns small-but-nonzero
    // x values the moment PlayLayer is alive (the player isn't pinned at
    // exactly 0 for long, and the chain may even resolve mid-motion). The
    // dx-in-band check alone reliably identifies "the level is running"; the
    // x < x_trigger_max check just rejects late starts deep inside a level.
    //
    // Whatever x we observe at the trigger tick, we back-compute how many
    // ticks earlier x was FIRST_TICK_X using the observed dx, and prepend that many
    // 0-bits ("not clicking") so sample 0 of the .bin still aligns with
    // tick 0.
    float  last_x       = 0.0f;
    bool   has_last_x   = false;
    double trigger_x    = 0.0;
    double trigger_dx   = 0.0;

    const auto phase1_base = clock::now();
    int64_t probe_tick = 0;
    bool triggered = false;
    while (g_running && !triggered) {
        ++probe_tick;
        const auto scheduled_probe = tick_time(phase1_base, probe_tick);
        std::this_thread::sleep_until(scheduled_probe);

        const float x = read_coord(gd, use_direct_xy, x_addr, OFF_X);
        if (!std::isfinite(x)) {
            // Pointer chain not yet resolved (PlayLayer might not exist).
            has_last_x = false;
            continue;
        }

        if (!has_last_x) {
            last_x = x;
            has_last_x = true;
            continue;
        }

        const double dx = static_cast<double>(x - last_x);
        last_x = x;

        if (dx >= min_step && dx <= max_step &&
            static_cast<double>(x) < x_trigger_max) {
            trigger_x  = static_cast<double>(x);
            trigger_dx = dx;
            triggered  = true;
            break;
        }
    }
    if (!g_running || !triggered) {
        timeEndPeriod(1);
        std::cerr << "Aborted before trigger.\n";
        return 1;
    }

    // How many ticks ago was x = FIRST_TICK_X? At the trigger tick the player is at
    // trigger_x and moving trigger_dx px/tick, so x=FIRST_TICK_X was approximately
    // (trigger_x - FIRST_TICK_X) / trigger_dx ticks before this tick. Clamp to
    // [0, MAX_PAD_TICKS] for safety.
    int pad_ticks = static_cast<int>(std::lround((trigger_x - FIRST_TICK_X) / trigger_dx));
    if (pad_ticks < 0)               pad_ticks = 0;
    if (pad_ticks > MAX_PAD_TICKS)   pad_ticks = MAX_PAD_TICKS;

    std::cout << "\n=== CAPTURE STARTED ===\n"
              << "Triggered: x=" << trigger_x << " dx=" << trigger_dx << " px/tick.\n"
              << "Synthesizing " << pad_ticks
              << " leading 0-bit ticks so sample 0 == tick 0 (x=" << FIRST_TICK_X << ").\n"
              << "Recording at " << hz << "Hz for up to " << duration_s
              << "s (" << static_cast<int64_t>(hz) * duration_s << " samples).\n"
              << "Press ESC at any time to stop and save.\n"
              << "=======================\n" << std::flush;

    // -------- Phase 2: 240Hz spacebar capture. --------
    const int64_t total_samples = static_cast<int64_t>(hz) * duration_s;
    BitBuffer bits;

    int64_t  sample_index = 0;
    int      last_bit     = 0;
    int64_t  max_late_ns  = 0;
    int64_t  missed_events = 0;
    int64_t  missed_ticks_total = 0;
    bool     esc_stopped  = false;

    // Clear any stale ESC state so a key pressed before capture doesn't
    // immediately end the recording.
    (void)GetAsyncKeyState(VK_ESCAPE_KEY);

    const auto start_ns = clock::now();
    // The trigger tick *itself* is sample 0, so its scheduled time is the
    // current probe tick we just woke up on above. Subsequent samples advance
    // by the exact hz rational schedule via tick_time().
    const auto capture_base = phase1_base;
    int64_t capture_tick = probe_tick;
    auto target = tick_time(capture_base, capture_tick);

    // Synthesize `pad_ticks` leading 0-bits so sample 0 == tick 0 (x=FIRST_TICK_X,
    // not clicking). Cap at total_samples just in case.
    for (int i = 0; i < pad_ticks && sample_index < total_samples; ++i) {
        bits.push(0);
        ++sample_index;
    }
    last_bit = 0;

    // Capture the trigger tick itself (sample == pad_ticks) immediately.
    if (sample_index < total_samples) {
        const bool pressed = (GetAsyncKeyState(VK_SPACE_KEY) & 0x8000) != 0;
        last_bit = pressed ? 1 : 0;
        bits.push(last_bit);
        ++sample_index;
    }

    while (g_running && sample_index < total_samples) {
        if ((GetAsyncKeyState(VK_ESCAPE_KEY) & 0x8000) != 0) {
            esc_stopped = true;
            std::cout << "\nESC pressed - stopping capture and saving...\n";
            break;
        }
        ++capture_tick;
        target = tick_time(capture_base, capture_tick);

        // Coarse sleep until shortly before the deadline, then yield/spin to
        // reduce oversleep at sub-5ms cadence.
        const auto sleep_until_tp = target - SLEEP_GUARD;
        if (sleep_until_tp > clock::now()) {
            std::this_thread::sleep_until(sleep_until_tp);
        }
        for (;;) {
            const auto now = clock::now();
            const auto remaining = target - now;
            if (remaining <= std::chrono::nanoseconds::zero()) break;
            if (remaining > YIELD_THRESHOLD) {
                std::this_thread::yield();
            } else {
                // spin
            }
        }

        const auto now = clock::now();
        const auto lateness = now - target;
        const int64_t lateness_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(lateness).count();
        if (lateness_ns > max_late_ns) max_late_ns = lateness_ns;

        int64_t missed = 0;
        while (sample_index < total_samples) {
            const auto next_target = tick_time(capture_base, capture_tick + 1);
            if (next_target > now) break;
            ++capture_tick;
            ++missed;
        }
        if (missed > 0) {
            ++missed_events;
            missed_ticks_total += missed;
            for (int64_t i = 0; i < missed && sample_index < total_samples; ++i) {
                bits.push(last_bit);
                ++sample_index;
            }
            if (sample_index >= total_samples) break;
        }

        const bool pressed = (GetAsyncKeyState(VK_SPACE_KEY) & 0x8000) != 0;
        last_bit = pressed ? 1 : 0;
        bits.push(last_bit);
        ++sample_index;
    }

    bits.flush();

    const auto end_ns = clock::now();
    const double elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(end_ns - start_ns).count();

    // Header is identical to bitstring/get_bitstring.py:
    //   magic "SP240BIN" (8 bytes) +
    //   <IIIb> = u32 hz, u32 duration_s, u32 total_samples, i8 unused_bits.
    // If stopped early via ESC (or Ctrl+C), persist what we actually captured
    // rather than what was originally requested. Round duration up so the
    // header's hz * duration is >= total_samples (consumers can clamp).
    const int64_t  actual_samples = sample_index;
    const int      actual_dur_s   = static_cast<int>((actual_samples + hz - 1) / hz);
    const uint32_t hz_u32      = static_cast<uint32_t>(hz);
    const uint32_t dur_u32     = static_cast<uint32_t>(actual_dur_s);
    const uint32_t total_u32   = static_cast<uint32_t>(actual_samples);
    const int8_t   unused_bits = static_cast<int8_t>((8 - (actual_samples % 8)) % 8);

    std::ofstream out(out_path, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
        std::cerr << "Could not open output file: " << out_path << "\n";
        timeEndPeriod(1);
        return 2;
    }
    out.write("SP240BIN", 8);
    out.write(reinterpret_cast<const char*>(&hz_u32),      sizeof(hz_u32));
    out.write(reinterpret_cast<const char*>(&dur_u32),     sizeof(dur_u32));
    out.write(reinterpret_cast<const char*>(&total_u32),   sizeof(total_u32));
    out.write(reinterpret_cast<const char*>(&unused_bits), sizeof(unused_bits));
    out.write(reinterpret_cast<const char*>(bits.bytes().data()),
              static_cast<std::streamsize>(bits.bytes().size()));
    out.flush();
    out.close();

    timeEndPeriod(1);

    std::cout << "\nDone" << (esc_stopped ? " (stopped by ESC)" : "") << ".\n";
    std::cout << "Samples written: " << sample_index << " / " << total_samples << "\n";
    std::cout << "Bytes:           " << bits.bytes().size() << "\n";
    std::cout << "Elapsed:         " << std::fixed << elapsed << " s\n";
    std::cout << "Max lateness:    " << (max_late_ns / 1e6) << " ms\n";
    std::cout << "Missed events:   " << missed_events << "\n";
    std::cout << "Missed ticks:    " << missed_ticks_total << "\n";
    std::cout << "Output:          " << out_path << "\n";

    return 0;
}
