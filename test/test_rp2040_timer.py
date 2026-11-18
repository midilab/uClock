"""Exercise the real RP2040 timer adapter against deferred SDK alarm reclamation."""

from pathlib import Path
import shutil
import subprocess
import tempfile


HARNESS = r"""
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

struct repeating_timer;
using TimerCallback = bool (*)(repeating_timer*);
struct repeating_timer {
    int64_t delay_us = 0;
    TimerCallback callback = nullptr;
    int alarm_id = 0;
};
struct Alarm {
    repeating_timer* timer = nullptr;
    bool cancelled = false;
};
Alarm alarms[16];
bool interrupts_disabled = false;
bool fail_allocation = false;
bool invalid_cancellation = false;
int allocations = 0;
int cancellations = 0;
int callbacks = 0;
bool change_inside_callback = false;

void check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}
uint32_t save_and_disable_interrupts() {
    bool previous = interrupts_disabled;
    interrupts_disabled = true;
    return previous;
}
void restore_interrupts(uint32_t previous) { interrupts_disabled = previous; }
void panic(const char*) { throw std::runtime_error("timer allocation failed"); }
bool add_repeating_timer_us(int64_t delay, TimerCallback callback, void*, repeating_timer* output) {
    ++allocations;
    output->delay_us = delay ? delay : 1;
    output->callback = callback;
    if (!fail_allocation) {
        for (int index = 0; index < 16; ++index) {
            if (!alarms[index].timer) {
                alarms[index] = {output, false};
                output->alarm_id = index + 1;
                return true;
            }
        }
    }
    output->alarm_id = -1;
    return false;
}
bool cancel_repeating_timer(repeating_timer* timer) {
    ++cancellations;
    if (timer->alarm_id < 0 || timer->alarm_id > 16) invalid_cancellation = true;
    bool valid = timer->alarm_id > 0 && timer->alarm_id <= 16;
    if (valid) alarms[timer->alarm_id - 1].cancelled = true;
    timer->alarm_id = 0;
    return valid;
}

#include "platforms/rp2040.h"

void uClockHandler() {
    ++callbacks;
    if (change_inside_callback) setTimer(1871);
}
void service_alarm_irq() {
    check(!interrupts_disabled, "test attempted to service a masked IRQ");
    for (auto& alarm : alarms) {
        if (alarm.cancelled) alarm = {};
    }
}
int64_t fire_timer() {
    service_alarm_irq();
    check(timer.alarm_id > 0 && timer.alarm_id <= 16, "clock timer lost its alarm registration");
    check(alarms[timer.alarm_id - 1].timer == &timer, "registered timer was reclaimed");
    uint32_t previous = save_and_disable_interrupts();
    bool repeat = timer.callback(&timer);
    restore_interrupts(previous);
    check(repeat, "clock callback unexpectedly stopped its timer");
    return timer.delay_us;
}

int main() {
    initTimer(5208);
    int initial_id = timer.alarm_id;
    check(fire_timer() == -5208, "initial fixed-rate period was incorrect");
    uint32_t previous = save_and_disable_interrupts();
    for (uint32_t update = 0; update < 128; ++update) setTimer(1871 + update % 7);
    check(interrupts_disabled, "period update enabled a caller's masked interrupts");
    restore_interrupts(previous);
    check(fire_timer() == -1872, "latest period was not adopted after rapid updates");
    check(timer.alarm_id == initial_id && allocations == 1 && cancellations == 0,
          "period updates replaced the live alarm");
    check(!invalid_cancellation, "period update passed a failed allocation ID to SDK cancellation");
    change_inside_callback = true;
    check(fire_timer() == -1871, "callback period change was not adopted on its return");
    check(timer.alarm_id == initial_id && allocations == 1, "callback period change replaced its alarm");
    change_inside_callback = false;
    setTimer(3742);
    check(!interrupts_disabled, "period update failed to restore enabled interrupts");
    check(fire_timer() == -3742, "slower tempo did not recover without restarting");
    setTimer(0);
    check(fire_timer() < 0, "zero period disabled fixed-rate scheduling");
    setTimer(UINT32_MAX);
    check(fire_timer() == -int64_t(UINT32_MAX), "period conversion overflowed signed 32-bit range");
    for (auto& alarm : alarms) alarm = {};
    timer.alarm_id = 0;
    fail_allocation = true;
    bool reported_failure = false;
    try { initTimer(5208); }
    catch (const std::runtime_error&) { reported_failure = true; }
    check(reported_failure && timer.alarm_id == 0, "initial allocation failure was silent or retained an invalid ID");
    std::cout << "PASS: RP2040 timer survives rapid/masked and callback period updates; allocation failure is explicit\n";
}
"""


def main():
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("g++") or shutil.which("clang++")
    if compiler is None:
        raise SystemExit("A native C++ compiler (g++ or clang++) is required")
    with tempfile.TemporaryDirectory(prefix="uclock-rp2040-") as temporary:
        directory = Path(temporary)
        (directory / "Arduino.h").write_text("")
        (directory / "pico").mkdir()
        (directory / "pico/sync.h").write_text("")
        source = directory / "timer.cpp"
        executable = directory / "timer"
        source.write_text(HARNESS)
        subprocess.run([
            compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
            "-fsanitize=undefined", "-fno-sanitize-recover=all",
            "-I", str(directory), "-I", str(root / "src"),
            str(source), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()