# uClock timing counters

uClock runs its sequencing logic once for every output PPQN timer tick. Most
`*_ref` values are periods measured in those timer ticks, while most
`*_counter` values are the current phase within one such period.

## Reference values

`mod_clock_ref = output_ppqn / input_ppqn`

This is the number of output timer ticks corresponding to one input clock
pulse. With 96 output PPQN and 24 input PPQN it is 4.

`mod_step_ref = output_ppqn / 4`

This is the number of output timer ticks in one sequencer step window. There
are four windows per quarter note, so at 96 PPQN it is 24 ticks. Despite the
name, it is a period rather than a reference to another object.

`SyncCallback::sync_ref = output_ppqn / callback_resolution`

This is the number of output timer ticks between calls to that sync callback.
A PPQN-24 callback under a PPQN-96 output clock therefore has a `sync_ref` of
4.

These divisions assume that the requested resolutions divide evenly. The
public PPQN enum currently supplies compatible values for the normal
configurations, but changing PPQN while running also changes the meaning of
all modulo phases.

## Clock counters and positions

`tick`

The global output-PPQN position. It increments once at the end of every
successful `handleInternalClock()` call. Output and step callbacks therefore
observe the pre-increment value. At 96 PPQN, 96 ticks represent one quarter
note.

`mod_clock_counter`

The phase of the generated clock relative to the input-clock resolution. At
the start of processing it is normalized from `mod_clock_ref` back to zero.
`int_clock_tick` advances whenever this phase is zero, then
`mod_clock_counter` increments.

For a 96/24 configuration, successive completed calls normally leave this
counter at 1, 2, 3, 4. The value 4 is normalized to zero at the beginning of
the following call. This is why an asynchronous debugger may see the
reference value even though the decision-time range is 0 through 3.

`int_clock_tick`

The internally generated position at input PPQN resolution. In external mode
it is compared with `ext_clock_tick` to detect phase error. It advances once
per `mod_clock_ref` output timer calls.

`ext_clock_tick`

The number of calls to `handleExternalClock()`. In external mode this is the
canonical input-pulse position used during startup and phase correction.

`SyncCallback::mod_counter`

The phase within that callback's `sync_ref` period. The callback runs when the
normalized phase is zero. As with `mod_clock_counter`, snapshots taken after a
handler may show values from 1 through `sync_ref`.

`SyncCallback::tick`

The logical invocation number supplied to that callback. It increments only
when the callback is dispatched. During external startup or phase correction
it is reconstructed from `tick / sync_ref`.

## Track and shuffle counters

`TRACK_SLOT::mod_step_counter`

The track's phase within a `mod_step_ref` window. Without shuffle, the step
callback fires at phase zero. Every track is processed synchronously by the
same `stepSeqTick()` invocation, so all tracks should have equal modulo phase
after the handler completes. A persistent difference is an invariant
violation, not normal shuffle.

`TRACK_SLOT::step_counter`

The logical step number passed to the callback. It advances only when that
track fires. Because shuffled tracks can fire at different phases, two valid
tracks can briefly show different logical step numbers around a boundary.
Comparing only `step_counter` values is therefore not sufficient to diagnose
drift.

For a shuffle value `shff`, the target phase is:

```text
shff >= 0: target = shff
shff < 0:  target = mod_step_ref + shff
```

At 96 PPQN, `mod_step_ref` is 24. A value of 6 fires at phase 6; a value of -6
fires at phase 18. The latter represents a position six ticks before the end
of the cyclic step window.

`shuffle_shoot_ctrl`

Controls which modulo cycle owns the pending logical step. After a step fires,
a nonnegative next offset waits until phase zero before it is armed. A negative
next offset is armed immediately because its target belongs before the next
phase-zero boundary. Treating this as only a duplicate guard causes alternating
`0, +N` templates to run at twice their intended rate.

`previous_shff` and `skip_next_phase_zero` preserve the nominal period after a
negative step. Because that step fires before its own boundary, a following
nonnegative step must skip the first phase zero and use the subsequent cycle.

`current_shff` and `current_shff_valid`

The shuffle offset latched for the pending logical step. Live template edits
do not change a target after timing for that step has begun; they apply when a
later step first becomes pending. This prevents moving an equality-based
target behind `mod_step_counter` and losing the step for a full cycle.

`shuffle_length_ctrl`

A derived adjustment based on the current and next shuffle values. Consumers
use it to adjust note length; it does not determine the global clock phase.

## Startup and phase correction

`start()` resets all positions. Internal mode enters `STARTED` immediately.
External mode enters `STARTING`, waits for external pulses in `SYNCING`, then
sets:

```text
tick = ext_clock_tick * mod_clock_ref
int_clock_tick = ext_clock_tick
step_counter = tick / mod_step_ref
mod_step_counter = 0
```

Legacy non-strict external phase correction performs the same reconstruction
at a configured quarter-note boundary. Strict external mode does not use this
startup or ongoing reconstruction.

### Strict external pulse ownership

With strict external mode enabled, each input pulse normally authorizes
`mod_clock_ref` output ticks. At 24 PPQN input and 96 PPQN output, one MIDI
Clock pulse therefore owns four output ticks. At 24 PPQN and above,
still-pending subdivisions remain authorized when another pulse arrives. At
lower input rates, a new pulse replaces stale pending subdivisions so delayed
work cannot build into a multi-beat burst. Work is processed at timer cadence
rather than replayed synchronously in the pulse handler. Strict external mode
does not reconstruct counters; callback positions remain monotonic. The pulse
handler processes at most one tick and timer callbacks process the remaining
authorized subdivisions. They cannot advance without an external pulse budget.

Lower input rates use the same rule. At 96 PPQN output, one 4-PPQN pulse owns
24 output ticks and six PPQN-24 callbacks. One 1-PPQN pulse owns 96 output
ticks and 24 PPQN-24 callbacks.

A low-rate pulse is also a phase observation. When the next callback is not on
the input grid, uClock selects the nearest future grid boundary and adjusts both
the tick budget and timer interval for one input period. This reaches the next
edge by running sequential callbacks slightly faster or slower; it never rewinds
position, skips callback IDs, or executes multiple ticks at once. Once aligned,
a 1-PPQN edge and its quarter-note callback occur together.

Tempo estimation uses approximately one quarter note of recent intervals,
capped by the configured buffer size. A 1-PPQN source therefore adopts each new
valid beat interval immediately; a 4-PPQN source smooths over four pulses.

This makes callback positions monotonic and removes the need for hard
quarter-note counter reconstruction in strict mode. Hard reconstruction can
move callback indices backward or forward, replaying or skipping musical
events. Non-strict external mode retains the legacy phase-correction behavior.

If pulses disappear, already authorized subdivisions may complete and then
the clock waits without entering `PAUSED` or `STOPED`. Song position and the
logical playing state are preserved. `isExternalClockStalled()` becomes true
after four accepted pulse periods (with a 20 ms minimum), and a later pulse
resumes from the next logical tick.

A gap classified as a discontinuity is not added to the tempo average. The
interval buffer is cleared and the last good tempo is retained until a normal
interval arrives. Strict-mode tempo updates occur once per accepted external
pulse rather than once per internal timer callback.

External pulse timestamps are supplied by the selected platform. The default
implementation uses `micros()`. Teensy 4 uses its free-running DWT cycle
counter because globally disabled interrupts can delay SysTick accounting and
make `micros()` undercount. Define `UCLOCK_EXTERNAL_CLOCK_USE_MICROS` to force
the legacy source for an A/B build.

Projects that detect an edge before they can safely dispatch it should call
`clockMeAt(observed_at_us)` later. Tempo estimation then uses the captured edge
time rather than queue or main-loop latency. `clockMe()` remains the preferred
immediate path and retains platform-specific timestamp precision. Hardware
capture, debouncing, interrupt queues, and pulse-width guarantees belong to
the project adapter rather than uClock.

### Internal tap phase slew

`syncInternalClockToBeat(observed_at_us)` treats a tap as a quarter-note phase
observation while remaining in internal mode. It selects the nearest beat-grid
phase error and distributes that correction over the next `output_ppqn` ticks.
The correction is limited to half of each normal tick interval, so timer
intervals remain positive. Tick IDs, sync callback counters, track positions,
shuffle state, and application time-signature offsets are never rewritten.

`getInternalPhaseCorrectionUs()` reports the most recent signed correction and
`getInternalPhaseSlewTicksRemaining()` reports convergence progress. A later
tap replaces the remaining correction using the newest tempo and observation.

MIDI Start remains a transport reset to tick zero. MIDI Continue resumes the
preserved local position. MIDI Clock alone contains no song-position data, so
after a disconnection uClock cannot infer that the sender moved to another bar;
that requires a new Start or MIDI Song Position Pointer support.

## Overflow counters

`int_overflow_counter` and `ext_overflow_counter` are handler nesting-depth
markers, not cumulative interrupt totals. A normal completed call increments
and then decrements its value, so it should return to zero. A value greater
than one while inside a handler indicates re-entry. Structured trace events
record such re-entry without formatting or serial output in the timing path.

## Structured trace

Define `UCLOCK_ENABLE_TRACE` to enable the fixed-size trace ring. Override
`UCLOCK_TRACE_BUFFER_SIZE` if the default 8192 records is unsuitable.

The producer performs no allocation, string formatting, or serial I/O. Call
`popTraceEvent()` from non-interrupt code to drain records and
`getTraceDroppedCount()` to detect discarded records. During normal operation,
a full ring discards its oldest event so recent history is retained. The first
step-phase divergence, handler re-entry, or invalid-state event freezes the
ring after recording that event. Further producer attempts increment the
dropped count without overwriting the captured lead-up. `isTraceFrozen()`
reports this state, and `clearTrace()` clears and re-arms the ring.

Important events include external pulses, the beginning of a phase error,
phase correction, PLL tempo changes (reported as BPM times 1000), shuffled
step firing, live template changes, shuffle enable/disable transitions,
per-track modulo-phase divergence, and handler re-entry. A `shuffle_change`
record stores the template index in `step`, the new offset in `sh`, and the
previous offset in `value`. A `shuffle_state` record stores the new enabled
state in `sh` and the previous state in `value`.

Define `UCLOCK_TRACE_EXTERNAL_CLOCK_TIMING` with `UCLOCK_ENABLE_TRACE` to emit
an `external_timing_delta` record after each measured pulse. Its `value` is
the `micros()` interval minus the platform interval, in microseconds. On a
platform whose timestamp source is `micros()`, the value is zero. On Teensy 4,
negative values show time omitted by `micros()` relative to the DWT counter.
The preceding `external_pulse` value is always the interval selected for tempo
estimation.

A `TRACE_STEP_PHASE_DIVERGED` event indicates a strong failure because shuffle
should alter callback time, not the underlying per-track modulo phase.

In the `pcb_studio` firmware build, use the serial console commands below:

```text
uclocktrace clear
uclocktrace on
uclocktrace status
uclocktrace off
```

Output records begin with `UCLOCK,`. Enable output shortly before attempting a
reproduction. The main loop drains at most eight records per pass. If the
`dropped` field increases while the trace is rolling, old records were
overwritten before the main loop could print them. While frozen, it counts new
records rejected to preserve the captured failure. Use `uclocktrace status` to
distinguish these cases.
