#include <vector>

#include <unity.h>

#include "uClock.h"

extern uint32_t uclock_us_interval;

static uint32_t fake_micros = 0;
static umodular::clock::uClockClass *callback_clock = nullptr;
static std::vector<uint32_t> fired_steps;
static std::vector<uint32_t> fired_ticks;
static std::vector<int8_t> shuffle_lengths;
static std::vector<uint32_t> sync_24_ticks;
static uint32_t sync_24_count = 0;
static uint32_t sync_4_count = 0;
static bool request_reentrant_tick = false;
static std::vector<uint8_t> callback_order;

uint32_t micros()
{
    return fake_micros;
}

uint32_t millis()
{
    return fake_micros / 1000;
}

static void record_step(uint32_t step)
{
    fired_steps.push_back(step);
    uint32_t callback_tick = callback_clock->tick;
    fired_ticks.push_back(callback_tick);
    shuffle_lengths.push_back(callback_clock->getShuffleLength());
}

static void record_sync_24(uint32_t tick)
{
    sync_24_ticks.push_back(tick);
    ++sync_24_count;
    if (request_reentrant_tick) {
        request_reentrant_tick = false;
        callback_clock->handleInternalClock();
    }
}

static void record_sync_4(uint32_t)
{
    ++sync_4_count;
}

static void record_order_sync(uint32_t)
{
    callback_order.push_back(1);
}

static void record_order_step(uint32_t)
{
    callback_order.push_back(2);
}

static void record_order_end(uint32_t)
{
    callback_order.push_back(3);
}

static void run_until_tick(umodular::clock::uClockClass &clock, uint32_t end_tick)
{
    while (clock.tick < end_tick) {
        fake_micros += 1000;
        clock.handleInternalClock();
    }
}

void setUp()
{
    fake_micros = 0;
    callback_clock = nullptr;
    fired_steps.clear();
    fired_ticks.clear();
    shuffle_lengths.clear();
    sync_24_ticks.clear();
    sync_24_count = 0;
    sync_4_count = 0;
    request_reentrant_tick = false;
    callback_order.clear();
}

void tearDown()
{
}

static void test_sync_callback_growth_preserves_existing_entries()
{
    umodular::clock::uClockClass clock;
    clock.setOnSync(umodular::clock::uClockClass::PPQN_24, record_sync_24);
    clock.setOnSync(umodular::clock::uClockClass::PPQN_4, record_sync_4);
    clock.init();

    TEST_ASSERT_EQUAL_UINT8(2, clock.sync_callback_size);
    TEST_ASSERT_EQUAL_UINT16(4, clock.sync_callbacks[0].sync_ref);
    TEST_ASSERT_EQUAL_UINT16(24, clock.sync_callbacks[1].sync_ref);

    clock.setOnSync(umodular::clock::uClockClass::PPQN_1, record_sync_4);
    TEST_ASSERT_EQUAL_UINT8(2, clock.sync_callback_size);

    clock.clock_state = umodular::clock::uClockClass::STARTED;
    run_until_tick(clock, 8);

    TEST_ASSERT_EQUAL_UINT32(2, sync_24_count);
    TEST_ASSERT_EQUAL_UINT32(1, sync_4_count);
}

static void test_reduced_ppqn_recovers_counters_above_new_references()
{
    umodular::clock::uClockClass clock;
    callback_clock = &clock;
    clock.setOnStep(record_step);
    clock.setOnSync(umodular::clock::uClockClass::PPQN_24, record_sync_24);
    clock.init();
    clock.mod_clock_counter = 3;
    clock.tracks[0].mod_step_counter = 20;
    clock.sync_callbacks[0].mod_counter = 3;

    clock.setOutputPPQN(umodular::clock::uClockClass::PPQN_48);
    clock.clock_state = umodular::clock::uClockClass::STARTED;
    run_until_tick(clock, 1);

    TEST_ASSERT_EQUAL_UINT16(1, clock.mod_clock_counter);
    TEST_ASSERT_EQUAL_UINT8(1, clock.tracks[0].mod_step_counter);
    TEST_ASSERT_EQUAL_UINT16(1, clock.sync_callbacks[0].mod_counter);
    TEST_ASSERT_EQUAL_UINT32(1, sync_24_count);
    TEST_ASSERT_EQUAL_UINT32(1, fired_steps.size());
}

static void test_output_tick_end_runs_after_sync_and_step_callbacks()
{
    umodular::clock::uClockClass clock;
    clock.setOnSync(umodular::clock::uClockClass::PPQN_96, record_order_sync);
    clock.setOnStep(record_order_step);
    clock.setOnOutputPPQNEnd(record_order_end);
    clock.init();
    clock.clock_state = umodular::clock::uClockClass::STARTED;

    clock.handleInternalClock();

    const uint8_t expected_order[] = {1, 2, 3};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_order, callback_order.data(), 3);
}

static void test_strict_external_clock_allows_only_one_output_group_per_pulse()
{
    umodular::clock::uClockClass clock;
    clock.setClockMode(umodular::clock::uClockClass::EXTERNAL_CLOCK);
    clock.setStrictExternalMode(true);
    clock.setOnSync(umodular::clock::uClockClass::PPQN_24, record_sync_24);
    clock.init();
    clock.start();

    fake_micros = 1000;
    clock.clockMe();

    TEST_ASSERT_EQUAL_UINT32(1, sync_24_count);
    TEST_ASSERT_EQUAL_UINT32(1, clock.tick);

    for (uint8_t i = 0; i < 10; i++) {
        fake_micros += 1000;
        clock.handleInternalClock();
    }

    TEST_ASSERT_EQUAL_UINT32(4, clock.tick);
    TEST_ASSERT_EQUAL_UINT32(1, sync_24_count);
}

static void test_back_to_back_external_pulses_preserve_both_output_groups()
{
    umodular::clock::uClockClass clock;
    clock.setClockMode(umodular::clock::uClockClass::EXTERNAL_CLOCK);
    clock.setStrictExternalMode(true);
    clock.setOnSync(umodular::clock::uClockClass::PPQN_24, record_sync_24);
    clock.init();
    clock.start();

    fake_micros = 1000;
    clock.clockMe();
    clock.clockMe();

    TEST_ASSERT_EQUAL_UINT32(2, clock.ext_clock_tick);
    TEST_ASSERT_EQUAL_UINT32(2, clock.tick);
    TEST_ASSERT_EQUAL_UINT32(1, sync_24_count);
    TEST_ASSERT_EQUAL_UINT16(6, clock.getExternalTicksRemaining());

    for (uint8_t i = 0; i < 10; i++)
        clock.handleInternalClock();

    TEST_ASSERT_EQUAL_UINT32(8, clock.tick);
    TEST_ASSERT_EQUAL_UINT32(2, sync_24_count);
}

static void assert_low_ppqn_pulse_budget(
    umodular::clock::uClockClass::PPQNResolution input_ppqn,
    uint32_t expected_output_ticks,
    uint32_t expected_sync_ticks)
{
    umodular::clock::uClockClass clock;
    clock.setOutputPPQN(umodular::clock::uClockClass::PPQN_96);
    clock.setInputPPQN(input_ppqn);
    clock.setClockMode(umodular::clock::uClockClass::EXTERNAL_CLOCK);
    clock.setStrictExternalMode(true);
    clock.setOnSync(umodular::clock::uClockClass::PPQN_24, record_sync_24);
    clock.init();
    clock.start();

    fake_micros = 1000;
    clock.clockMe();
    while (clock.getExternalTicksRemaining() > 0) {
        fake_micros += 1000;
        clock.handleInternalClock();
    }

    TEST_ASSERT_EQUAL_UINT32(1, clock.ext_clock_tick);
    TEST_ASSERT_EQUAL_UINT32(expected_output_ticks, clock.tick);
    TEST_ASSERT_EQUAL_UINT32(expected_sync_ticks, sync_24_ticks.size());
    for (uint32_t index = 0; index < sync_24_ticks.size(); index++)
        TEST_ASSERT_EQUAL_UINT32(index, sync_24_ticks[index]);

    for (uint8_t attempt = 0; attempt < 10; attempt++)
        clock.handleInternalClock();
    TEST_ASSERT_EQUAL_UINT32(expected_output_ticks, clock.tick);
}

static void test_strict_external_ppqn_4_pulse_authorizes_quarter_group()
{
    assert_low_ppqn_pulse_budget(
        umodular::clock::uClockClass::PPQN_4, 24, 6);
}

static void test_strict_external_ppqn_16_pulse_authorizes_six_output_ticks()
{
    assert_low_ppqn_pulse_budget(
        umodular::clock::uClockClass::PPQN_16, 6, 2);
}

static void test_strict_external_ppqn_1_pulse_authorizes_full_quarter()
{
    assert_low_ppqn_pulse_budget(
        umodular::clock::uClockClass::PPQN_1, 96, 24);
}

static void test_live_switch_to_ppqn_1_external_clock_keeps_timer_progressing()
{
    umodular::clock::uClockClass clock;
    clock.setOutputPPQN(umodular::clock::uClockClass::PPQN_96);
    clock.setStrictExternalMode(true);
    clock.setOnSync(umodular::clock::uClockClass::PPQN_24, record_sync_24);
    clock.init();
    clock.start();

    fake_micros = 1000;
    clock.handleInternalClock();
    clock.setInputPPQN(umodular::clock::uClockClass::PPQN_1);
    clock.setClockMode(umodular::clock::uClockClass::EXTERNAL_CLOCK);

    fake_micros = 501000;
    clock.clockMeAt(fake_micros);
    for (uint8_t index = 1; index < 96; index++) {
        fake_micros += 5208;
        clock.handleInternalClock();
    }

    TEST_ASSERT_EQUAL_UINT8(umodular::clock::uClockClass::STARTED,
                            clock.clock_state);
    TEST_ASSERT_EQUAL_UINT32(96, clock.tick);
    TEST_ASSERT_EQUAL_UINT32(24, sync_24_ticks.size());
    TEST_ASSERT_EQUAL_UINT16(0, clock.getExternalTicksRemaining());
}

static void test_late_ppqn_1_pulse_does_not_burst_pending_ticks()
{
    umodular::clock::uClockClass clock;
    clock.setOutputPPQN(umodular::clock::uClockClass::PPQN_96);
    clock.setInputPPQN(umodular::clock::uClockClass::PPQN_1);
    clock.setClockMode(umodular::clock::uClockClass::EXTERNAL_CLOCK);
    clock.setStrictExternalMode(true);
    clock.setOnSync(umodular::clock::uClockClass::PPQN_24, record_sync_24);
    clock.init();
    clock.start();

    fake_micros = 1000;
    clock.clockMeAt(fake_micros);
    TEST_ASSERT_EQUAL_UINT32(1, clock.tick);
    TEST_ASSERT_EQUAL_UINT16(95, clock.getExternalTicksRemaining());

    fake_micros = 501000;
    clock.clockMeAt(fake_micros);

    TEST_ASSERT_EQUAL_UINT32(2, clock.tick);
    TEST_ASSERT_EQUAL_UINT32(1, sync_24_ticks.size());
    TEST_ASSERT_EQUAL_UINT16(94, clock.getExternalTicksRemaining());
}

static void test_reentering_external_mode_discards_stale_authorization()
{
    umodular::clock::uClockClass clock;
    clock.setOutputPPQN(umodular::clock::uClockClass::PPQN_96);
    clock.setInputPPQN(umodular::clock::uClockClass::PPQN_1);
    clock.setClockMode(umodular::clock::uClockClass::EXTERNAL_CLOCK);
    clock.setStrictExternalMode(true);
    clock.init();
    clock.start();

    fake_micros = 1000;
    clock.clockMeAt(fake_micros);
    TEST_ASSERT_EQUAL_UINT16(95, clock.getExternalTicksRemaining());

    clock.setClockMode(umodular::clock::uClockClass::INTERNAL_CLOCK);
    clock.setClockMode(umodular::clock::uClockClass::EXTERNAL_CLOCK);

    TEST_ASSERT_EQUAL_UINT16(0, clock.getExternalTicksRemaining());
}

static void test_reentrant_internal_callback_does_not_advance_twice()
{
    umodular::clock::uClockClass clock;
    callback_clock = &clock;
    clock.setOnSync(umodular::clock::uClockClass::PPQN_24, record_sync_24);
    clock.init();
    clock.clock_state = umodular::clock::uClockClass::STARTED;
    request_reentrant_tick = true;

    clock.handleInternalClock();

    TEST_ASSERT_EQUAL_UINT32(1, clock.tick);
    TEST_ASSERT_EQUAL_UINT32(1, sync_24_count);
    TEST_ASSERT_EQUAL_UINT16(0, clock.getIntOverflowCounter());
    TEST_ASSERT_FALSE(clock.isTraceFrozen());

    clock.traceEvent(umodular::clock::uClockClass::TRACE_EXTERNAL_PULSE);
    umodular::clock::uClockClass::TraceEvent event;
    bool found_following_pulse = false;
    while (clock.popTraceEvent(event)) {
        if (event.type == umodular::clock::uClockClass::TRACE_EXTERNAL_PULSE)
            found_following_pulse = true;
    }
    TEST_ASSERT_TRUE(found_following_pulse);
}

static void test_external_tempo_ignores_short_startup_interval()
{
    umodular::clock::uClockClass clock;
    clock.setClockMode(umodular::clock::uClockClass::EXTERNAL_CLOCK);
    clock.setStrictExternalMode(true);
    clock.init();
    clock.start();

    fake_micros = 1000;
    clock.clockMe();
    fake_micros += 11354;
    clock.clockMe();
    fake_micros += 33333;
    clock.clockMe();
    fake_micros += 33334;
    clock.clockMe();

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 75.0f, clock.getTempo());
}

static void test_timestamped_external_pulses_ignore_delivery_latency()
{
    umodular::clock::uClockClass clock;
    clock.setOutputPPQN(umodular::clock::uClockClass::PPQN_96);
    clock.setInputPPQN(umodular::clock::uClockClass::PPQN_1);
    clock.setClockMode(umodular::clock::uClockClass::EXTERNAL_CLOCK);
    clock.setStrictExternalMode(true);
    clock.init();
    clock.start();

    fake_micros = 9000000;
    clock.clockMeAt(1000000);
    clock.clockMeAt(1500000);
    clock.clockMeAt(2000000);
    clock.clockMeAt(2500000);

    TEST_ASSERT_EQUAL_UINT32(4, clock.ext_clock_tick);
    TEST_ASSERT_EQUAL_UINT32(500000, clock.ext_interval);
    TEST_ASSERT_EQUAL_UINT32(500000, clock.getLastAcceptedExternalInterval());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 120.0f, clock.getTempo());
}

static void test_ppqn_1_tempo_tracks_each_new_interval()
{
    umodular::clock::uClockClass clock;
    clock.setOutputPPQN(umodular::clock::uClockClass::PPQN_96);
    clock.setInputPPQN(umodular::clock::uClockClass::PPQN_1);
    clock.setClockMode(umodular::clock::uClockClass::EXTERNAL_CLOCK);
    clock.setStrictExternalMode(true);
    clock.init();
    clock.start();

    clock.clockMeAt(1000000);
    clock.clockMeAt(1500000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 120.0f, clock.getTempo());

    clock.clockMeAt(2500000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f, clock.getTempo());
}

static void test_ppqn_1_phase_slew_aligns_next_quarter_boundary()
{
    umodular::clock::uClockClass clock;
    callback_clock = &clock;
    clock.setOutputPPQN(umodular::clock::uClockClass::PPQN_96);
    clock.setInputPPQN(umodular::clock::uClockClass::PPQN_1);
    clock.setClockMode(umodular::clock::uClockClass::EXTERNAL_CLOCK);
    clock.setStrictExternalMode(true);
    clock.setOnStep(record_step);
    clock.setOnSync(umodular::clock::uClockClass::PPQN_24, record_sync_24);
    clock.init();
    clock.start();

    clock.clockMeAt(1000000);
    for (uint8_t index = 0; index < 70; index++)
        clock.handleInternalClock();
    TEST_ASSERT_EQUAL_UINT32(71, clock.tick);

    clock.clockMeAt(1500000);

    TEST_ASSERT_EQUAL_UINT32(72, clock.tick);
    TEST_ASSERT_EQUAL_UINT32(120, clock.getExternalTicksRemaining());
    TEST_ASSERT_EQUAL_UINT32(4132, uclock_us_interval);

    for (uint8_t index = 1; index < 121; index++)
        clock.handleInternalClock();
    TEST_ASSERT_EQUAL_UINT32(192, clock.tick);

    clock.clockMeAt(2000000);
    TEST_ASSERT_EQUAL_UINT32(193, clock.tick);
    TEST_ASSERT_EQUAL_UINT32(192, fired_ticks.back());
    TEST_ASSERT_EQUAL_UINT32(48, sync_24_ticks.back());
}

static void test_internal_beat_sync_slews_without_changing_tick_sequence()
{
    umodular::clock::uClockClass clock;
    clock.setOnSync(umodular::clock::uClockClass::PPQN_24, record_sync_24);
    clock.init();
    clock.start();

    fake_micros = 1000;
    clock.handleInternalClock();

    fake_micros = 126000;
    TEST_ASSERT_TRUE(clock.syncInternalClockToBeat(fake_micros));
    TEST_ASSERT_EQUAL_INT32(125000, clock.getInternalPhaseCorrectionUs());
    TEST_ASSERT_EQUAL_UINT16(96, clock.getInternalPhaseSlewTicksRemaining());

    for (uint16_t index = 0; index < 96; index++) {
        fake_micros += 5208;
        clock.handleInternalClock();
    }

    TEST_ASSERT_EQUAL_UINT16(0, clock.getInternalPhaseSlewTicksRemaining());
    TEST_ASSERT_EQUAL_UINT32(97, clock.tick);
    TEST_ASSERT_EQUAL_UINT32(25, sync_24_ticks.size());
    for (uint32_t index = 0; index < sync_24_ticks.size(); index++)
        TEST_ASSERT_EQUAL_UINT32(index, sync_24_ticks[index]);
}

static void test_early_external_pulse_catches_up_without_skipping_callbacks()
{
    umodular::clock::uClockClass clock;
    clock.setClockMode(umodular::clock::uClockClass::EXTERNAL_CLOCK);
    clock.setStrictExternalMode(true);
    clock.setOnSync(umodular::clock::uClockClass::PPQN_24, record_sync_24);
    clock.init();
    clock.start();

    fake_micros = 1000;
    clock.clockMe();
    fake_micros = 21000;
    clock.clockMe();

    TEST_ASSERT_EQUAL_UINT32(2, clock.tick);
    TEST_ASSERT_EQUAL_UINT32(1, sync_24_ticks.size());
    TEST_ASSERT_EQUAL_UINT16(6, clock.external_ticks_remaining);

    while (clock.getExternalTicksRemaining() > 0)
        clock.handleInternalClock();

    const uint32_t expected_ticks[] = {0, 1};
    TEST_ASSERT_EQUAL_UINT32(8, clock.tick);
    TEST_ASSERT_EQUAL_UINT32(2, sync_24_ticks.size());
    TEST_ASSERT_EQUAL_UINT32_ARRAY(expected_ticks, sync_24_ticks.data(), 2);
}

static void test_external_clock_resumes_after_dropout_without_resetting_position()
{
    umodular::clock::uClockClass clock;
    clock.setClockMode(umodular::clock::uClockClass::EXTERNAL_CLOCK);
    clock.setStrictExternalMode(true);
    clock.setOnSync(umodular::clock::uClockClass::PPQN_24, record_sync_24);
    clock.init();
    clock.start();

    fake_micros = 1000;
    clock.clockMe();
    fake_micros = 21000;
    clock.clockMe();
    TEST_ASSERT_EQUAL_UINT32(20000, clock.getLastAcceptedExternalInterval());

    for (uint8_t i = 0; i < 10; i++) {
        fake_micros += 1000;
        clock.handleInternalClock();
    }
    TEST_ASSERT_EQUAL_UINT32(8, clock.tick);

    fake_micros += 81000;
    for (uint8_t i = 0; i < 10; i++)
        clock.handleInternalClock();
    TEST_ASSERT_EQUAL_UINT32(8, clock.tick);
    TEST_ASSERT_EQUAL_UINT8(umodular::clock::uClockClass::STARTED, clock.clock_state);
    TEST_ASSERT_TRUE(clock.isExternalClockStalled());

    clock.clockMe();
    TEST_ASSERT_FALSE(clock.isExternalClockStalled());
    TEST_ASSERT_EQUAL_UINT32(20000, clock.getLastAcceptedExternalInterval());

    fake_micros += 20000;
    clock.clockMe();

    while (clock.getExternalTicksRemaining() > 0)
        clock.handleInternalClock();

    const uint32_t expected_ticks[] = {0, 1, 2, 3};
    TEST_ASSERT_EQUAL_UINT32(16, clock.tick);
    TEST_ASSERT_EQUAL_UINT32(4, sync_24_ticks.size());
    TEST_ASSERT_EQUAL_UINT32_ARRAY(expected_ticks, sync_24_ticks.data(), 4);
    TEST_ASSERT_EQUAL_UINT32(20000, clock.getLastAcceptedExternalInterval());
}

static void test_external_tempo_changes_do_not_duplicate_or_skip_sync_ticks()
{
    umodular::clock::uClockClass clock;
    clock.setClockMode(umodular::clock::uClockClass::EXTERNAL_CLOCK);
    clock.setStrictExternalMode(true);
    clock.setOnSync(umodular::clock::uClockClass::PPQN_24, record_sync_24);
    clock.init();
    clock.start();

    const uint32_t pulse_intervals[] = {
        21000, 21000, 18000, 18000, 25000, 25000, 16000, 30000
    };
    for (uint32_t pulse = 0; pulse < 32; pulse++) {
        uint8_t timer_ticks_before_pulse = pulse % 4;
        for (uint8_t tick_index = 0; tick_index < timer_ticks_before_pulse; tick_index++) {
            fake_micros += 1000;
            clock.handleInternalClock();
        }
        fake_micros += pulse_intervals[pulse % 8];
        clock.clockMe();
    }

    while (clock.getExternalTicksRemaining() > 0)
        clock.handleInternalClock();

    TEST_ASSERT_EQUAL_UINT32(32, sync_24_ticks.size());
    for (uint32_t i = 0; i < sync_24_ticks.size(); i++)
        TEST_ASSERT_EQUAL_UINT32(i, sync_24_ticks[i]);
}

static void test_external_continue_preserves_position_and_start_rewinds()
{
    umodular::clock::uClockClass clock;
    clock.setClockMode(umodular::clock::uClockClass::EXTERNAL_CLOCK);
    clock.setStrictExternalMode(true);
    clock.setOnSync(umodular::clock::uClockClass::PPQN_24, record_sync_24);
    clock.init();
    clock.start();

    fake_micros = 1000;
    clock.clockMe();
    while (clock.getExternalTicksRemaining() > 0) {
        fake_micros += 1000;
        clock.handleInternalClock();
    }

    clock.pause();
    clock.pause();
    fake_micros += 20000;
    clock.clockMe();

    TEST_ASSERT_EQUAL_UINT32(2, sync_24_ticks.size());
    TEST_ASSERT_EQUAL_UINT32(1, sync_24_ticks[1]);

    clock.start();
    fake_micros += 20000;
    clock.clockMe();

    TEST_ASSERT_EQUAL_UINT32(3, sync_24_ticks.size());
    TEST_ASSERT_EQUAL_UINT32(0, sync_24_ticks[2]);
}

static void test_mid_cycle_shuffle_activation_does_not_add_a_step()
{
    umodular::clock::uClockClass clock;
    callback_clock = &clock;
    clock.setOnStep(record_step);
    clock.init();
    clock.clock_state = umodular::clock::uClockClass::STARTED;
    run_until_tick(clock, 10);

    int8_t shuffle_template[] = {8, -8};
    clock.setShuffleTemplate(shuffle_template, 2);
    clock.setShuffle(true);
    run_until_tick(clock, 40);

    const uint32_t expected_steps[] = {0, 1};
    const uint32_t expected_ticks[] = {0, 16};
    TEST_ASSERT_EQUAL_UINT32(2, fired_steps.size());
    TEST_ASSERT_EQUAL_UINT32(2, fired_ticks.size());
    TEST_ASSERT_EQUAL_UINT32_ARRAY(expected_steps, fired_steps.data(), 2);
    TEST_ASSERT_EQUAL_UINT32_ARRAY(expected_ticks, fired_ticks.data(), 2);
}

static void test_shuffle_length_is_next_offset_minus_current_offset()
{
    umodular::clock::uClockClass clock;
    callback_clock = &clock;
    clock.setOnStep(record_step);
    clock.init();
    clock.clock_state = umodular::clock::uClockClass::STARTED;

    int8_t shuffle_template[] = {8, -8};
    clock.setShuffleTemplate(shuffle_template, 2);
    clock.setShuffle(true);
    run_until_tick(clock, 40);

    const int8_t expected_lengths[] = {-16, 16};
    TEST_ASSERT_EQUAL_UINT32(2, shuffle_lengths.size());
    TEST_ASSERT_EQUAL_INT8_ARRAY(expected_lengths, shuffle_lengths.data(), 2);
}

static void test_shuffle_offset_wraps_at_template_size()
{
    umodular::clock::uClockClass clock;
    clock.setOnStep(record_step);
    clock.init();

    int8_t shuffle_template[] = {3, -5, 7};
    clock.setShuffleTemplate(shuffle_template, 3);

    TEST_ASSERT_EQUAL_INT8(3, clock.getShuffleOffset(0));
    TEST_ASSERT_EQUAL_INT8(-5, clock.getShuffleOffset(1));
    TEST_ASSERT_EQUAL_INT8(7, clock.getShuffleOffset(2));
    TEST_ASSERT_EQUAL_INT8(3, clock.getShuffleOffset(3));
}

static void test_live_shuffle_update_preserves_latched_step()
{
    umodular::clock::uClockClass clock;
    callback_clock = &clock;
    clock.setOnStep(record_step);
    clock.init();
    clock.clock_state = umodular::clock::uClockClass::STARTED;

    int8_t initial_template[] = {8, -8};
    clock.setShuffleTemplate(initial_template, 2);
    clock.setShuffle(true);
    run_until_tick(clock, 10);

    int8_t updated_template[] = {8, 8};
    clock.setShuffleTemplate(updated_template, 2);
    run_until_tick(clock, 60);

    const uint32_t expected_steps[] = {0, 1, 2};
    const uint32_t expected_ticks[] = {8, 16, 56};
    TEST_ASSERT_EQUAL_UINT32(3, fired_steps.size());
    TEST_ASSERT_EQUAL_UINT32(3, fired_ticks.size());
    TEST_ASSERT_EQUAL_UINT32_ARRAY(expected_steps, fired_steps.data(), 3);
    TEST_ASSERT_EQUAL_UINT32_ARRAY(expected_ticks, fired_ticks.data(), 3);
}

static void test_trace_freezes_after_first_anomaly()
{
    using Clock = umodular::clock::uClockClass;
    Clock clock;
    clock.clearTrace();

    clock.traceEvent(Clock::TRACE_STEP_FIRE, 0, 4);
    clock.traceEvent(Clock::TRACE_INVALID_STATE, 0, 5);
    clock.traceEvent(Clock::TRACE_STEP_FIRE, 0, 6);

    TEST_ASSERT_TRUE(clock.isTraceFrozen());
    TEST_ASSERT_EQUAL_UINT32(1, clock.getTraceDroppedCount());

    Clock::TraceEvent event;
    TEST_ASSERT_TRUE(clock.popTraceEvent(event));
    TEST_ASSERT_EQUAL_UINT8(Clock::TRACE_STEP_FIRE, event.type);
    TEST_ASSERT_TRUE(clock.popTraceEvent(event));
    TEST_ASSERT_EQUAL_UINT8(Clock::TRACE_INVALID_STATE, event.type);
    TEST_ASSERT_FALSE(clock.popTraceEvent(event));

    clock.clearTrace();
    TEST_ASSERT_FALSE(clock.isTraceFrozen());
    TEST_ASSERT_EQUAL_UINT32(0, clock.getTraceDroppedCount());
}

static void test_position_reset_preserves_callback_and_step_alignment()
{
    using Clock = umodular::clock::uClockClass;
    for (int ppqn : {1, 2, 4, 8, 12, 16, 24}) {
        Clock clock;
        callback_clock = &clock;
        clock.setOutputPPQN(Clock::PPQN_96);
        clock.setInputPPQN(static_cast<Clock::PPQNResolution>(ppqn));
        clock.setOnSync(Clock::PPQN_24, record_sync_24);
        clock.setOnStep(record_step);
        clock.init();
        clock.setStrictExternalMode(true);
        clock.setClockMode(Clock::EXTERNAL_CLOCK);
        clock.start();
        for (uint32_t position : {4608u, 4608u, 6144u, 0u}) {
            const auto state = clock.clock_state;
            const float tempo = clock.getTempo();
            clock.tracks[0].shuffle.current_shff_valid = true;
            clock.tracks[0].shuffle.skip_next_phase_zero = true;
            clock.tracks[0].shuffle.previous_shff = 8;
            clock.ext_interval_buffer[0] = 12345;
            clock.internal_phase_correction_remaining_us = 100;
            sync_24_ticks.clear(); fired_steps.clear();
            TEST_ASSERT_TRUE(clock.resetCountersToPosition(position));
            TEST_ASSERT_EQUAL(state, clock.clock_state);
            TEST_ASSERT_EQUAL(Clock::EXTERNAL_CLOCK, clock.clock_mode);
            TEST_ASSERT_EQUAL_FLOAT(tempo, clock.getTempo());
            TEST_ASSERT_TRUE(sync_24_ticks.empty() && fired_steps.empty());
            TEST_ASSERT_FALSE(clock.tracks[0].shuffle.current_shff_valid);
            TEST_ASSERT_FALSE(clock.tracks[0].shuffle.skip_next_phase_zero);
            TEST_ASSERT_EQUAL_INT(0, clock.tracks[0].shuffle.previous_shff);
            TEST_ASSERT_EQUAL_UINT32(0, clock.ext_interval_buffer[0]);
            TEST_ASSERT_EQUAL_INT(0, clock.internal_phase_correction_remaining_us);
            TEST_ASSERT_EQUAL_UINT32(position, clock.tick);
            TEST_ASSERT_EQUAL_UINT32(position / (96 / ppqn), clock.int_clock_tick);
            TEST_ASSERT_EQUAL_UINT32(clock.int_clock_tick, clock.ext_clock_tick);
            TEST_ASSERT_EQUAL_UINT32(position / 24, clock.tracks[0].step_counter);
            TEST_ASSERT_EQUAL_UINT32(position / 4, clock.sync_callbacks[0].tick);
            TEST_ASSERT_EQUAL_UINT32(0, clock.getExternalTicksRemaining());
            sync_24_ticks.clear(); fired_steps.clear();
            fake_micros += 500000 / ppqn;
            clock.clockMeAt(fake_micros);
            run_until_tick(clock, position + 96 / ppqn);
            TEST_ASSERT_FALSE(sync_24_ticks.empty());
            TEST_ASSERT_FALSE(fired_steps.empty());
            TEST_ASSERT_EQUAL_UINT32(position / 4, sync_24_ticks.front());
            TEST_ASSERT_EQUAL_UINT32(position / 24, fired_steps.front());
        }
        clock.resetCounters();
        TEST_ASSERT_EQUAL_UINT32(0, clock.tick);
        TEST_ASSERT_EQUAL_UINT32(0, clock.sync_callbacks[0].tick);
    }
}

static void test_position_reset_validates_references_and_tracks_fractional_phase()
{
    using Clock = umodular::clock::uClockClass;
    Clock clock;
    TEST_ASSERT_FALSE(clock.resetCountersToPosition(101));
    TEST_ASSERT_EQUAL_UINT32(0, clock.tick);
    clock.setOutputPPQN(Clock::PPQN_96);
    clock.setInputPPQN(Clock::PPQN_24);
    clock.setOnSync(Clock::PPQN_24, record_sync_24);
    clock.setOnStep(record_step);
    clock.init();
    TEST_ASSERT_TRUE(clock.resetCountersToPosition(101));
    TEST_ASSERT_EQUAL_UINT32(25, clock.int_clock_tick);
    TEST_ASSERT_EQUAL_UINT32(1, clock.mod_clock_counter);
    TEST_ASSERT_EQUAL_UINT32(25, clock.sync_callbacks[0].tick);
    TEST_ASSERT_EQUAL_UINT32(1, clock.sync_callbacks[0].mod_counter);
    TEST_ASSERT_EQUAL_UINT32(4, clock.tracks[0].step_counter);
    TEST_ASSERT_EQUAL_UINT32(5, clock.tracks[0].mod_step_counter);
    TEST_ASSERT_EQUAL(Clock::INTERNAL_CLOCK, clock.clock_mode);
    TEST_ASSERT_EQUAL(Clock::STOPPED, clock.clock_state);
    clock.start();
    clock.pause();
    const auto paused = clock.clock_state;
    TEST_ASSERT_TRUE(clock.resetCountersToPosition(101));
    TEST_ASSERT_EQUAL(paused, clock.clock_state);
    clock.mod_clock_ref = 0;
    TEST_ASSERT_FALSE(clock.resetCountersToPosition(0));
    TEST_ASSERT_EQUAL_UINT32(101, clock.tick);
    clock.mod_clock_ref = 4;
    clock.mod_step_ref = 0;
    TEST_ASSERT_FALSE(clock.resetCountersToPosition(0));
    TEST_ASSERT_EQUAL_UINT32(101, clock.tick);
    clock.mod_step_ref = 24;
    clock.sync_callbacks[0].sync_ref = 0;
    TEST_ASSERT_FALSE(clock.resetCountersToPosition(0));
    TEST_ASSERT_EQUAL_UINT32(101, clock.tick);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_position_reset_preserves_callback_and_step_alignment);
    RUN_TEST(test_position_reset_validates_references_and_tracks_fractional_phase);
    RUN_TEST(test_sync_callback_growth_preserves_existing_entries);
    RUN_TEST(test_reduced_ppqn_recovers_counters_above_new_references);
    RUN_TEST(test_output_tick_end_runs_after_sync_and_step_callbacks);
    RUN_TEST(test_strict_external_clock_allows_only_one_output_group_per_pulse);
    RUN_TEST(test_back_to_back_external_pulses_preserve_both_output_groups);
    RUN_TEST(test_strict_external_ppqn_4_pulse_authorizes_quarter_group);
    RUN_TEST(test_strict_external_ppqn_16_pulse_authorizes_six_output_ticks);
    RUN_TEST(test_strict_external_ppqn_1_pulse_authorizes_full_quarter);
    RUN_TEST(test_live_switch_to_ppqn_1_external_clock_keeps_timer_progressing);
    RUN_TEST(test_late_ppqn_1_pulse_does_not_burst_pending_ticks);
    RUN_TEST(test_reentering_external_mode_discards_stale_authorization);
    RUN_TEST(test_reentrant_internal_callback_does_not_advance_twice);
    RUN_TEST(test_external_tempo_ignores_short_startup_interval);
    RUN_TEST(test_timestamped_external_pulses_ignore_delivery_latency);
    RUN_TEST(test_ppqn_1_tempo_tracks_each_new_interval);
    RUN_TEST(test_ppqn_1_phase_slew_aligns_next_quarter_boundary);
    RUN_TEST(test_internal_beat_sync_slews_without_changing_tick_sequence);
    RUN_TEST(test_early_external_pulse_catches_up_without_skipping_callbacks);
    RUN_TEST(test_external_clock_resumes_after_dropout_without_resetting_position);
    RUN_TEST(test_external_tempo_changes_do_not_duplicate_or_skip_sync_ticks);
    RUN_TEST(test_external_continue_preserves_position_and_start_rewinds);
    RUN_TEST(test_mid_cycle_shuffle_activation_does_not_add_a_step);
    RUN_TEST(test_shuffle_length_is_next_offset_minus_current_offset);
    RUN_TEST(test_shuffle_offset_wraps_at_template_size);
    RUN_TEST(test_live_shuffle_update_preserves_latched_step);
    RUN_TEST(test_trace_freezes_after_first_anomaly);
    return UNITY_END();
}