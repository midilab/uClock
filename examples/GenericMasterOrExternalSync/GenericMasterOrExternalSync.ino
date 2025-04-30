#include <uClock.h>

// external or internal sync?
bool _external_sync_on = false;

// the main uClock PPQN resolution ticking
void onOutputPPQNCallback(uint32_t tick) {
  // tick your sequencers or tickable devices...
}

void onStepCallback(uint32_t step) {
  // triger step data for sequencer device...
}

// The callback function called by uClock each Pulse of 1PPQN clock resolution.
void onSync1Callback(uint32_t tick) {
  // send sync signal to...
}

// The callback function called by uClock each Pulse of 2PPQN clock resolution.
void onSync2Callback(uint32_t tick) {
  // send sync signal to...
}

// The callback function called by uClock each Pulse of 4PPQN clock resolution.
void onSync4Callback(uint32_t tick) {
  // send sync signal to...
}

// The callback function called by uClock each Pulse of 24PPQN clock resolution.
void onSync24Callback(uint32_t tick) {
  // send sync signal to...
}

// The callback function called by uClock each Pulse of 48PPQN clock resolution.
void onSync48Callback(uint32_t tick) {
  // send sync signal to...
}

// The callback function called when clock starts by using uClock.start() method.
void onClockStartCallback() {
  // send start signal to...
}

// The callback function called when clock stops by using uClock.stop() method.
void onClockStopCallback() {
  // send stop signal to...
}

void setup() {

  // inits the clock library
  uClock.init();

  // avaliable output PPQN resolutions for this example
  // [ uClock.PPQN_48, uClock.PPQN_96, uClock.PPQN_384, uClock.PPQN_480, uClock.PPQN_960 ]
  // not mandatory to call, the default is 96PPQN if not set
  uClock.setOutputPPQN(uClock.PPQN_96);

  // you need to use at least one!
  uClock.setOnOutputPPQN(onOutputPPQNCallback);
  uClock.setOnStep(onStepCallback);
  // multi sync output signatures avaliable
  // normaly used by eurorack modular modules
  uClock.setOnSync1(onSync1Callback);
  uClock.setOnSync2(onSync2Callback);
  uClock.setOnSync4(onSync4Callback);
  // midi sync standard
  uClock.setOnSync24(onSync24Callback);
  // some korg machines do 48ppqn
  uClock.setOnSync48(onSync48Callback);

  uClock.setOnClockStart(onClockStartCallback);
  uClock.setOnClockStop(onClockStopCallback);

  // set external sync mode?
  if (_external_sync_on) {
    uClock.setClockMode(uClock.EXTERNAL_CLOCK);
  }

  // starts clock
  uClock.start();
}

void loop() {
  // do we need to external sync?
  if (_external_sync_on) {
    // watch for external sync signal income
    bool signal_income = true; // your external input signal check will be this condition result
    if (signal_income) {
      // at each clockMe call uClock will process and handle external/internal syncronization
      uClock.clockMe();
    }
  }
}