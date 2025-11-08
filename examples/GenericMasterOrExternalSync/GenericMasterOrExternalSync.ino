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
  // setup clock library
  // avaliable output resolutions
  // [ uClock.PPQN_4, uClock.PPQN_8, uClock.PPQN_12, uClock.PPQN_24, uClock.PPQN_48, uClock.PPQN_96, uClock.PPQN_384, uClock.PPQN_480, uClock.PPQN_960 ]
  // not mandatory to call, the default is 96PPQN if not set
  uClock.setOutputPPQN(uClock.PPQN_96);

  // you need to use at least one!
  uClock.setOnOutputPPQN(onOutputPPQNCallback);
  uClock.setOnStep(onStepCallback);
  // multi sync output signatures avaliable
  // normaly used by eurorack modular modules
  uClock.setOnSync(uClock.PPQN_1, onSync1Callback);
  uClock.setOnSync(uClock.PPQN_2, onSync2Callback);
  uClock.setOnSync(uClock.PPQN_4, onSync4Callback);
  // midi sync standard
  uClock.setOnSync(uClock.PPQN_24, onSync24Callback);
  // some korg machines does 48ppqn
  uClock.setOnSync(uClock.PPQN_48, onSync48Callback);

  uClock.setOnClockStart(onClockStartCallback);
  uClock.setOnClockStop(onClockStopCallback);

  // set external sync mode?
  if (_external_sync_on) {
    uClock.setClockMode(uClock.EXTERNAL_CLOCK);
    // what is the clock of incomming signal to sync with?
    // not mandatory to call, the default is 24PPQN if not set
    // avaliable input resolutions -  should be always InputPPQN <= OutputPPQN
    // [ uClock.PPQN_1, uClock.PPQN_2, uClock.PPQN_4, uClock.PPQN_8, uClock.PPQN_12, uClock.PPQN_24, uClock.PPQN_48, uClock.PPQN_96, uClock.PPQN_384, uClock.PPQN_480, uClock.PPQN_960 ]
    uClock.setInputPPQN(uClock.PPQN_24);
  }

  // inits the clock library
  uClock.init();

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
