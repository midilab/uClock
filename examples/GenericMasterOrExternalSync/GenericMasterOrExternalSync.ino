#include <uClock.h>

// external or internal sync?
bool _external_sync_on = false;

// the main uClock PPQN resolution ticking
void onPPQNCallback(uint32_t tick) {
  // tick your sequencers or tickable devices...
}

void onStepCallback(uint32_t step) {
  // triger step data for sequencer device...
}

// The callback function called by uClock each Pulse of 24PPQN clock resolution.
void onSync24Callback(uint32_t tick) {
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

  // avaliable resolutions
  // [ uClock.PPQN_24, uClock.PPQN_48, uClock.PPQN_96, uClock.PPQN_384, uClock.PPQN_480, uClock.PPQN_960 ]
  // not mandatory to call, the default is 96PPQN if not set
  uClock.setPPQN(uClock.PPQN_96);

  // you need to use at least one!
  uClock.setOnPPQN(onPPQNCallback);
  uClock.setOnStep(onStepCallback);
  uClock.setOnSync24(onSync24Callback);

  uClock.setOnClockStart(onClockStartCallback);
  uClock.setOnClockStop(onClockStopCallback);

  // set external sync mode?
  if (_external_sync_on) {
    uClock.setMode(uClock.EXTERNAL_CLOCK);
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