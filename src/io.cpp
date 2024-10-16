#include "io.h"

/*
  class scheduler {
  public:
    scheduler();
    void setAlarm(irq_number_t IRQn, uint32_t pollFreq, void(*callback)());
    void endAlarm(irq_number_t IRQn);
    static void onAlarm0();
    static void onAlarm1();
    static void onAlarm2();
    static void onAlarm3();
  private:
    std::vector<bool> _isActive;
    std::vector<uint32_t> _pollFreq;
    std::vector<void*> _callbackPtr;
    void onAlarm(irq_number_t IRQn);
}
*/

scheduler::scheduler() {
  _isActive = {0,0,0,0};
}

void scheduler::setAlarm(irq_number_t IRQn, uint32_t pollFreq, void(*callback)()) {
  _isActive[IRQn] = 1;
  _pollFreq[IRQn] = pollFreq;
  _callbackPtr[IRQn] = callback;
  hw_set_bits(&timer_hw->inte, 1u << IRQn);   // initialize the timer
  switch (IRQn) {
    case IRQ_num_0:
      irq_set_exclusive_handler(IRQn, scheduler::onAlarm0());
      break;
    case IRQ_num_1:
      irq_set_exclusive_handler(IRQn, scheduler::onAlarm1());
      break;
    case IRQ_num_2:
      irq_set_exclusive_handler(IRQn, scheduler::onAlarm2());
      break;
    case IRQ_num_3:
      irq_set_exclusive_handler(IRQn, scheduler::onAlarm3());
      break;
    default:
      irq_set_exclusive_handler(IRQn, nullptr);
      break;
  }
  irq_set_enabled(IRQn, true);               // ENGAGE!
}

void scheduler::endAlarm(irq_number_t IRQn) {
  hw_clear_bits(&timer_hw->inte, 1u << IRQn);
  irq_set_enabled(IRQn, false); 
}

void scheduler::resetAlarm(irq_number_t IRQn) {
  hw_clear_bits(&timer_hw->intr, 1u << IRQn);   // initialize the timer
  uint64_t temp = timer_hw->timerawh;
  timer_hw->alarm[IRQn] = ((temp << 32) | timer_hw->timerawl) + _pollFreq[IRQn];
}

void scheduler::onAlarm(irq_number_t IRQn) {
  _callbackPtr[IRQn]();
}

static void scheduler::onAlarm0() {onAlarm(0);}
static void scheduler::onAlarm1() {onAlarm(1);}
static void scheduler::onAlarm2() {onAlarm(2);}
static void scheduler::onAlarm3() {onAlarm(3);}

pinGrid::pinGrid(
  std::vector<byte> muxPins, 
  std::vector<byte> colPins, 
  std::vector<int> outputMap
) {
  _muxPins = muxPins;
  _colPins = colPins;
  _outputMap = outputMap;
  _muxSize = _muxPins.size();
  _muxMaxValue = (1u << _muxSize);
  _colSize = _colPins.size();
}

void pinGrid::begin(byte timerIRQ, uint32_t pollFreq) {
  _gridState.resize(_colSize << _muxSize);
  _muxCounter = 0;
  _colCounter = 0;

  _timerIRQ = timerIRQ;
  _pollFreq = pollFreq;


  resetCounterAndTimer();
}

bool pinGrid::pull(std::vector<state_t>& refTo) {
  if (_readComplete) {
    for (size_t eachEntry = 0; eachEntry < _outputMap.size(); eachEntry++) {
      refTo[_outputMap[eachEntry]] = _gridState[eachEntry];
    }
    resetCounterAndTimer();
    return true;
  } else {
    return false;
  }
}

void pinGrid::onPoll() {
  if (!(_readComplete)) {
    resetTimer();
    _gridState[_gridCounter] = (3 & (
        (_gridState[_gridCounter] << 1)
      + (digitalRead(_colPins[_colCounter]) == LOW)
    ));
    ++_gridCounter;
    _muxCounter = (++_muxCounter) % _muxMaxValue;
    for (byte eachBit = 0; eachBit < _muxSize; eachBit++) {
      digitalWrite(_muxPins[eachBit], (_muxCounter >> eachBit) & 1);
    }
    if (!(_muxCounter)) {
      pinMode(_colPins[_colCounter], INPUT);        // Set the selected column pin back to INPUT mode (0V / LOW).
      _colCounter = (_colCounter + 1) % _colSize;
      pinMode(_colPins[_colCounter], INPUT_PULLUP); // Set that column pin to INPUT_PULLUP mode (+3.3V / HIGH).
      _readComplete = !(_colCounter);  // if _colcounter and _muxcounter at zero
    }
  }
}

void pinGrid::resetCounterAndTimer() {
  _readComplete = false;
  _gridCounter = 0;
  resetTimer();
}

