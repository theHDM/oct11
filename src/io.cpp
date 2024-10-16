#include "io.h"





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


  hw_set_bits(&timer_hw->inte, 1u << _timerIRQ);   // initialize the timer
  irq_set_exclusive_handler(_timerIRQ, onPoll); // function to run every interrupt
  irq_set_enabled(_timerIRQ, true);               // ENGAGE!
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

void pinGrid::resetTimer() {
  hw_clear_bits(&timer_hw->intr, 1u << _timerIRQ);   // initialize the timer
  uint64_t temp = timer_hw->timerawh;
  timer_hw->alarm[_timerIRQ] = ((temp << 32) | timer_hw->timerawl) + _pollFreq;
}