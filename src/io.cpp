#include "io.h"

void alarm_t::alarm_t() {_active = true;}

void alarm_t::~alarm_t() {
  irq_set_enabled(_irq, false);
  irq_set_exclusive_handler(_irq, nullptr);
  hw_clear_bits(&timer_hw->inte, 1u << _irq);
}

void alarm_t::config(irq_number_t irq, uint32_t pollFreq, void(*callback)()) {
  _irq = irq;
  _pollFreq = pollFreq;
  _callback = callback;
  hw_set_bits(&timer_hw->inte, 1u << _irq);   // initialize the timer  
  switch (_irq) {
    case irq_0: irq_set_exclusive_handler(_irq, onAlarm0); break;
    case irq_1: irq_set_exclusive_handler(_irq, onAlarm1); break;
    case irq_2: irq_set_exclusive_handler(_irq, onAlarm2); break;
    case irq_3: irq_set_exclusive_handler(_irq, onAlarm3); break;
    default:    irq_set_exclusive_handler(_irq, nullptr);  break;
  }
  irq_set_enabled(_irq, true);               // ENGAGE!
}

void alarm_t::runCallback() {
  hw_clear_bits(&timer_hw->intr, 1u << _irq);   // initialize the timer
  uint64_t temp = timer_hw->timerawh;
  timer_hw->alarm[_irq] = ((temp << 32) | timer_hw->timerawl) + _pollFreq;
  _callback();
}

static void scheduler::onAlarm0() {alarms[0].runCallback();}
static void scheduler::onAlarm1() {alarms[1].runCallback();}
static void scheduler::onAlarm2() {alarms[2].runCallback();}
static void scheduler::onAlarm3() {alarms[3].runCallback();}

/*
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include "hardware/timer.h"
#include "hardware/irq.h"

enum class state_t {
  btn_off     = 0b00,
  btn_press   = 0b01,
  btn_hold    = 0b11,
  btn_release = 0b10
};

class pinGrid {
  public:
    pinGrid(
      std::vector<byte> muxPins,
      std::vector<byte> colPins, 
      std::vector<int> outputMap
    );
    void begin(
      irq_number_t timerIRQ, 
      uint32_t pollFreq
    );
    bool pull(std::vector<state_t>& refTo);

  private:
    std::vector<byte> _muxPins;
    std::vector<byte> _colPins;
    std::size_t _muxSize;
    std::size_t _colSize;
    std::vector<state_t> _gridState;
    std::vector<int> _outputMap;
    byte _muxCounter;
    byte _colCounter;
    byte _gridCounter;
    int _muxMaxValue;
    bool _readComplete;
    void onPoll();
    void resetCounterAndTimer();
    void resetTimer();
};

*/

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

