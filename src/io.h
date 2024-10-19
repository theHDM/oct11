#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include "hardware/timer.h"
#include "hardware/irq.h"

// sorry, we have to use global variables
// because IRQ requires a static callback
// reference. otherwise i suppose this
// could be a library.

enum class assign_IRQ {
  poll_synth = 2,
  poll_pinGrid = 3
};
enum class btn_state {
  off     = 0b00,
  press   = 0b01,
  release = 0b10,
  hold    = 0b11
};



// setup grid
// begin grid
// on poll
// on read complete


class pinGrid {
  private:
    std::vector<byte>       _muxPins;
    std::vector<byte>       _colPins;
    std::size_t             _muxSize;
    std::size_t             _colSize;
    std::vector<btn_state>  _gridState;
    std::vector<int>        _outputMap;
    byte                    _muxCounter;
    byte                    _colCounter;
    byte                    _gridCounter;
    int                     _muxMaxValue;
    bool                    _readComplete;

    void resetCounter();

  public:
    pinGrid(
      std::vector<byte> muxPins, 
      std::vector<byte> colPins, 
      std::vector<int> outputMap
    ) {
      _muxPins = muxPins;
      _muxSize = _muxPins.size();
      _muxMaxValue = (1u << _muxSize);
      _colPins = colPins;
      _colSize = _colPins.size();
      _outputMap = outputMap;
    };
    void begin(
      byte irq, 
      uint32_t pollFreq
    ) {
      _gridState.resize(_colSize << _muxSize);
      _muxCounter = 0;
      _colCounter = 0;



      scheduler::alarms[irq].setup(irq,pollFreq,this->onPoll);
      // resetCounterAndTimer();
      resetCounter();
    }
    void onPoll();
    bool pull(std::vector<state_t>& refTo);
};


void pinGrid::onPoll() {
  if (!(_readComplete)) {
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
      _colCounter = (++_colCounter) % _colSize;
      pinMode(_colPins[_colCounter], INPUT_PULLUP); // Set that column pin to INPUT_PULLUP mode (+3.3V / HIGH).
      _readComplete = !(_colCounter);  // if _colcounter and _muxcounter at zero
    }
  }
}

bool pinGrid::pull(std::vector<state_t>& refTo) {
  if (_readComplete) {
    for (size_t eachEntry = 0; eachEntry < _outputMap.size(); eachEntry++) {
      refTo[_outputMap[eachEntry]] = _gridState[eachEntry];
    }
    // resetCounterAndTimer();
    resetCounter();
    return true;
  } else {
    return false;
  }
}

void pinGrid::resetCounter() {
  _readComplete = false;
  _gridCounter = 0;
}


class rotary {
  public:
    rotary(
      byte _Apin, 
      byte _Bpin, 
      byte _Cpin, 
      byte _bufferLimit
    ); // declare constructor

    void invertDirection(); // declare function to swap A/B pins
    void update();
    int  getTurnFromBuffer(); // positive = counterclockwise
    int  getClick();
    int  getValueInTurnBuffer();
    byte getApin();
    byte getBpin();
    byte getCpin();
    int  getKnobState();

  private:
    int  turnBuffer;
    int  clickBuffer;
    byte Apin;
    byte Bpin;
    byte Cpin;
    byte state;
    byte press;
    bool clicked;
    bool bufferTurns;
};




alarm_t::alarm_t() {
  _active = true;
}

alarm_t::~alarm_t() {
  irq_set_enabled(_irq, false);
  irq_set_exclusive_handler(_irq, nullptr);
  hw_clear_bits(&timer_hw->inte, 1u << _irq);
  _active = false;
}

void alarm_t::setup(byte irq, uint32_t pollFreq, void(*callback)()) {
  _irq = irq;
  _pollFreq = pollFreq;
  _callback = callback;
}

void alarm_t::runCallback() {
  hw_clear_bits(&timer_hw->intr, 1u << _irq);   // clear the interrupt
  uint64_t temp = timer_hw->timerawh;
  timer_hw->alarm[_irq] = ((temp << 32) | timer_hw->timerawl) + _pollFreq;
  _callback();
}

void setup(byte irq, uint32_t pollFreq, void(*callback)()) {
  alarms[irq].setup(irq, pollFreq, callback);
  hw_set_bits(&timer_hw->inte, 1u << irq);   // initialize the timer  
  irq_set_enabled(irq, true);               // ENGAGE!
}


static void on_poll_synth() {

}

static void on_poll_pinGrid() {

}

void setupIRQs() {
  irq_set_exclusive_handler(assign_IRQ::poll_synth,   on_poll_synth);
  irq_set_exclusive_handler(assign_IRQ::poll_pinGrid, on_poll_pinGrid);
}
