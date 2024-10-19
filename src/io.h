#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

// sorry, we have to use global variables
// because IRQ requires a static callback
// reference. otherwise i suppose this
// could be a library.

enum class assign_IRQ
{
  poll_synth = 2,
  poll_pinGrid = 3
};

enum class btn_state
{
  off     = 0b00,
  press   = 0b01,
  release = 0b10,
  hold    = 0b11
};

uint64_t runTime() 
{
  uint64_t temp = timer_hw->timerawh;
  return ((temp << 32) | timer_hw->timerawl);
}

class pinGrid
{
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
  void resetCounter() 
  {
    _readComplete = false;
    _gridCounter = 0;
  }

public:    
  pinGrid
  (
    std::vector<byte> muxPins, 
    std::vector<byte> colPins, 
    std::vector<int> outputMap
  )
  {
    _muxPins = muxPins;
    _muxSize = _muxPins.size();
    _muxMaxValue = (1u << _muxSize);
    _colPins = colPins;
    _colSize = _colPins.size();
    _outputMap = outputMap;
  };

  void begin() 
  {
    _gridState.resize(_colSize << _muxSize);
    _muxCounter = 0;
    _colCounter = 0;
    resetCounter();
  }

  void poll() 
  {
    if (!(_readComplete)) 
    {
      _gridState[_gridCounter] = (3 & ((_gridState[_gridCounter] << 1)
        + (digitalRead(_colPins[_colCounter]) == LOW)));
      ++_gridCounter;
      _muxCounter = (++_muxCounter) % _muxMaxValue;
      for (byte eachBit = 0; eachBit < _muxSize; eachBit++) 
      {
        digitalWrite(_muxPins[eachBit], (_muxCounter >> eachBit) & 1);
      }
      if (!(_muxCounter)) 
      {
        pinMode(_colPins[_colCounter], INPUT);        // 0V / LOW
        _colCounter = (++_colCounter) % _colSize;
        pinMode(_colPins[_colCounter], INPUT_PULLUP); // +3.3V / HIGH
        _readComplete = !(_colCounter);  // complete when _colcounter and _muxcounter at zero
      }
    }
  }

  bool read
  (
    std::vector<state_t>& refTo
  ) 
  {
    if (_readComplete) 
    {
      for (size_t i = 0; i < _outputMap.size(); i++) 
      {
        refTo[_outputMap[i]] = _gridState[i];
      }
      resetCounter();
      return true;
    } else {
      return false;
    }
  }
};

class rotary 
{
private:
  int  _turnBuffer;
  int  _clickBuffer;
  byte _Apin;
  byte _Bpin;
  byte _Cpin;
  byte _state;
  byte _press;
  bool _clicked;
  bool _bufferTurns;
public:
  rotary(
    byte rotaryPins[3]
  ) 
  {
    _Apin = rotaryPins[0];
    _Bpin = rotaryPins[1];
    _Cpin = rotaryPins[2];
  }
  void invertDirection(); // declare function to swap A/B pins
  void update();
  int  getTurnFromBuffer(); // positive = counterclockwise
  int  getClick();
  int  getValueInTurnBuffer();
  byte getApin();
  byte getBpin();
  byte getCpin();
  int  getKnobState();
}

// synth
// given sample Hz = 44100
// given maximum F_CPU = 125000000
// F_CPU / Sample = X
// round down to nearest 2^N 
// 44100 x 2^N is PWM clock.
// rounddown(F_CPU * 16 / PWM) = multiplier (in 16ths)
// set CPU clock = PWM * multiplier / 16

//void setupSynth(byte pin, byte slice) 
//{
//  gpio_set_function(pin, GPIO_FUNC_PWM);      // set that pin as PWM
//  pwm_set_phase_correct(slice, true);           // phase correct sounds better
//  pwm_set_wrap(slice, 254);                     // 0 - 254 allows 0 - 255 level
//  pwm_set_clkdiv(slice, 1.0f);                  // run at full clock speed
//  pwm_set_chan_level(slice, PIEZO_CHNL, 0);        // initialize at zero to prevent whining sound
//  pwm_set_enabled(slice, true);                 // ENGAGE!
//  hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);  // initialize the timer
//  irq_set_exclusive_handler(ALARM_IRQ, poll);     // function to run every interrupt
//  irq_set_enabled(ALARM_IRQ, true);               // ENGAGE!
//  uint64_t temp = timer_hw->timerawh;
//  timer_hw->alarm[ALARM_NUM] = ((temp << 32) | timer_hw->timerawl) + POLL_INTERVAL_IN_MICROSECONDS;
//  resetSynthFreqs();
//  sendToLog("synth is ready.");
//}

// void poll() {
//   pwm_set_chan_level(slice, channel, nextBuffer);
// }

// establish global instances

pinGrid pinGrid_instance(muxPins, colPins, mapGridToPixel);
rotary  rotary_instance(rotaryPins);
// synth object declare

void snooze(byte irq, uint64_t uS) {
  hw_clear_bits(&timer_hw->intr, 1u << irq);   // clear the interrupt
  timer_hw->alarm[irq] = runTime() + uS;
}

static void on_poll_pinGrid() {
  snooze(assign_IRQ::poll_pinGrid, 16);
  pinGrid_instance.poll();
}

static void on_poll_synth() {
  snooze(assign_IRQ::poll_synth, 24);
//  synth_instance.poll();
}

void setup_pinGrid() {
  pinGrid_instance.begin();
  hw_set_bits(&timer_hw->inte, 1u << assign_IRQ::poll_pinGrid);   // initialize the timer  
  irq_set_exclusive_handler(assign_IRQ::poll_pinGrid, on_poll_pinGrid);
  irq_set_enabled(assign_IRQ::poll_pinGrid, true);               // ENGAGE!
}

void setup_synth() {
// synth_instance.begin();
  hw_set_bits(&timer_hw->inte, 1u << assign_IRQ::poll_synth);   // initialize the timer    
  irq_set_exclusive_handler(assign_IRQ::poll_synth,   on_poll_synth);
  irq_set_enabled(assign_IRQ::poll_synth, true);               // ENGAGE!
}
