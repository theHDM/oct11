#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

// to do: 
// 2) move synth setup and recall into object
// 3) set up wraparounds for LED display

// sorry, we have to use global variables
// because IRQ requires a static callback
// reference. otherwise i suppose this
// could be a library.

const uint8_t IRQ_poll_rotary  = TIMER_IRQ_2;  // you can change this if you need to free up this timer
const uint8_t IRQ_poll_pinGrid = TIMER_IRQ_3;  // samesies
const uint8_t IRQ_poll_synth   = PWM_IRQ_WRAP;  // if we upgrade to RP2350, may need to update to PWM_IRQ0_WRAP
const uint frequency_poll_rotary = 5;  // set this as high as possible while rotary still reads accurately
const uint frequency_poll_pinGrid = 16;  // set this as low as possible while keys register correctly.

enum {
  btn_off     = 0b00,
  btn_press   = 0b01,
  btn_release = 0b10,
  btn_hold    = 0b11
};

void upd_btn_state(uint8_t &refState, uint8_t press) {
  refState = (3 & ((refState << 1) | (press & 1)));
}

uint64_t runTime() {
  uint64_t temp = timer_hw->timerawh;
  return ((temp << 32) | timer_hw->timerawl);
}

class semaphore_obj {
private:  _state = false;
public:   flag() {_state = true;}
          clear() {_state = false;}
          state() {return _state;}
}

class pinGrid_obj {
private:
  std::vector<uint8_t>   _muxPins;
  std::vector<uint8_t>   _colPins;
  std::size_t         _muxSize;
  std::size_t         _colSize;
  std::vector<uint8_t>   _gridState;
  std::vector<uint16_t>    _outputMap;
  uint8_t             _muxCounter;
  uint8_t             _colCounter;
  uint16_t            _gridCounter;
  uint16_t            _muxMaxValue;
  bool                _readComplete;
  void resetCounter() {
    _readComplete = false;
    _gridCounter = 0;
  }

public:    
  void setup(std::vector<uint8_t> muxPins, 
             std::vector<uint8_t> colPins, 
             std::vector<uint16_t> outputMap) {
    _muxPins = muxPins;
    _muxSize = _muxPins.size();
    _muxMaxValue = (1u << _muxSize);
    _colPins = colPins;
    _colSize = _colPins.size();
    _outputMap = outputMap;
    _gridState.resize(_colSize << _muxSize);
    _muxCounter = 0;
    _colCounter = 0;
    resetCounter();
  }
  void poll() {
    if (!(_readComplete)) 
    {
      upd_btn_state(_gridState[_gridCounter], (digitalRead(_colPins[_colCounter]) == LOW));
      ++_gridCounter;
      _muxCounter = (++_muxCounter) % _muxMaxValue;
      for (uint8_t eachBit = 0; eachBit < _muxSize; eachBit++) 
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
  bool readTo(std::vector<uint8_t> &refTo) {
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

class rotary_obj {
private:
  uint8_t _Apin;
  uint8_t _Bpin;
  uint8_t _Cpin;
  bool _invert;
  uint8_t _turnState;
  uint8_t _clickState;
  int8_t  _turnBuffer;
  uint64_t _lastClickTime;
  std::vector<uint64_t> _clickQueue;
  const uint8_t stateMatrix[7][4] = {
    {0,4,1,0},
    {2,0,1,0},{2,3,1,0},{2,3,0,8},
    {5,4,0,0},{5,4,6,0},{5,0,6,16}
  };
public:
  void setup(uint8_t Apin, uint8_t Bpin, uint8_t Cpin) {
    _Apin = Apin;
    _Bpin = Bpin;
    _Cpin = Cpin;
    pinMode(_Apin, INPUT_PULLUP);
    pinMode(_Bpin, INPUT_PULLUP);
    pinMode(_Cpin, INPUT_PULLUP);
    _invert = false;
    _turnState = 0;
    _clickState = 0;
    _turnBuffer = 0;
    _lastClickTime = 0;
    _clickQueue.clear();
  }
  void invertDirection(bool invert) {_invert = invert;}
  void poll() {
    uint8_t A = digitalRead(_Apin);
    uint8_t B = digitalRead(_Bpin);
    uint8_t getRotation = (_invert ? ((A << 1) | B) : ((B << 1) | A));
    _state = stateMatrix[_state & 3][getRotation];
    _turnBuffer += (_state & 8) >> 3;
    _turnBuffer -= (_state & 16) >> 4;
    upd_btn_state(_clickState, digitalRead(_Cpin));
    switch (_clickState) { 
      case btn_press:
        _lastClickTime = runTime();
        break;
      case btn_release;
        _clickQueue.emplace(runTime() - _lastClickTime);
        break;
      default:
        break;
    }
  }
  int8_t getTurnFromBuffer() {
    if (_turnBuffer) {
      int8_t oneTurn = ((_turnBuffer > 0) ? 1 : -1);
      _turnBuffer -= oneTurn;
      return oneTurn;
    }
    return 0;
  }
  uint64_t getClickFromBuffer() {
    if (!(_clickQueue.empty())) {
      return _clickQueue.pop();
    }
    return 0;
  }
};

class ringBuffer_obj {
private:
  std::vector<uint8_t> _data;
  uint _size;
  uint _readPointer;
  uint _writePointer;
  uint _freeBytes;
public:
  ringBuffer_obj(uint size) {
    _size = size;
    _data.resize(_size);
    _readPointer = 0;
    _writePointer = 0;
    _freeBytes = _size;
  }
  uint8_t read() {
    if (_freeBytes == _size) {
      return 0;
    }
    uint8_t x = _data[_readPointer];
    _readPointer = _readPointer++ % _size;
    _freeBytes++;
    return x;
  }
  bool write(uint8_t x) {
    if (_freeBytes == 0) {
      return false;
    }
    _data[_writePointer] = x;
    _writePointer = _writePointer++ % _size;
    _freeBytes--;
    return true;
  }
  uint getSize() {
    return _size;
  }
  uint getFree() {
    return _freeBytes;
  }
};

class audioOut_obj {
private:
  uint8_t _sampleClockPin;
  std::vector<uint8_t> _pwmPins;
  std::vector<uint8_t> _analogPins; 
public:
  void setup_48kHz_clock(uint8_t pin) {
    // given sample Hz = 48000, crystal oscillator = 12mHz
    // using PLL function (datasheet 2.18)
    // cap clock speed at F_CPU = 133mHz
    // FB = 102, POSTDIV1 = 5, POSTDIV2 = 2
    // VCO freq = 1224 mHz, clock speed = 122.4 mHz
    // exactly 2550 CPU cycles in 1/48000 of a second
    // each sample PWM loops (510 CPU cycles) exactly 5 times
    // audio jitter should be almost non-existent
    // !!!
    set_sys_clock_pll(12'000'000 * 102, 5, 2);
    uint8_t slice = pwm_gpio_to_slice_num(pin);
    gpio_set_function(pin, GPIO_FUNC_PWM);      // set that pin as PWM
    pwm_set_phase_correct(slice, false);    
    pwm_set_wrap(slice, 2449);                  
    pwm_set_clkdiv(slice, 1.0f);                  // run at full clock speed
    pwm_set_gpio_level(pin, 0);                   // initialize at zero to prevent whining sound
    pwm_set_irq_enabled (slice, true);            // every loop of the timer, trigger the PWM interrupt
    pwm_set_enabled(slice, true);                 // ENGAGE!
  }
  void setup_audio_feed(uint8_t pin, bool isAnalog) {
    if (!isAnalog) {
      _pwmPins.push(pin);
      // synthesize 8-bit sound
      // PWM_TOP = 254, with phase correct. Loop every (254 + 1) * 2 = 510 cycles.
      uint8_t slice = pwm_gpio_to_slice_num(pin);
      gpio_set_function(pin, GPIO_FUNC_PWM);      // set that pin as PWM
      pwm_set_phase_correct(slice, true);           
      pwm_set_wrap(slice, 254);                     
      pwm_set_clkdiv(slice, 1.0f);                  // run at full clock speed
      pwm_set_gpio_level(pin, 0);              // initialize at zero to prevent whining sound
      pwm_set_irq_enabled (slice, false);     // do not trigger the PWM interrupt.
      pwm_set_enabled(slice, true);                 // ENGAGE!
    } else {
      _analogPins.push(pin);
    }
  }
  void send(uint8_t lvl) {
    for (auto i : _pwmPins) {
      pwm_set_gpio_level(i, lvl);
    }
    for (auto j : _analogPins) {
      analogWrite(j, lvl);
    }
  }
};

// establish global instances

pinGrid_obj pinGrid;
rotary_obj  rotary;
ringBuffer_obj synthBuf(1024);
audioOut_obj audioOut;

static void on_poll_rotary() {
  hw_clear_bits(&timer_hw->intr, 1u << IRQ_poll_rotary);   // clear the interrupt
  timer_hw->alarm[IRQ_poll_rotary] = runTime() + frequency_poll_rotary;  // repeat every few microseconds
  rotary.poll();                      // update the state of the rotary knob
}

static void on_poll_pinGrid() {
  hw_clear_bits(&timer_hw->intr, 1u << IRQ_poll_pinGrid);   // clear the interrupt
  timer_hw->alarm[IRQ_poll_pinGrid] = runTime() + frequency_poll_pinGrid;  // repeat every few microseconds
  pinGrid.poll();                        // read the next set of pins
}

static void on_poll_synth() {
  hw_clear_bits(&timer_hw->intr, 1u << IRQ_poll_synth);   // clear the interrupt
  audioOut.send(synthBuf.read());          // write the next sample to all audio outs
}

void setup_rotary_interrupt() {
  hw_set_bits(&timer_hw->inte, 1u << IRQ_poll_rotary);   // initialize the timer  
  timer_hw->alarm[IRQ_poll_rotary] = runTime() + 10'000;   // wait like 10ms until setup complete
  irq_set_exclusive_handler(IRQ_poll_rotary, on_poll_rotary);
  irq_set_enabled(IRQ_poll_rotary, true);               // ENGAGE!
}

void setup_pinGrid_interrupt() {
  hw_set_bits(&timer_hw->inte, 1u << IRQ_poll_pinGrid);   // initialize the timer  
  timer_hw->alarm[IRQ_poll_pinGrid] = runTime() + 10'000;   // wait like 10ms until setup complete
  irq_set_exclusive_handler(IRQ_poll_pinGrid, on_poll_pinGrid);
  irq_set_enabled(IRQ_poll_pinGrid, true);               // ENGAGE!
}

void setup_synth_interrupt() {
  hw_set_bits(&timer_hw->inte, 1u << IRQ_poll_synth);   // initialize the timer    
  irq_set_exclusive_handler(IRQ_poll_synth,   on_poll_synth);
  irq_set_enabled(IRQ_poll_synth, true);               // ENGAGE!
}
