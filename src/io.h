#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include "hardware/timer.h"
#include "hardware/irq.h"

enum class irq_number_t {
  irq_0 = 0,
  irq_1 = 1,
  irq_2 = 2,
  irq_3 = 3
};

static void onAlarm0();
static void onAlarm1();
static void onAlarm2();
static void onAlarm3();

class alarm_t {
  public:
    alarm_t();
    ~alarm_t();
    config(irq_number_t irq, uint32_t pollFreq, void(*callback)());
    runCallback();
  private:
    irq_number_t _irq;
    bool _active;
    uint32_t _pollFreq;
    void (*_callback)();
};

class scheduler {
  public:
    scheduler();
    ~scheduler();
    alarm_t[4] alarms;   
};

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
