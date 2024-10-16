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

class scheduler {
  public:
    scheduler();
    void setAlarm(byte IRQn, uint32_t pollFreq, void *callback);
    static void onAlarm0();
    static void onAlarm1();
    static void onAlarm2();
    static void onAlarm3();
  private:


}


class pinGrid {
  public:
    pinGrid(
      std::vector<byte> muxPins,
      std::vector<byte> colPins, 
      std::vector<int> outputMap
    );
    void begin(
      byte timerIRQ, 
      uint32_t pollFreq
    );
    bool pull(std::vector<state_t>& refTo);

  private:
    std::vector<byte> _muxPins;
    std::vector<byte> _colPins;
    byte _timerIRQ;
    uint32_t _pollFreq;
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
