#pragma once
#include <Arduino.h>
#include <vector>

const std::vector<uint8_t> muxPins = {4,5,2,3};
const std::vector<uint8_t> colPins = {6,7,8,9,10,11,12,13,14,15};
const std::vector<uint16_t> mapGridToPixel = {
  /* COL PIN  MUX value:
   index   0  1  2  3  4  5  6  7  8  9  A   B   C   D   E   F */
  /* 0 */  0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,
  /* 1 */  1,11,21,31,41,51,61,71,81,91,101,111,121,131,141,151,
  /* 2 */  2,12,22,32,42,52,62,72,82,92,102,112,122,132,142,152,
  /* 3 */  3,13,23,33,43,53,63,73,83,93,103,113,123,133,143,153,
  /* 4 */  4,14,24,34,44,54,64,74,84,94,104,114,124,134,144,154,
  /* 5 */  5,15,25,35,45,55,65,75,85,95,105,115,125,135,145,155,
  /* 6 */  6,16,26,36,46,56,66,76,86,96,106,116,126,136,146,156,
  /* 7 */  7,17,27,37,47,57,67,77,87,97,107,117,127,137,147,157,
  /* 8 */  8,18,28,38,48,58,68,78,88,98,108,118,128,138,148,158,
  /* 9 */  9,19,29,39,49,59,69,79,89,99,109,119,129,139,149,159
};
const uint8_t OLED_sdaPin = 16;
const uint8_t OLED_sclPin = 17;
const uint8_t rotaryPinA = 20;
const uint8_t rotaryPinB = 21;
const uint8_t ledPin = 22;
const uint8_t rotaryPinC = 24;
const std::vector<uint8_t> adcPins = {};      // audio can use analog write on pins 26-29
const std::vector<uint8_t> pwmPins = {23,25}; // audio must use PWM / digital write on all other pins
const uint8_t emptyPinForCycleCounting = 29;
