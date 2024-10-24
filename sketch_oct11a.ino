#include "src/config.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <Wire.h>               // this is necessary to deal with the pins and wires
#include <GEM_u8g2.h>           // library of code to create menu objects on the B&W display
#include <vector>
#include "src/io.h"

/*
U8G2_SH1107_SEEED_128X128_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);
File settingsFile;
*/

// establish global instances
std::vector<uint8_t> hex;

void setup() {
  /*
  Wire.setSDA(OLED_sdaPin);
  Wire.setSCL(OLED_sclPin);
  u8g2.begin();                       // Menu and graphics setup
  u8g2.setBusClock(1000000);          // Speed up display
  u8g2.setContrast(63);   // Set contrast
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_4x6_tf);
  // put your setup code here, to run once:
  LittleFSConfig cfg;       // Configure file system defaults
  cfg.setAutoFormat(true);  // Formats file system if it cannot be mounted.
  LittleFS.setConfig(cfg);
  LittleFS.begin();         // Mounts file system.
  if (!LittleFS.begin()) {
    u8g2.drawStr(20,20,"FS not mounted");
  } else {
    u8g2.drawStr(20,20,"FS mounted");
  }
  settingsFile = LittleFS.open("settings.dat","r+");
  if (!settingsFile) {
    settingsFile = LittleFS.open("settings.dat","w+");
    u8g2.drawStr(20,30,"W+'d");      
  } else {
    u8g2.drawStr(20,30,"R+'d");      
  }
  u8g2.sendBuffer();
  settingsFile.printf("testing 123");
  settingsFile.close();
*/
 

  hex.resize(colPins.size() << muxPins.size());

}

void loop() {
  if (pinGrid.readTo(hex)) {
    // interpret grid info
  }

  // if ready for menu, do GLCD stuff


}

void setup1() {
  setup_pinGrid(muxPins,colPins,mapGridToPixel);
  setup_rotary(rotaryPinA,rotaryPinB,rotaryPinC);
  setup_synth(pwmPins,adcPins,emptyPinForCycleCounting);
}

void loop1() {
  0;
}

void loop1() {
  
}
