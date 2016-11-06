//--------------------------------------------------------------------------------
// EDm_BLE, v0.1.0
// Retrieve battery diagnostic data from your smart electric drive EV.
//
// (c) 2016 by MyLab-odyssey
//
// Licensed under "MIT License (MIT)", see license file for more information.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER OR CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//--------------------------------------------------------------------------------
//! \file    EDm_BLE.ino
//! \brief   Retrieve data from your smart electric drive EV.
//! \brief   Using a RFduino micro controller and the Bynk app for display
//! \brief   www.rfduino.com, www.blynk.cc
//! \date    2016-November
//! \author  MyLab-odyssey
//! \version 0.1.0
//--------------------------------------------------------------------------------

//#define BLYNK_DEBUG
//#define BLYNK_PRINT Serial

#define BLYNK_USE_DIRECT_CONNECT
//#define BLYNK_MAX_SENDBYTES 256 // Default is 128

#include <BlynkSimpleRFduinoBLE.h>
#include <RFduinoBLE.h>
#include <String.h>
#include "canDiag.h"

#define CS     10                //!< chip select pin of MCP2515 CAN-Controller
#define CS_SD  8                 //!< CS for SD card, if you plan to use a logger...
MCP_CAN CAN0(SS);                //!< Set CS pin

canDiag DiagCAN;
BatteryDiag_t BMS;
ChargerDiag_t NLG6;
CoolingSub_t CLS;
DriveStats_t DRV;

CTimeout CAN_Timeout(1000);     //!< Timeout value for CAN response in millis
CTimeout Terminal_Timeout(1000);
CTimeout DEVstatus_Timeout(3000);

CTimeout DRV_Timeout(5000);
CTimeout BMS_Timeout(2500);

//#define BLYNK_SEND_THROTTLE 20

WidgetLED LEDcon(V0);
//WidgetTerminal terminal(V12);

//unsigned long time = millis();
unsigned long IOcount = 0;
unsigned int delta = 0;

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "put your auth token here";

int state = 0;
int rssi = 0;
boolean fConnected = false;
byte taskSOC = 1;
byte taskPower = 1;
boolean fOK_SOC = false;
boolean fOK_Power = false;
byte failCount = 0;

int tab_index = 0;

//--------------------------------------------------------------------------------
//! \brief   SETUP()
//--------------------------------------------------------------------------------
void setup() {
  //Serial.begin(9600);
  //while(!Serial);

  //LED CAN-Bus active
  pinMode(0, OUTPUT);
  //LED BLE connected
  pinMode(1, OUTPUT);

  // MCP2515 read buffer: setting pin 2 for input, LOW if CAN messages are received
  pinMode(2, INPUT);

  // Initialize MCP2515 and clear filters
  DiagCAN.begin(&CAN0, &CAN_Timeout);
  DiagCAN.clearCAN_Filter();

  connectRFDuino();
  Serial.println("Waiting for connections...");

  /*while ((fConnected = Blynk.connect()) == false) {
    // Wait until connected
  }*/

}

//--------------------------------------------------------------------------------
//! \brief   LOOP()
//--------------------------------------------------------------------------------
void loop() { 
  Blynk.run();
  if (state && fConnected && DRV_Timeout.Expired(true)) {
    state = DiagCAN.ReadCAN(&DRV, 0);
  }
  Blynk.run();
  if (state && fConnected && BMS_Timeout.Expired(true)) {
    state = DiagCAN.ReadCAN(&BMS, 0);
  }
  Blynk.run();
  if (DEVstatus_Timeout.Expired(true)) {
    DEVstatus();
  }
}

void connectRFDuino () {
  RFduinoBLE.deviceName = "RFDuino";
  RFduinoBLE.advertisementInterval = MILLISECONDS(300);
  RFduinoBLE.txPowerLevel = 0;  // (-20dbM to +4 dBm)

  // start the BLE stack
  RFduinoBLE.begin();
  Blynk.begin(auth);
}

//--------------------------------------------------------------------------------
//! \brief   Read SOC values
//! \brief   Trigger on a virtual pin from Blynk app, set frequency in app
//! \param   selected virtual pin, CONST from Blynk library
//--------------------------------------------------------------------------------
BLYNK_READ(V1) {
  ReadValues_SOC();
  //Serial.print("SOC: "); Serial.println(BMS.SOC);
  //Blynk.virtualWrite(V1, BMS.SOC);
}

//--------------------------------------------------------------------------------
//! \brief   Read Power values (voltage, amps, calculate power)
//! \brief   Trigger on a virtual pin from Blynk app, set frequency in app
//! \param   selected virtual pin, CONST from Blynk library
//--------------------------------------------------------------------------------
BLYNK_READ(V3) {
  ReadValues_Power();
}

//--------------------------------------------------------------------------------
//! \brief   Read LV value, voltage from 12V battery
//! \brief   Trigger on a virtual pin from Blynk app, set frequency in app
//! \param   selected virtual pin, CONST from Blynk library
//--------------------------------------------------------------------------------
BLYNK_READ(V6) {
  if(state) Blynk.virtualWrite(V6, BMS.LV);
}

//--------------------------------------------------------------------------------
//! \brief   Read ODO value
//! \brief   Trigger on a virtual pin from Blynk app, set frequency in app
//! \param   selected virtual pin, CONST from Blynk library
//--------------------------------------------------------------------------------
BLYNK_READ(V7) {
  if(state) Blynk.virtualWrite(V7, BMS.ODO);
  //Serial.print("ODO, "); Serial.println(state);
}

//--------------------------------------------------------------------------------
//! \brief   Read ECO values
//! \brief   Trigger on a virtual pin from Blynk app, set frequency in app
//! \param   selected virtual pin, CONST from Blynk library
//--------------------------------------------------------------------------------
BLYNK_READ(V21) {
  //state = DiagCAN.ReadECO(&DRV);
  //delay(100);
  if(state) Blynk.virtualWrite(V21, DRV.ECO_total);
  if(state) Blynk.virtualWrite(V22, DRV.ECO_accel);
  if(state) Blynk.virtualWrite(V23, DRV.ECO_const);
  if(state) Blynk.virtualWrite(V24, DRV.ECO_coast);
  //Serial.print("ECO, "); Serial.println(state);
}

//--------------------------------------------------------------------------------
//! \brief   Read available power (three stages 99, 66, 33, zero %)
//! \brief   Trigger on a virtual pin from Blynk app, set frequency in app
//! \param   selected virtual pin, CONST from Blynk library
//--------------------------------------------------------------------------------
BLYNK_READ(V25) {
  if(state) Blynk.virtualWrite(V25, DRV.range);
  if(state) {
    String _usablePower ="";
    switch (DRV.usablePower) {
      case 99:
        _usablePower ="> > >";
        break;
      case 66:
        _usablePower ="> >";
        break;
      case 33:
        _usablePower =">";
        break;
      default:
         _usablePower ="- - -";
        break;
    }
    Blynk.virtualWrite(V27, _usablePower);
  }
  //Serial.print("Range, "); Serial.println(state);
}

//--------------------------------------------------------------------------------
//! \brief   Read enery consumption / 100km values, two counters
//! \brief   Trigger on a virtual pin from Blynk app, set frequency in app
//! \param   selected virtual pin, CONST from Blynk library
//--------------------------------------------------------------------------------
BLYNK_READ(V28) {
  //state = DiagCAN.ReadEnergyConsumption(&DRV); 
  //state = true;
  //delay(100);
  if(state) Blynk.virtualWrite(V28, DRV.energyStart /100.0);
  if(state) Blynk.virtualWrite(V29, DRV.energyReset /100.0);
  //Serial.print("Energy, "); Serial.println(state);
}

//--------------------------------------------------------------------------------
//! \brief   Read ODO counters from start and from custom reset
//! \brief   Trigger on a virtual pin from Blynk app, set frequency in app
//! \param   selected virtual pin, CONST from Blynk library
//--------------------------------------------------------------------------------
BLYNK_READ(V30) {
  //state = DiagCAN.ReadUserCounter(&DRV);
  //state = true;
  //delay(100);
  if(state) Blynk.virtualWrite(V30, DRV.odoStart / 10.0);
  if(state) Blynk.virtualWrite(V31, DRV.odoReset / 10.0);
  //Serial.print("UserCounter, "); Serial.println(state);
}

//--------------------------------------------------------------------------------
//! \brief   Uptime counter, for development and to test Blynk connection
//! \brief   Trigger on a virtual pin from Blynk app, set frequency in app
//! \param   selected virtual pin, CONST from Blynk library
//--------------------------------------------------------------------------------
BLYNK_READ(V10) {
  Blynk.virtualWrite(V10, IOcount);
  Serial.println(IOcount++);
  //Serial.print("UptimeCounter, "); Serial.println(state);
  Blynk.virtualWrite(V2, BMS.realSOC / 10.0);
}

//--------------------------------------------------------------------------------
//! \brief   Trigger from virtual button, for development and test
//! \param   selected virtual pin, CONST from Blynk library
//--------------------------------------------------------------------------------
BLYNK_WRITE(V15) {
  int pinData = param.asInt();
  if (pinData == 0) {
    /*for (byte n = 1; n < 94; n++) {
      terminal.print("#"); terminal.print(n);
      terminal.println(": 3885 mV, 19142 As/10");
      delay(100);
      terminal.flush();
      Blynk.run();
    }*/
    DiagCAN.WakeUp();
  }
}

//--------------------------------------------------------------------------------
//! \brief   Sync widget values on first Blynk connection
//--------------------------------------------------------------------------------
BLYNK_CONNECTED() {
  Blynk.syncAll();
}

//--------------------------------------------------------------------------------
//! \brief   Show status of CAN-Bus and Blynk connect
//--------------------------------------------------------------------------------
void DEVstatus() {
  //long ts = millis();
  
  fConnected = Blynk.connected();
  if (!state || !fConnected) { //fConnected
      state = DiagCAN.ClearReadBuffer();
  }
  
  if (!state && failCount < 2) {
    failCount++;
    state = true;
  } else if (failCount > 2) {
    state = false;
    failCount = 0;   
  } else if (state && failCount > 0) {
    failCount--;
    state = true;
  }
  
  Serial.print(state); Serial.print(", "); Serial.println(failCount);
  digitalWrite(0, state);

  if (fConnected) {
    LEDcon.setValue(state * 255); 
    Blynk.virtualWrite(V26, String(DRV.HVactive, BIN)); 
    Blynk.virtualWrite(V11, rssi);
    Blynk.virtualWrite(V9, RFduino_temperature(CELSIUS));
    digitalWrite(1, true);
  } else {
    digitalWrite(1, false);
  }
}

//--------------------------------------------------------------------------------
//! \brief   Callback to return signal strength (RX level -0dBm to -127dBm)
//--------------------------------------------------------------------------------
void RFduinoBLE_onRSSI(int _rssi) {
  rssi = _rssi; 
}


