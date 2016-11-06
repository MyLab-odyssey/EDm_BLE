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
//! \file    EDm_CAN.ino
//! \brief   Helper function for CAN bus readouts.
//! \brief   Functions will be changed during development without notification!
//! \date    2016-November
//! \author  MyLab-odyssey
//! \version 0.1.0
//--------------------------------------------------------------------------------

boolean ReadValues_SOC() {
  if (1) {
    Blynk.virtualWrite(V1, BMS.SOC);
    Blynk.virtualWrite(V2, BMS.realSOC / 10.0);
    //Serial.print("SOC true in: ");
    //Serial.println(millis() - _timer);
    return true;
  } else {
    //Serial.print("SOC false in: ");
    //Serial.println(millis() - _timer);
    return false;
  }
}

boolean ReadValues_Power() {
  if (1) {
    Blynk.virtualWrite(V3, BMS.HV);
    Blynk.virtualWrite(V4, ((BMS.Power / 8192.0) -1) * 300);
    Blynk.virtualWrite(V5, DRV.velocity);
    //Serial.print("Power true in: ");
    //Serial.println(millis() - _timer);
    return true;
  } else {
    //Serial.print("Power false in: ");
    //Serial.println(millis() - _timer);
    return false;
  }
}

boolean getDataSOC() {
  taskSOC = 1;
  do {
    switch (taskSOC) {
      case 1:
        state = DiagCAN.ReadSOC(&BMS);
        break;
      case 2:
        state = DiagCAN.ReadSOCinternal(&BMS);
        break;
    }
    taskSOC++;
  } while (state && taskSOC < 3);
  
  if (state) {
    Blynk.virtualWrite(V1, BMS.SOC);
    Blynk.virtualWrite(V2, BMS.realSOC / 10.0);
    return true;
  } else {
    return false;
  }  
}

boolean getDataPower() {
  taskPower = 1;
  do {
    switch (taskPower) {
     case 1:
        state = DiagCAN.ReadHV(&BMS);
        break; 
      case 2:
        state = DiagCAN.ReadPower(&BMS);
        break;
      case 3:
        state = DiagCAN.ReadVelocity(&DRV);
        break;
    }
    taskPower++;
  } while (state && taskPower < 4);
  
  if (state) {
    Blynk.virtualWrite(V3, BMS.HV);
    Blynk.virtualWrite(V4, ((BMS.Power / 8192.0) -1) * 300);
    Blynk.virtualWrite(V5, DRV.velocity);
    return true;
  } else {
    return false;
  }   
}

