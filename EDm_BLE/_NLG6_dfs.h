//--------------------------------------------------------------------------------
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
//! \file    NLG6_dfs.h
//! \brief   Definitions and structures for the NLG6-Charger module.
//! \date    2016-November
//! \author  MyLab-odyssey
//! \version 0.5.0
//--------------------------------------------------------------------------------
#ifndef NLG6_DFS_H
#define NLG6_DFS_H

//Definitions for NLG6
#define TEMP_OFFSET 40

//NLG6 data structure
typedef struct {       
  uint16_t MainsAmps[3];          //!< AC current of L1, L2, L3
  uint16_t MainsVoltage[3];       //!< AC voltage of L1, L2, L3
  byte Amps_setpoint;             //!< AC charging current set by user in BC (Board Computer)
  uint16_t AmpsCableCode;         //!< Maximum current cable (resistor coded)
  uint16_t AmpsChargingpoint;     //!< Maxiumum current of chargingpoint
  uint16_t DC_Current;            //!< DC current measured by charger
  uint16_t DC_HV;                 //!< DC HV measured by charger
  byte LV;                        //!< 12V onboard voltage of Charger DC/DC
  byte Temps[7];                  //!< internal temperatures in charger unit and heat exchanger
  byte ReportedTemp;              //!< mean temperature, reported by charger
  byte SocketTemp;                //!< temperature of mains socket charger
  byte CoolingPlateTemp;          //!< temperature of cooling plate 
  String PN_HW;                   //!< Part number of base hardware (wo revisioning)
  String SWrev;                   //!< Part number of base software and updates
} ChargerDiag_t; 

const PROGMEM byte rqChargerPN_HW[4]              = {0x03, 0x22, 0xF1, 0x11};
const PROGMEM byte rqChargerSWrev[4]              = {0x03, 0x22, 0xF1, 0x21};
const PROGMEM byte rqChargerVoltages[4]           = {0x03, 0x22, 0x02, 0x26};
const PROGMEM byte rqChargerAmps[4]               = {0x03, 0x22, 0x02, 0x25};
const PROGMEM byte rqChargerSelCurrent[4]         = {0x03, 0x22, 0x02, 0x2A};
const PROGMEM byte rqChargerTemperatures[4]       = {0x03, 0x22, 0x02, 0x23}; 

const String NLG6_PN_HW = "4519822221";  //!< Part number for NLG6 fast charging hardware

#endif // of #ifndef NLG6_DFS_H
