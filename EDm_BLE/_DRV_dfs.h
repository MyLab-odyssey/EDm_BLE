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
//! \file    DRV_dfs.h
//! \brief   Definitions and structures for drivetrain statistics.
//! \date    2016-November
//! \author  MyLab-odyssey
//! \version 0.1.1
//--------------------------------------------------------------------------------
#ifndef DRV_DFS_H
#define DRV_DFS_H

//Definitions for DRV parameters

//DRV data structure
typedef struct {     
  uint16_t velocity;         //!< Speed as reported in the dashboard
  int HVactive;              //!< Status of car
  byte usablePower;          //!< available power in % (99, 66, 33 as the power bars in dash)
  byte range;                //!< Available range as calculated by the car
  byte ECO_accel;            //!< ECO driving indicator for acceleration (x/2)
  byte ECO_const;            //!< ECO indicator for constant driving (x/2)
  byte ECO_coast;            //!< ECO driving indicator for effective coasting (x/2)
  byte ECO_total;            //!< Total ECO driving indicator (weighted average) (x/2)
  uint16_t energyStart;      //!< Used energy per 100km from start (as in dashboard) (x/100)
  uint16_t energyReset;      //!< Used energy per 100km from reset (as in dashboard) (x/100)
  uint16_t odoStart;         //!< ODO count in km from start (as in dashboard) (x/100)
  uint16_t odoReset;         //!< ODO count in km from reset (as in dashboard) (x/100)
  
} DriveStats_t; 

#endif // of #ifndef DRV_DFS_H
