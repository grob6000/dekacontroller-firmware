/*    
 *    dekacontroller-types.h
 *    Copyright (C) 2022 grob6000 (https://github.com/grob6000)
 *    This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

// definition of types for data structures

typedef enum {Ok, Begin, ZeroM0, ZeroH, SetH, SetM, WaitMark, Error, None} SyncState;
typedef enum {Main, Timezone, Drift, Sync} DisplayMode;
#define MODECOUNT 4

typedef struct {
  DisplayMode displaymode : 2;
  SyncState syncstate : 4;
} ModeStruct;

typedef struct {
  bool displayrefresh : 1;
  bool forcemessage : 1;
  bool timechange : 1;
  bool gpschange : 1;
  bool tzchange : 1;
  bool syncchange : 1;
  bool driftchange : 1;
  bool runchange : 1;
} ChangeFlags;

typedef struct {
  bool run_ok : 1;
  bool gps_hastime : 1;
  bool gps_hasfix : 1;
  bool gps_oldfix : 1;
  bool gps_hascomms : 1;
  bool time_drift : 1;
  bool time_error: 1;
} StatusStruct;
