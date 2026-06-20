/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2024 Damien Maguire <info@evbmw.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BMW_PHEV_BMS_H
#define BMW_PHEV_BMS_H

/*
 * BMW PHEV Gen3/4 SME (Battery Management Electronics) integration.
 *
 * This class serves DUAL roles in the ZombieVerter architecture:
 *   Role A - BMS (extends BMS base class): UDS polling for cell voltages,
 *            temperatures, SOC, SOH, isolation, current/voltage limits.
 *   Role B - Contactor/Shunt (static methods like SBOX/VWBOX): contactor
 *            control via 0x10B/0x53A, vehicle environment CAN frames, SME wakeup.
 *
 * The SME communicates via ISO-TP (UDS over CAN, extended addressing, ID 0x6F1→0x607).
 * Contactors are driven by 0x10B (CRC + alive counter) with 0x53A providing the
 * associated vehicle/contactor STATE indication (A/B/C/D sequence).
 *
 * Ported from the Battery-Emulator BMW-PHEV-BATTERY implementation.
 */

#include <stdint.h>
#include "canhardware.h"
#include "params.h"
#include "bms.h"
#include "uds_handler.h"

class BmwPhevBMS : public BMS
{
public:
   // ---- BMS interface (polymorphic, called by VCU scheduler) ----
   // SetCanInterface: registers SME broadcast + UDS CAN IDs on bmsCan.
   // DecodeCAN(uint8_t*): dispatches received frames to per-ID handlers.
   // MaxChargeCurrent: returns 0 if SME demands open or limits exceeded,
   //   otherwise the SME's charge current limit from 0x2F5 broadcast.
   // Task100Ms: called every 100ms. Handles UDS polling, CAN TX (0x12F,
   //   0x53A, vehicle keep-alive, wakeup), timeout counting, and updating
   //   ZombieVerter params with the latest SME data.
   void SetCanInterface(CanHardware* c) override;
   void DecodeCAN(int id, uint8_t* data) override;
   float MaxChargeCurrent() override;
   void Task100Ms() override;

   // ---- Static contactor/shunt interface (matching SBOX/VWBOX pattern) ----
   // ControlContactors: called every 10ms from Ms10Task with the live VCU
   //   opmode. Drives 0x10B (contactor command) + 0x53A state machine.
   // RegisterCanMessages: registers SME broadcast IDs on the shunt CAN bus.
   // DecodeCAN(uint32_t[2]): shunt-style decode dispatcher.
   static void ControlContactors(int opmode, CanHardware* can);
   static void RegisterCanMessages(CanHardware* can);
   static void DecodeCAN(int id, uint32_t data[2]);

   // ---- Static data for web UI (read from params) ----
   static int32_t Voltage;    // Pack voltage in dV
   static int32_t Amperes;    // Current in deciAmps
   static int32_t Temperature; // Max cell temp in °C

   // ---- Contactor state accessor (for utils.cpp precharge gating) ----
   // Returns true when the SME's internal contactors are confirmed closed:
   // 0x10B is commanding close, 0x53A is in STEADY state AND measured
   // feedback confirms it (post-contactor voltage tracking pack voltage,
   // or pack current flowing). Used by ProcessUdc() to gate udc — we must
   // not report voltage as valid until the SME has actually closed.
   static bool AreContactorsClosed();

   // Post-contactor voltage measured via UDS 0xDD66 (dV). Exposed as udc3
   // by ProcessUdc() so the close-confirmation feedback is visible on the
   // web UI (compare against udc2 = pack voltage).
   static uint16_t GetPostContactorVoltage() { return postContactorVoltage_dV; }

private:
   // ---- CRC ----
   static uint8_t Crc8SAEJ1850(const uint8_t* data, uint8_t len, uint8_t init);
   static const uint8_t crc8_table[256];

   // ---- Alive counter ----
   static uint8_t IncrementAlive(uint8_t counter);
   static const uint8_t ALIVE_MAX = 14;

   // ---- UDS / ISO-TP ---- (delegated to UdsHandler)
   static UdsHandler uds;
   static void OnUdsResponse(uint8_t moduleID, const uint8_t* data,
                             uint16_t len, void* ctx);

   // ---- Contactor state machine (0x53A) ----
   enum Phev53AState { ST53A_IDLE, ST53A_CLOSE_REQ, ST53A_ESCALATED, ST53A_STEADY };
   static Phev53AState phev53aState;
   static bool contactorCloseReq;
   static uint16_t startupCounter;   // needs >255 for 3.2s delay at 10ms ticks

   // ---- Pre-close balancing stop ----
   static uint8_t preCloseStopsRemaining;
   static const uint8_t PRE_CLOSE_STOP_COUNT = 3;

   // ---- Balancing on/off burst (driven by BMS_BalancingOn param) ----
   static bool    lastBalancingOn;          // previous param state for edge detection
   static uint8_t balancingBurstRemaining;  // >0 = burst in progress
   static bool    balancingBurstIsStart;    // true=send START, false=send STOP
   static const uint8_t BALANCING_BURST_COUNT = 3;

   // ---- SME emergency contactor open (0x112 bytes 5-6) ----
   static bool smeDemandsOpen;       // true when SME requests contactors open
   static bool smeEmergencyLatched;  // set after an SME-demanded open; blocks re-close until opmode leaves RUN/CHARGE
   static bool smeSignalsInvalid;    // true while any 0x112 open-request field reads "signal invalid"
   static bool shutdownPending;      // true when MOD_OFF requested, waiting for current decay
   static uint8_t smeOpenDelayTicks; // countdown after current drops before forcing open (shared)

   // ---- Contactor close confirmation (measured feedback) ----
   static bool contactorsConfirmedClosed; // post-contactor V tracks pack V, or current flowing

   // ---- 0x53A STATE B hold counter ----
   static uint8_t stateBHoldCount;   // number of 200ms ticks remaining in STATE B

   // ---- Timing counters (incremented each Task100Ms = 100ms) ----
   static uint8_t tickCounter;       // 0..9, rolls every 1s
   static uint8_t slowPollIndex;     // 0..numSlowReqs-1
   static uint8_t fastPollIndex;     // 0..numFastReqs-1
   static uint16_t timeoutCounter;   // decremented each Task100Ms, reset on data rx (BMS_Timeout*10, up to 1200)
   static bool batteryAwake;
   static bool wakeInProgress;       // bus is at 100kbps for SME wakeup — suppress all other TX

   // ---- Stale cell-data detection (frozen UDS values while current flows) ----
   static int16_t staleLastMinCell;  // last seen minCellVoltage_mV
   static int16_t staleLastMaxCell;  // last seen maxCellVoltage_mV
   static uint16_t cellStaleTicks;   // 100ms ticks since either value last changed
   static bool cellDataStale;        // both frozen > STALE_PERIOD while current flows

   // ---- 0x10B 20ms timing (every 2nd Ms10Task call) ----
   static uint8_t ms10Counter;

   // ---- One-shot command silence ----
   static bool udsOneShotPending;
   static uint8_t udsOneShotSilence;

   // ---- Boot balancing stop sent flag ----
   static bool bootBalancingStopSent;

   // ---- Internal data (populated from CAN/UDS) ----
   static uint16_t packVoltage_dV;       // from 0x112
   static int32_t  packCurrent_dA;       // from 0x112 (signed, deci-amps: raw 0.1A/bit minus 8192 offset)
   static uint16_t postContactorVoltage_dV; // from UDS 0xDD66 (dV, used for close confirmation)
   static uint16_t displaySoc;           // from 0x432 (0.5%/bit)
   static uint16_t avgSocState;          // from UDS 0xDDC4 (0.01%)
   static uint16_t minSohState;          // from UDS 0xDD7B (%*100)
   static int16_t  minCellVoltage_mV;    // from UDS 0xDFA0
   static int16_t  maxCellVoltage_mV;    // from UDS 0xDFA0
   static int16_t  minTemperature_C;     // from 0x1FA
   static int16_t  maxTemperature_C;     // from 0x1FA
   static uint16_t maxDesignVoltage_dV;  // from UDS 0xDD7E
   static uint16_t minDesignVoltage_dV;  // from UDS 0xDD7E
   static int16_t  maxDischargeAmps;     // from 0x2F5
   static int16_t  maxChargeAmps;        // from 0x2F5
   static uint16_t isoExtKOhm;           // from UDS 0xDD6A bytes 3-4
   static uint16_t isoTrgKOhm;           // from UDS 0xDD6A bytes 5-6 (trigger threshold reading)
   static uint16_t isoIntKOhm;           // from UDS 0xDD6A bytes 7-8
   static uint8_t  isoExtPlausible;      // from UDS 0xDD6A byte 9  (1 = plausible)
   static uint8_t  isoTrgPlausible;      // from UDS 0xDD6A byte 10 (1 = plausible)
   static uint8_t  isoIntPlausible;      // from UDS 0xDD6A byte 11 (1 = plausible)
   static uint16_t isoRawKOhm;           // from UDS 0xD6D9 (single-frame raw iso)
   static uint8_t  isoQuality;           // from UDS 0xD6D9 (quality 0-21, higher = better)
   static uint8_t  isoTestStatus;        // 71 03 AD 61 byte5: 0=not run, 1=successful, 2=running
   static uint8_t  isoTestFault;         // 71 03 AD 61 byte6: 0=no fault, 1=fault, 0xFF=undefined
   static uint8_t  isoStatusBoostSecs;   // 1Hz status polling countdown after test start
   static uint8_t  isoErrExt;            // from 0x1FA byte0 bits0-1 (qualifier: 2 = fault)
   static uint8_t  isoErrInt;            // from 0x1FA byte0 bits2-3 (qualifier: 2 = fault)
   static uint8_t  isoWarn;              // from 0x1FA byte2 bits6-7 (qualifier: 2 = fault)
   static uint8_t  interlockStat;        // from 0x1FA byte1 bits0-1 (qualifier: 2 = not seated)
   static uint8_t  prechargeStat;        // from 0x1FA byte1 bits2-3 (2 = precharge blocked)
   static uint8_t  dcswStat;             // from 0x1FA byte1 bits4-5 (0=open 1=precharging 2=closed)
   static uint8_t  emgModeStat;          // from 0x1FA byte1 bits6-7 (qualifier: 2 = emergency)
   static uint8_t  svcReqStat;           // from 0x1FA byte2 bits0-1 (qualifier: 2 = service required)
   static uint8_t  weldStat;             // from 0x1FA byte2 bits4-5 (1/2 = welded contactors)
   static uint8_t  coldValveStat;        // from 0x1FA byte3 bits0-3 (0 = OK)
   static uint16_t maxChargeVoltage_dV;  // from 0x2F5 bytes0-1
   static uint16_t minDischargeVoltage_dV; // from 0x2F5 bytes4-5
   static uint32_t dtcCodes[5];          // last 5 DTCs from 19 02 (3-byte codes)
   static uint8_t  dtcCount;             // total valid DTCs in last read
   static uint32_t availChargePowerShort_W;   // from 0x40D
   static uint32_t availDischargePowerShort_W;
   static uint32_t availChargePowerLong_W;    // from 0x40D
   static uint32_t availDischargePowerLong_W;
   static uint8_t  balancingStatus;      // from UDS 0xAD6B
   static bool     packLimitAvailable;
   static uint16_t energyMax_50Wh;        // from 0x431 (raw/50 = kWh)
   static uint16_t predictedEnergyRemaining_Wh; // from 0x239 (Wh)

   // ---- UDS request CAN frames ----
   static const uint8_t UDS_SOC[5];
   static const uint8_t UDS_SOH[5];
   static const uint8_t UDS_VOLTAGE_LIMITS[5];
   static const uint8_t UDS_CURRENT_LIMITS[5];
   static const uint8_t UDS_CELLSUMMARY[5];
   static const uint8_t UDS_CELL_VOLTAGES[5];
   static const uint8_t UDS_CELL_TEMP[5];
   static const uint8_t UDS_ISO_READING1[5];
   static const uint8_t UDS_ISO_READING2[5];
   static const uint8_t UDS_BALANCING_STATUS[8];
   static const uint8_t UDS_READ_DTC[8];
   static const uint8_t UDS_CLEAR_DTC[8];
   static const uint8_t UDS_HARD_RESET[4];
   static const uint8_t UDS_BALANCING_START[8];
   static const uint8_t UDS_BALANCING_STOP[8];
   static const uint8_t UDS_ISOLATION_TEST[8];
   static const uint8_t UDS_ISOLATION_STATUS[8];
   static const uint8_t UDS_POST_VOLTAGE[5];

   // ---- UDS poll arrays ----
   static const uint8_t* fastReqs[4];
   static const int numFastReqs;
   static const uint8_t* slowReqs[9];
   static const int numSlowReqs;

   // ---- Vehicle CAN TX frames ----
   static const uint8_t VEH_3A0[8];
   static const uint8_t VEH_3CA[8];
   static const uint8_t VEH_3E8[2];
   static const uint8_t VEH_2CA[2];

   // ---- Singleton pointer for static methods ----
   static BmwPhevBMS* instance;
   static CanHardware* bmsCan;    // CAN interface for BMS UDS/data (set in SetCanInterface)
};

#endif // BMW_PHEV_BMS_H
