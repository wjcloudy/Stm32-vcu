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
 * BMW PHEV Gen3/4 SME (Battery Management Electronics) integration.
 * Ported from the Battery-Emulator BMW-PHEV-BATTERY implementation.
 *
 * Provides:
 *   - UDS polling for cell voltages, temperatures, SOC, SOH, isolation, limits
 *   - Contactor control via 0x10B / 0x53A handshake
 *   - Vehicle environment CAN frames to keep the SME happy
 *   - SME wakeup via 0x554 magic packet
 *
 * ============================================================================
 * ARCHITECTURE
 * ============================================================================
 * This class serves DUAL roles in the ZombieVerter architecture:
 *
 *   Role A - BMS (extends BMS base class): selected via Param::BMS_Mode=6
 *     Task100Ms() is called every 100ms. It performs UDS polling, transmits
 *     vehicle environment CAN frames, and updates BMS_Vmin/Vmax/Tmin/Tmax/
 *     BMS_ChargeLim parameters. The MaxChargeCurrent() method is used by
 *     the charge termination logic.
 *
 *   Role B - Contactor/Shunt (static methods like SBOX/VWBOX): selected via
 *     Param::ShuntType=5. ControlContactors() is called every 10ms from Ms10Task
 *     with the current opmode (0=OFF, 1=RUN, 2=PRECHARGE, 3=PCHFAIL, 4=CHARGE).
 *     It drives the contactors via 0x10B at 20ms with CRC + alive counter.
 *
 * ============================================================================
 * CAN BUS SETUP
 * ============================================================================
 * The SME communicates on a SINGLE CAN bus at 500 kbps. Both BMSCan and
 * ShuntCan parameters should point to the same CAN interface. Extended
 * addressing is used: UDS requests go to 0x6F1, responses come back on
 * 0x607. Broadcast data from the SME arrives on standard 11-bit IDs.
 *
 * ============================================================================
 * SME BROADCAST MAP (frames received FROM the SME)
 * ============================================================================
 * 0x112  20ms   Status Of High-Voltage Battery 2
 *               bytes0-1: current (offset 8192, *10 = mA)
 *               bytes2-3: pack voltage in dV (0.1 V/bit LE)
 *               byte5-6: contactor open request flags
 *
 * 0x2F5  100ms  High-Voltage Battery Charge/Discharge Limitations
 *               bytes0-1: max charge voltage
 *               bytes2-3: max charge current (offset 819.2)
 *               bytes4-5: min discharge voltage
 *               bytes6-7: max discharge current (offset 819.2)
 *
 * 0x239  200ms  Predicted charge condition and target
 *               bytes1-2: predicted energy charge condition (Wh)
 *               bytes3-4: predicted energy charging target (*0.02 = kWh)
 *
 * 0x40D  1s     Charging status of high-voltage storage - 1
 *               Available power short/long term charge/discharge (*3 = W)
 *
 * 0x430  1s     Charging status of high-voltage battery - 2
 *               Prediction voltages short/long term charge/discharge
 *
 * 0x431  200ms  Data High-Voltage Battery Unit
 *               byte0: service disconnect, isolation measurement, abort flags
 *               bytes2-3: prediction duration charging (minutes)
 *               byte4: prediction time end of charging
 *               bytes5-6: energy content maximum (/50 = kWh)
 *
 * 0x432  200ms  SOC% info
 *               byte0: operating mode + target CV voltage
 *               byte2: charging condition minimum (/2 = %)
 *               byte3: charging condition maximum (/2 = %)
 *               byte4: display SOC (0.5%/bit)
 *
 * 0x1FA  1s     Status Of High-Voltage Battery - 1
 *               byte0: isolation errors, cooling request, valve status
 *               byte1: interlock, precharge, DCSW, emergency mode
 *               byte2: service request, emergency error, DCSW error, iso warning
 *               byte3: cold shutoff valve
 *               byte6: min temperature (-50 offset, °C)
 *               byte7: max temperature (-50 offset, °C)
 *
 * 0x607  async  UDS responses (extended addressing, target 0xF1)
 *
 * ============================================================================
 * VEHICLE TX MAP (frames WE transmit to keep the SME happy)
 * ============================================================================
 * These frames silence "No message" (CAD4xx) DTCs in the SME. Only STATIC
 * frames or frames with implemented counters are transmitted.
 *
 * 0x10B  20ms   Contactors command (EME)           CRC + alive counter
 * 0x12F  100ms  Terminal Data                      CRC + counter 0x20-0x2E
 * 0x53A  200ms  Contactor STATE indication          A/B/C/D state machine
 * 0x3A0  1s     Vehicle condition (BDC)             Static
 * 0x328  1s     Relative time / clock (KOMBI)       Live seconds + days
 * 0x3CA  1s     Driving-info forecast (KOMBI)       Static
 * 0x3E8  1s     OBD diagnosis, engine ctrl (EME)    Static
 * 0x2CA  1s     Ambient temperature (KOMBI)         0x6E = 15°C static
 *
 * NOT transmitted (need rolling counter logic first):
 * 0x1A1  Vehicle speed (DSC)          byte1 lo-nibble rolls 0..E
 * 0x433  HV-battery specification     byte1 toggles 0x0C<->0x0D
 *
 * ============================================================================
 * UDS POLL MAP (requests sent TO SME on 0x6F1, responses on 0x607)
 * ============================================================================
 * FAST POLL (every 200ms, round-robin):
 *   22 DF A0  Cell summary (min/max voltage + temps)
 *   22 DD C4  SOC% (0.01%/bit)
 *   22 DD 7E  Voltage limits (multi-frame)
 *   22 DD 66  Post-contactor voltage
 *
 * SLOW POLL (every 1s, round-robin):
 *   22 DD 6A  Isolation reading 1 (multi-frame)
 *   22 D6 D9  Isolation reading 2
 *   22 DD 7D  Current limits (multi-frame)
 *   22 DD 7B  SOH%
 *   22 DF A5  All 96 individual cell voltages (multi-frame)
 *   22 DD C0  Cell temperatures min/max/avg (multi-frame)
 *   31 03 AD 6B  Balancing status (01=active, 03=not active)
 *   19 02 0C  Read DTCs (multi-frame)
 *   22 F1 90  Paired VIN (placeholder - not fully implemented)
 *
 * ONE-SHOT commands (triggered by user request, silence polls for 1s after):
 *   14 FF FF FF  Clear DTCs
 *   11 01        SME hard reset
 *   31 01 AD 61  Start isolation test
 *   31 01 AD 6B  Start balancing
 *   31 02 AD 6B  Stop balancing
 *
 * ============================================================================
 * CONTACTOR CLOSE SEQUENCE
 * ============================================================================
 * Two CAN messages work together. 0x10B is the actual contactor driver;
 * 0x53A reflects an associated vehicle/contactor STATE indication.
 *
 * 0x10B (20ms) - ACTUAL CONTACTOR DRIVER
 *   byte0 = CRC (SAE J1850, init 0x3F, over bytes 1..2)
 *   byte1 = high nibble: 0x0=open, 0x1=close; low nibble: alive 0..14
 *   byte2 = 0xFC (constant)
 *   Close is only commanded when ALL of:
 *     - contactorCloseReq == true
 *     - startupCounter >= 320 (~3.2s boot delay at 10ms ticks)
 *     - 0x53A state machine reached ESCALATED or STEADY
 *     - pre-close balancing stop frames all sent
 *     - no SME emergency open latched (requires opmode to leave RUN/CHARGE)
 *
 * 0x53A (200ms) - STATE INDICATION
 *   STATE A IDLE:      byte5=0x00, byte6=0x01  (contactors open)
 *   STATE B CLOSE_REQ: byte5=0x80, byte6=0x01  (close announced ~3.5s)
 *   STATE C ESCALATED: byte5=0x84, byte6=0x01  (single transient frame)
 *   STATE D STEADY:    byte5=0x84, byte6=0x00  (contactors closed)
 *
 * The OEM bus announces STATE B BEFORE 0x10B drives the contactors closed.
 * We replay this sequence compressed: B held briefly, then C transient,
 * then D steady. 0x10B is gated on reaching C/D.
 *
 * PRE-CLOSE BALANCING STOP:
 *   Balancing BLOCKS contactor close in the SME. Before closing, we send
 *   3 guarded stopRoutine(0xAD6B) frames (one per UDS silence window) to
 *   ensure any latched balancing is cancelled.
 *
 * ============================================================================
 * SME WAKEUP
 * ============================================================================
 * When the SME is asleep (no 0x112/0x432 received), we attempt to wake it
 * every 1s by sending the magic packet 0x554 {0x5A,0xA5,0x5A,0xA5}.
 * Ideally this should be sent at 100 kbps (TJA1055 remote wake timing),
 * but repeated attempts at 500 kbps will also wake most modules.
 *
 * ============================================================================
 * NOTES / TODO
 * ============================================================================
 * - SME emergency contactor open: 0x112 bytes 5-6 are decoded. When the SME
 *   demands contactors open, MaxChargeCurrent() returns 0 (stops charging)
 *   and ControlContactors() executes a safe shutdown: wait for pack current
 *   < 5A, then wait 500ms, then force 0x10B open. The event is LATCHED:
 *   no re-close is allowed until the VCU opmode leaves RUN/CHARGE (key cycle).
 * - Contactor close CONFIRMATION is measured, not assumed: AreContactorsClosed()
 *   only reports true once post-contactor voltage (UDS 0xDD66, polled every
 *   fast slot while waiting) tracks pack voltage within 10%, or pack current
 *   >5A is flowing. Until then udc stays 0 and the VCU remains in precharge,
 *   so an SME that refuses to close leads to PCHFAIL instead of a false RUN.
 * - Stale-data guard: if min/max cell voltages both freeze for 60 minutes
 *   while >5A flows, BMS_Vmin/Vmax are forced invalid (99/-1) to stop charge.
 * - UDS reassembly timeout: if no FF/CF arrives for 2 seconds, the in-progress
 *   transfer is abandoned so a lost frame can't wedge polling permanently.
 * - 0x53A STATE B now holds for 5 × 200ms = 1s before escalating to C/D,
 *   matching the OEM ordering (announce close before 0x10B drives contactors).
 * - Cell temperatures come from the 0x1FA broadcast only. The UDS 0xDDC0
 *   response layout is unverified, so it is polled but not parsed.
 * - While the SME wakeup runs the bus at 100kbps, ALL other TX on the bus is
 *   suppressed (wakeInProgress) — mixed-baud frames would cause error storms.
 * - Paired VIN (22 F1 90) and pack info (22 DF 71) are polled but not
 *   fully decoded into params yet.
 * - Isolation test results (31 03 AD 61) are not polled after starting
 *   the test; the result polling loop should be added.
 * - Vehicle frames 0x1A1 (speed) and 0x433 (HV spec) are NOT transmitted
 *   because they require rolling counters that aren't implemented yet.
 *   Sending frozen values risks "signal invalid" DTCs which are worse than
 *   the "No message" DTCs from silence.
 * - Per-cell voltages (96 cells from 22 DF A5) are scanned for min/max
 *   but not exposed individually via params (no space in current Param
 *   system for 96 cell values).
 */

#include "bmw_phev_bms.h"
#include <libopencm3/stm32/rtc.h>
#include <cstring>
#include "uds_handler.h"

// Set to 0 for release builds: 0x7E0 overlaps the standard OBD-II engine-ECU
// request ID and will confuse diagnostic tools sharing the bus. The frame
// layout is documented in Documentation/CAN dbcs/ZV_Diag.dbc.
#define PHEV_DIAG_TX 1

/*===========================================================================
 * SAE J1850 CRC-8 lookup table (poly 0x1D, init 0x00)
 *===========================================================================*/
const uint8_t BmwPhevBMS::crc8_table[256] = {
    0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
    0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
    0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
    0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
    0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
    0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
    0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
    0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
    0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
    0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
    0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
    0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
    0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
    0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
    0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
    0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4,
};

/*===========================================================================
 * UDS Request Payloads (extended addressing: byte0=0x07 target, byte1=PCI)
 *===========================================================================*/
const uint8_t BmwPhevBMS::UDS_SOC[5]              = {0x07, 0x03, 0x22, 0xDD, 0xC4};
const uint8_t BmwPhevBMS::UDS_SOH[5]              = {0x07, 0x03, 0x22, 0xDD, 0x7B};
const uint8_t BmwPhevBMS::UDS_VOLTAGE_LIMITS[5]   = {0x07, 0x03, 0x22, 0xDD, 0x7E};
const uint8_t BmwPhevBMS::UDS_CURRENT_LIMITS[5]   = {0x07, 0x03, 0x22, 0xDD, 0x7D};
const uint8_t BmwPhevBMS::UDS_CELLSUMMARY[5]      = {0x07, 0x03, 0x22, 0xDF, 0xA0};
const uint8_t BmwPhevBMS::UDS_CELL_VOLTAGES[5]    = {0x07, 0x03, 0x22, 0xDF, 0xA5};
const uint8_t BmwPhevBMS::UDS_CELL_TEMP[5]        = {0x07, 0x03, 0x22, 0xDD, 0xC0};
const uint8_t BmwPhevBMS::UDS_ISO_READING1[5]     = {0x07, 0x03, 0x22, 0xDD, 0x6A};
const uint8_t BmwPhevBMS::UDS_ISO_READING2[5]     = {0x07, 0x03, 0x22, 0xD6, 0xD9};
const uint8_t BmwPhevBMS::UDS_BALANCING_STATUS[8] = {0x07, 0x04, 0x31, 0x03, 0xAD, 0x6B, 0x00, 0x00};
const uint8_t BmwPhevBMS::UDS_READ_DTC[8]         = {0x07, 0x03, 0x19, 0x02, 0x0C, 0x00, 0x00, 0x00};
const uint8_t BmwPhevBMS::UDS_CLEAR_DTC[8]        = {0x07, 0x04, 0x14, 0xFF, 0xFF, 0xFF, 0x00, 0x00};
const uint8_t BmwPhevBMS::UDS_HARD_RESET[4]       = {0x07, 0x02, 0x11, 0x01};
const uint8_t BmwPhevBMS::UDS_BALANCING_START[8]  = {0x07, 0x04, 0x31, 0x01, 0xAD, 0x6B, 0x00, 0x00};
const uint8_t BmwPhevBMS::UDS_BALANCING_STOP[8]   = {0x07, 0x04, 0x31, 0x02, 0xAD, 0x6B, 0x00, 0x00};
const uint8_t BmwPhevBMS::UDS_ISOLATION_TEST[8]   = {0x07, 0x04, 0x31, 0x01, 0xAD, 0x61, 0x00, 0x00};
const uint8_t BmwPhevBMS::UDS_POST_VOLTAGE[5]     = {0x07, 0x03, 0x22, 0xDD, 0x66};

/*===========================================================================
 * UDS Poll Arrays
 *===========================================================================*/
const uint8_t* BmwPhevBMS::fastReqs[4] = {
    UDS_CELLSUMMARY, UDS_SOC, UDS_VOLTAGE_LIMITS, UDS_POST_VOLTAGE
};
const int BmwPhevBMS::numFastReqs = 4;

const uint8_t* BmwPhevBMS::slowReqs[9] = {
    UDS_ISO_READING1, UDS_ISO_READING2, UDS_CURRENT_LIMITS,
    UDS_SOH, UDS_CELL_VOLTAGES, UDS_CELL_TEMP,
    UDS_BALANCING_STATUS, UDS_READ_DTC, UDS_READ_DTC  // Paired VIN skipped, DTC doubled for placeholder
};
const int BmwPhevBMS::numSlowReqs = 9;

/*===========================================================================
 * Vehicle Environment CAN Frames (static, keep SME happy)
 *===========================================================================*/
const uint8_t BmwPhevBMS::VEH_3A0[8] = {0xFF, 0xFF, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
const uint8_t BmwPhevBMS::VEH_3CA[8] = {0xB7, 0x60, 0x01, 0x0F, 0x0F, 0x31, 0xFF, 0xFF};
const uint8_t BmwPhevBMS::VEH_3E8[2] = {0xF1, 0xFF};
const uint8_t BmwPhevBMS::VEH_2CA[2] = {0x6E, 0x6F};

/*===========================================================================
 * Static Member Initialization
 *===========================================================================*/
BmwPhevBMS* BmwPhevBMS::instance = nullptr;
CanHardware* BmwPhevBMS::bmsCan = nullptr;

int32_t BmwPhevBMS::Voltage = 0;
int32_t BmwPhevBMS::Amperes = 0;
int32_t BmwPhevBMS::Temperature = 0;

UdsHandler BmwPhevBMS::uds;
BmwPhevBMS::Phev53AState BmwPhevBMS::phev53aState = ST53A_IDLE;
bool BmwPhevBMS::contactorCloseReq = false;
uint16_t BmwPhevBMS::startupCounter = 0;
uint8_t BmwPhevBMS::preCloseStopsRemaining = 0;
bool BmwPhevBMS::smeDemandsOpen = false;
bool BmwPhevBMS::smeEmergencyLatched = false;
bool BmwPhevBMS::smeSignalsInvalid = false;
bool BmwPhevBMS::shutdownPending = false;
uint8_t BmwPhevBMS::smeOpenDelayTicks = 0;
uint8_t BmwPhevBMS::stateBHoldCount = 0;
bool BmwPhevBMS::contactorsConfirmedClosed = false;

uint8_t BmwPhevBMS::tickCounter = 0;
uint8_t BmwPhevBMS::slowPollIndex = 0;
uint8_t BmwPhevBMS::fastPollIndex = 0;
uint16_t BmwPhevBMS::timeoutCounter = 0;
bool BmwPhevBMS::batteryAwake = false;
bool BmwPhevBMS::wakeInProgress = false;
int16_t BmwPhevBMS::staleLastMinCell = 0;
int16_t BmwPhevBMS::staleLastMaxCell = 0;
uint16_t BmwPhevBMS::cellStaleTicks = 0;
bool BmwPhevBMS::cellDataStale = false;
uint8_t BmwPhevBMS::ms10Counter = 0;
bool BmwPhevBMS::udsOneShotPending = false;
uint8_t BmwPhevBMS::udsOneShotSilence = 0;
bool BmwPhevBMS::bootBalancingStopSent = false;
bool BmwPhevBMS::lastBalancingOn = false;
uint8_t BmwPhevBMS::balancingBurstRemaining = 0;
bool BmwPhevBMS::balancingBurstIsStart = false;

uint16_t BmwPhevBMS::packVoltage_dV = 3700;
int32_t  BmwPhevBMS::packCurrent_dA = 0;
uint16_t BmwPhevBMS::postContactorVoltage_dV = 0;
uint16_t BmwPhevBMS::displaySoc = 0;
uint16_t BmwPhevBMS::avgSocState = 5000;
uint16_t BmwPhevBMS::minSohState = 9999;
int16_t  BmwPhevBMS::minCellVoltage_mV = 3700;
int16_t  BmwPhevBMS::maxCellVoltage_mV = 3700;
int16_t  BmwPhevBMS::minTemperature_C = 0;
int16_t  BmwPhevBMS::maxTemperature_C = 0;
uint16_t BmwPhevBMS::maxDesignVoltage_dV = 4650;
uint16_t BmwPhevBMS::minDesignVoltage_dV = 3000;
int16_t  BmwPhevBMS::maxDischargeAmps = 0;
int16_t  BmwPhevBMS::maxChargeAmps = 0;
uint16_t BmwPhevBMS::isoExtKOhm = 0;
uint16_t BmwPhevBMS::isoIntKOhm = 0;
uint16_t BmwPhevBMS::isoRawKOhm = 0;
uint32_t BmwPhevBMS::availChargePowerShort_W = 0;
uint32_t BmwPhevBMS::availDischargePowerShort_W = 0;
uint32_t BmwPhevBMS::availChargePowerLong_W = 0;
uint32_t BmwPhevBMS::availDischargePowerLong_W = 0;
uint8_t  BmwPhevBMS::balancingStatus = 4; // 4 = unknown
bool     BmwPhevBMS::packLimitAvailable = false;
uint16_t BmwPhevBMS::energyMax_50Wh = 0;
uint16_t BmwPhevBMS::predictedEnergyRemaining_Wh = 0;

/*===========================================================================
 * CRC / Alive Helpers
 *===========================================================================*/
uint8_t BmwPhevBMS::Crc8SAEJ1850(const uint8_t* data, uint8_t len, uint8_t init)
{
    uint8_t crc = init;
    for (uint8_t i = 0; i < len; i++)
        crc = crc8_table[(crc ^ data[i]) & 0xFF];
    return crc;
}

uint8_t BmwPhevBMS::IncrementAlive(uint8_t counter)
{
    counter++;
    if (counter > ALIVE_MAX) counter = 0;
    return counter;
}

/*===========================================================================
 * UDS response callback
 *===========================================================================*/
void BmwPhevBMS::OnUdsResponse(uint8_t moduleID, const uint8_t* data,
                               uint16_t len, void* ctx)
{
    (void)ctx;

    // Single-frame responses: data[0]=service, data[1..2]=parameter
    // Multi-frame responses: data[0..2]=62/DF/A0 header, data[3+]=payload
    switch (moduleID)
    {
    // ---- Single-frame responses ----
    case 0xC4: // SOC (62 DD C4)
        if (len >= 5)
            avgSocState = (data[3] << 8) | data[4];
        break;

    case 0x7B: // SOH (62 DD 7B)
        if (len >= 4)
            minSohState = (uint16_t)data[3] * 100;
        break;

    case 0xD9: // Raw isolation (62 D6 D9) — kept separate from the qualified
               // ext/int readings of 0xDD6A so BMS_IsolationExt doesn't flicker
               // between two different measurements
        if (len >= 5)
            isoRawKOhm = (data[3] << 8) | data[4];
        break;

    case 0x66: // Post-contactor voltage (62 DD 66), raw in dV
        if (len >= 5)
        {
            postContactorVoltage_dV = (data[3] << 8) | data[4];

            // Close confirmation: the post-contactor rail tracking pack
            // voltage within 10% proves the contactors physically closed.
            if (contactorCloseReq && packVoltage_dV > 100)
            {
                int32_t delta = (int32_t)packVoltage_dV - (int32_t)postContactorVoltage_dV;
                if (delta < 0) delta = -delta;
                if (delta < packVoltage_dV / 10)
                    contactorsConfirmedClosed = true;
            }
        }
        break;

    case 0x6B: // Balancing routine response (71 03 AD 6B status)
        if (len >= 5 && data[0] == 0x71 && data[1] == 0x03 &&
            data[2] == 0xAD && data[3] == 0x6B)
            balancingStatus = data[4];
        break;

    case 0x02: // DTC response (SF 59 02 FF or MF 59 02 ...)
        if (len >= 2 && data[0] == 0x59 && data[1] == 0x02)
            timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;
        break;

    // ---- Multi-frame responses ----
    case 0xA5: // Cell voltages — placeholder (use CELLSUMMARY instead)
        break;

    case 0xA0: // Cell summary (min/max voltage + temps)
        if (len >= 13)
        {
            uint16_t minRaw = (data[9] << 8) | data[10];
            uint16_t maxRaw = (data[11] << 8) | data[12];
            if (minRaw != 0xFFFF && maxRaw != 0xFFFF && minRaw != 0 && maxRaw != 0)
            {
                minCellVoltage_mV = (minRaw + 5) / 10;  // round to nearest mV
                maxCellVoltage_mV = (maxRaw + 5) / 10;
            }
        }
        break;

    case 0x7E: // Voltage limits
        if (len >= 7)
        {
            maxDesignVoltage_dV = ((data[3] << 8) | data[4]) / 10;
            minDesignVoltage_dV = ((data[5] << 8) | data[6]) / 10;
            packLimitAvailable = true;
        }
        break;

    case 0x7D: // Current limits
        if (len >= 7)
        {
            maxChargeAmps    = (int16_t)(((data[3] << 8) | data[4]) / 10);
            maxDischargeAmps = (int16_t)(((data[5] << 8) | data[6]) / 10);
        }
        break;

    case 0x6A: // Isolation reading 1
        if (len >= 9)
        {
            isoExtKOhm = (data[3] << 8) | data[4];
            isoIntKOhm = (data[7] << 8) | data[8];
        }
        break;

    case 0xC0: // Cell temperatures (UDS DD C0) — response layout unverified,
               // so it is intentionally NOT parsed. The previous decode read
               // the same bytes for min and max. Temps come from 0x1FA instead.
        break;

    default:
        break;
    }
}

/*===========================================================================
 * CAN Frame Transmission Helper
 *===========================================================================*/
static void SendCAN(CanHardware* can, uint32_t id, const uint8_t* data, uint8_t dlc)
{
    if (!can) return;
    uint8_t buf[8] = {0};
    for (int i = 0; i < dlc && i < 8; i++) buf[i] = data[i];
    can->Send(id, (uint32_t*)buf, dlc);
}

/*===========================================================================
 * BMS Interface - SetCanInterface
 *===========================================================================*/
void BmwPhevBMS::SetCanInterface(CanHardware* c)
{
    can = c;
    bmsCan = c;
    instance = this;

    // Initialise the shared UDS handler
    uds.Init(c, 0x6F1, 0x607, OnUdsResponse, nullptr);

    // Register all CAN IDs we need to receive
    can->RegisterUserMessage(0x112);  // Pack voltage/current (20ms)
    can->RegisterUserMessage(0x2F5);  // Charge/discharge limits (100ms)
    can->RegisterUserMessage(0x239);  // Predicted energy (200ms)
    can->RegisterUserMessage(0x40D);  // Available power (1s)
    can->RegisterUserMessage(0x430);  // Prediction voltages (1s)
    can->RegisterUserMessage(0x431);  // Battery unit data (200ms)
    can->RegisterUserMessage(0x432);  // SOC info (200ms)
    can->RegisterUserMessage(0x1FA);  // Status / temps (1s)
    can->RegisterUserMessage(0x607);  // UDS responses
}

/*===========================================================================
 * Static Shunt Interface - RegisterCanMessages (matching SBOX pattern)
 *===========================================================================*/
void BmwPhevBMS::RegisterCanMessages(CanHardware* can)
{
    bmsCan = can;

    // Ensure UDS handler uses the shunt CAN interface
    uds.Init(can, 0x6F1, 0x607, OnUdsResponse, nullptr);

    can->RegisterUserMessage(0x112);
    can->RegisterUserMessage(0x2F5);
    can->RegisterUserMessage(0x239);
    can->RegisterUserMessage(0x40D);
    can->RegisterUserMessage(0x430);
    can->RegisterUserMessage(0x431);
    can->RegisterUserMessage(0x432);
    can->RegisterUserMessage(0x1FA);
    can->RegisterUserMessage(0x607);
}

/*===========================================================================
 * BMS Interface - DecodeCAN (uint8_t* version)
 *===========================================================================*/
void BmwPhevBMS::DecodeCAN(int id, uint8_t* data)
{
    switch (id)
    {
    case 0x112: // [20ms] Status Of High-Voltage Battery 2
        batteryAwake = true;
        packVoltage_dV = (data[3] << 8) | data[2];                    // dV directly
        packCurrent_dA = (int32_t)(((data[1] << 8) | data[0]) - 8192); // 0.1A/bit, offset 8192
        Voltage = packVoltage_dV;
        Amperes = packCurrent_dA; // deciAmps
        timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

        // Close confirmation: >5A can only flow through closed contactors.
        // Keeps confirmation alive while driving even if UDS polling wedges.
        if (contactorCloseReq && (packCurrent_dA > 50 || packCurrent_dA < -50))
            contactorsConfirmedClosed = true;

        // SME contactor open requests (byte5 bits6-7, byte6 bits0-3)
        // 00=no statement, 01=not active, 10=active, 11=invalid
        {
            uint8_t openReq = (data[5] & 0xC0) >> 6;          // battery_request_open_contactors
            uint8_t openReqInst = (data[6] & 0x03);            // battery_request_open_contactors_instantly
            uint8_t openReqFast = (data[6] & 0x0C) >> 2;       // battery_request_open_contactors_fast
            if (openReq == 2 || openReqInst == 2 || openReqFast == 2)
                smeDemandsOpen = true;  // SME wants contactors open — safe shutdown in ControlContactors
            // "Signal invalid" (11) on any field: don't trip a shutdown, but
            // stop charging until the SME reports valid data again.
            smeSignalsInvalid = (openReq == 3 || openReqInst == 3 || openReqFast == 3);
        }
        break;

    case 0x2F5: // [100ms] Charge/Discharge Limitations
        // maxChargeVoltage = (data[1]<<8)|data[0]  (not stored currently)
        maxChargeAmps = (int16_t)((((data[3] << 8) | data[2]) - 8192) / 10);
        // minDischargeVoltage = (data[5]<<8)|data[4] (not stored currently)
        maxDischargeAmps = (int16_t)((((data[7] << 8) | data[6]) - 8192) / 10);
        break;

    case 0x239: // [200ms] Predicted energy
        predictedEnergyRemaining_Wh = ((uint16_t)data[2] << 8) | data[1];
        break;

    case 0x40D: // [1s] Available power (*3 = W)
        availChargePowerShort_W    = ((data[1] << 8) | data[0]) * 3;
        availDischargePowerShort_W = ((data[3] << 8) | data[2]) * 3;
        availChargePowerLong_W     = ((data[5] << 8) | data[4]) * 3;
        availDischargePowerLong_W  = ((data[7] << 8) | data[6]) * 3;
        break;

    case 0x430: // [1s] Prediction voltages
        break;

    case 0x431: // [200ms] Battery unit data
        // Energy content maximum: low byte5, high nibble of byte6 (raw/50 = kWh)
        energyMax_50Wh = (((uint16_t)(data[6] & 0x0F)) << 8) | data[5];
        break;

    case 0x432: // [200ms] SOC info
        batteryAwake = true;
        displaySoc = data[4]; // 0.5%/bit
        break;

    case 0x1FA: // [1s] Status / temps
        if (data[6] > 0 && data[6] < 255)
            minTemperature_C = (int16_t)data[6] - 50;
        if (data[7] > 0 && data[7] < 255)
            maxTemperature_C = (int16_t)data[7] - 50;
        Temperature = maxTemperature_C;
        break;

    case 0x607: // UDS Response — delegated to shared UdsHandler.
        // Single dispatch is guaranteed by CanCallback: the static shunt
        // path is skipped when this instance is also the selected BMS.
        uds.HandleResponse(data, 8);
        timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;
        break;

    default:
        break;
    }
}

/*===========================================================================
 * Static Shunt Interface - DecodeCAN (uint32_t version, matching SBOX pattern)
 *===========================================================================*/
void BmwPhevBMS::DecodeCAN(int id, uint32_t data[2])
{
    if (instance)
        instance->DecodeCAN(id, (uint8_t*)data);
}

/*===========================================================================
 * BMS Interface - MaxChargeCurrent
 *===========================================================================*/
float BmwPhevBMS::MaxChargeCurrent()
{
    // SME demands contactors open (or has demanded it this drive cycle),
    // or its open-request signals read invalid — cut all current
    if (smeDemandsOpen || smeEmergencyLatched || smeSignalsInvalid) return 0;

    if (timeoutCounter < 1) return 0;

    // Cell data frozen while current flows — can't trust the limits
    if (cellDataStale) return 0;

    // Pack overvoltage vs SME-reported design limit (UDS 0xDD7E)
    if (packLimitAvailable && packVoltage_dV > maxDesignVoltage_dV) return 0;

    // Check voltage/temperature limits from params
    float vmax = Param::GetFloat(Param::BMS_VmaxLimit);
    float vmin = Param::GetFloat(Param::BMS_VminLimit);
    float tmax = Param::GetFloat(Param::BMS_TmaxLimit);
    float tmin = Param::GetFloat(Param::BMS_TminLimit);

    if (maxCellVoltage_mV > vmax * 1000.0f) return 0;
    if (minCellVoltage_mV < vmin * 1000.0f) return 0;
    if (maxTemperature_C > tmax) return 0;
    if (minTemperature_C < tmin) return 0;

    // Use the BMS-reported charge current limit (from 0x2F5 or UDS).
    // No valid limit data means something is wrong (0x2F5 broadcasts at
    // 100ms whenever the SME is awake) — don't charge blind.
    if (maxChargeAmps > 0 && maxChargeAmps < 500)
        return (float)maxChargeAmps;

    return 0;
}

/*===========================================================================
 * BMS Interface - Task100Ms (core timed loop)
 *
 * This is called every 100ms by the VCU scheduler.
 * All timed actions (CAN TX, UDS polling, timeouts) derive from this.
 *===========================================================================*/
void BmwPhevBMS::Task100Ms()
{
    // ---- Timeout ----
    if (timeoutCounter > 0) timeoutCounter--;

    // ---- One-shot silence countdown ----
    if (udsOneShotPending)
    {
        if (udsOneShotSilence > 0)
            udsOneShotSilence--;
        else
            udsOneShotPending = false;
    }

    // ---- UDS transport timeout (shared handler) ----
    uds.Tick100Ms();

    // ---- SME wakeup via 0x554 magic packet at 100kbps ----
    // TJA1055 remote wake timing requires dominant >38µs; at 500kbps a bit
    // is only 2µs, too short.  100kbps gives 10µs/bit → 4 bits = 40µs > 38µs.
    // While the bus is at 100kbps, wakeInProgress suppresses ALL other TX
    // on this bus (here and in ControlContactors) — transmitting 500kbps
    // frames at the wrong bit rate causes error-frame storms and can push
    // the CAN peripheral towards error-passive/bus-off.
    if (wakeInProgress)
    {
        // 1 × 100ms elapsed at 100kbps — always restore the bus speed,
        // even if batteryAwake flipped in the meantime
        if (bmsCan) bmsCan->SetBaudrate(CanHardware::Baud500);
        wakeInProgress = false;
    }
    else if (!batteryAwake && tickCounter == 0 && bmsCan)
    {
        wakeInProgress = true;
        bmsCan->SetBaudrate(CanHardware::Baud100);
        const uint8_t wakeup[4] = {0x5A, 0xA5, 0x5A, 0xA5};
        SendCAN(bmsCan, 0x554, wakeup, 4);
        SendCAN(bmsCan, 0x554, wakeup, 4);
    }

    // ---- Normal TX runs regardless of batteryAwake for debugging ----
    // Once the SME responds to any frame (0x112/0x432/0x607), batteryAwake
    // flips true and all data flows normally. Suppressed only while the
    // wakeup has the bus at 100kbps.
    if (!wakeInProgress)
    {
        // ---- Boot: send one balancing stop to clear any latched routine ----
        if (!bootBalancingStopSent && !udsOneShotPending)
        {
            // Also clear the user balancing param: it's saveable, and booting
            // with it ON would fire a start burst that blocks contactor close.
            Param::SetInt(Param::BMS_BalancingOn, 0);
            lastBalancingOn = false;
            uds.Abort();  // cancel any in-progress UDS transfer
            SendCAN(bmsCan, 0x6F1, UDS_BALANCING_STOP, 8);
            bootBalancingStopSent = true;
            udsOneShotPending = true;
            udsOneShotSilence = 10; // 1s silence
        }

        // ---- Pre-close balancing stop phase ----
        if (preCloseStopsRemaining > 0 && !udsOneShotPending)
        {
            uds.Abort();  // cancel any in-progress UDS transfer
            SendCAN(bmsCan, 0x6F1, UDS_BALANCING_STOP, 8);
            preCloseStopsRemaining--;
            udsOneShotPending = true;
            udsOneShotSilence = 10; // 1s silence
        }

        // ---- User-requested balancing on/off burst (driven by BMS_BalancingOn param) ----
        // Edge detection each call: when the user toggles the web UI param,
        // fire a burst of START or STOP frames to latch the routine in the SME.
        {
            bool balOn = (Param::GetInt(Param::BMS_BalancingOn) != 0);
            if (balOn != lastBalancingOn)
            {
                lastBalancingOn = balOn;
                balancingBurstIsStart = balOn;
                balancingBurstRemaining = BALANCING_BURST_COUNT;
            }
        }
        if (balancingBurstRemaining > 0 && !udsOneShotPending)
        {
            uds.Abort();  // cancel any in-progress UDS
            SendCAN(bmsCan, 0x6F1, balancingBurstIsStart ? UDS_BALANCING_START : UDS_BALANCING_STOP, 8);
            balancingBurstRemaining--;
            udsOneShotPending = true;
            udsOneShotSilence = 10; // 1s silence
        }

        // ---- 0x12F Terminal Status (100ms) ----
        {
            static uint8_t alive100 = 0;
            uint8_t frame12F[8] = {0x00, 0x20, 0x86, 0x1B, 0xF1, 0x35, 0x30, 0x02};
            frame12F[1] = 0x20 + alive100;
            frame12F[0] = Crc8SAEJ1850(&frame12F[1], 7, 0x3F);
            SendCAN(bmsCan, 0x12F, frame12F, 8);
            alive100 = IncrementAlive(alive100);
        }

#if PHEV_DIAG_TX
        // ---- Diagnostic frame 0x7E0 (8 bytes, 100ms) ----
        // Byte 0: poll/block status   hi-nibble=tickCounter, lo-nibble=reason
        //          0x0=free, 0x1=udsOneShotPending, 0x2=udsCtx.inProgress,
        //          0x3=!bmsCan, 0x4=odd tick, 0xA+N=preCloseStops(N=1..3)
        // Byte 1: contactor flags     80=closed 40=closeReq 20=shutdownPend
        //          10=smeOpen 0C=phev53aState 02=battAwake 01=allowClose
        // Byte 2-3: packVoltage_dV (uint16 LE)
        // Byte 4-5: minCellVoltage_mV (uint16 LE, raw from CELLSUMMARY)
        // Byte 6: displaySoc / stuck moduleID
        // Byte 7: maxTemperature_C (int8)
        {
            uint8_t diag[8] = {0};

            // Byte 0: poll reason + tick
            uint8_t reason = 0x00;
            if (preCloseStopsRemaining > 0)
                reason = 0x0A + preCloseStopsRemaining;  // 0x0B..0x0D
            else if (udsOneShotPending)
                reason = 0x01;
            else if (uds.IsBusy())
                reason = 0x02;
            else if (!bmsCan)
                reason = 0x03;
            else if (tickCounter & 1)
                reason = 0x04;
            diag[0] = ((tickCounter & 0x0F) << 4) | (reason & 0x0F);

            // Byte 1: contactor state
            bool allowClose = contactorCloseReq &&
                              (startupCounter >= 320) &&
                              (phev53aState == ST53A_ESCALATED || phev53aState == ST53A_STEADY) &&
                              (preCloseStopsRemaining == 0) &&
                              !smeDemandsOpen &&
                              !smeEmergencyLatched &&
                              !shutdownPending;
            uint8_t state = 0;
            if (AreContactorsClosed()) state |= 0x80;
            if (contactorCloseReq)      state |= 0x40;
            if (shutdownPending)        state |= 0x20;
            if (smeDemandsOpen)         state |= 0x10;
            state |= ((uint8_t)phev53aState & 0x03) << 2;
            if (batteryAwake)           state |= 0x02;
            if (allowClose)             state |= 0x01;
            diag[1] = state;

            // Bytes 2-3: minCellVoltage_mV (uint16 LE)
            uint16_t cellMin_mV = (uint16_t)minCellVoltage_mV;
            diag[2] = (uint8_t)(cellMin_mV & 0xFF);
            diag[3] = (uint8_t)((cellMin_mV >> 8) & 0xFF);

            // Bytes 4-5: maxCellVoltage_mV (uint16 LE)
            uint16_t cellMax_mV = (uint16_t)maxCellVoltage_mV;
            diag[4] = (uint8_t)(cellMax_mV & 0xFF);
            diag[5] = (uint8_t)((cellMax_mV >> 8) & 0xFF);

            // Byte 6: displaySoc normally; stuck UDS moduleID when pollReason=0x02
            if (uds.IsBusy())
                diag[6] = uds.GetModuleID();  // e.g. 0xA0=CELLSUMMARY, 0x7E=VoltLimits, 0xC4=SOC
            else
                diag[6] = (uint8_t)displaySoc;

            // Byte 7: max temperature (°C, signed)
            diag[7] = (uint8_t)(int8_t)maxTemperature_C;

            SendCAN(bmsCan, 0x7E0, diag, 8);
        }
#endif // PHEV_DIAG_TX

        // ---- UDS Fast Poll (every 200ms = every 2nd call) ----
        // A 1-tick silence after each send prevents the slow poll from
        // colliding on tick 0 and gives the SME time to respond.
        if ((tickCounter & 1) == 0 && tickCounter != 0 && !udsOneShotPending && bmsCan)
        {
            // While waiting for close confirmation, poll post-contactor
            // voltage every slot instead of rotating, so confirmation isn't
            // delayed by the rotation and precharge can complete within the
            // VCU's 5s precharge timeout.
            if (contactorCloseReq && !contactorsConfirmedClosed)
            {
                if (uds.SendRequest(UDS_POST_VOLTAGE, 5))
                {
                    udsOneShotPending = true;
                    udsOneShotSilence = 1; // 100ms
                }
            }
            else if (uds.SendRequest(fastReqs[fastPollIndex], 5))
            {
                fastPollIndex++;
                if (fastPollIndex >= numFastReqs) fastPollIndex = 0;
                udsOneShotPending = true;
                udsOneShotSilence = 1; // 100ms
            }
        }

        // ---- 0x53A Contactor State (200ms = every 2nd call) ----
        if ((tickCounter & 1) == 0 && bmsCan)
        {
            uint8_t frame53A[8] = {0x40, 0x3A, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00};

            if (!contactorCloseReq)
            {
                phev53aState = ST53A_IDLE;
                stateBHoldCount = 0;
            }
            else
            {
                switch (phev53aState)
                {
                case ST53A_IDLE:
                    phev53aState = ST53A_CLOSE_REQ;
                    stateBHoldCount = 5; // Hold STATE B for 5 * 200ms = 1s
                    break;
                case ST53A_CLOSE_REQ:
                    if (stateBHoldCount > 0)
                        stateBHoldCount--;
                    if (stateBHoldCount == 0)
                        phev53aState = ST53A_ESCALATED; // B held long enough — escalate
                    break;
                case ST53A_ESCALATED:
                    phev53aState = ST53A_STEADY; // Single transient — settle to steady
                    break;
                case ST53A_STEADY:
                    break;
                }
            }

            switch (phev53aState)
            {
            case ST53A_IDLE:      frame53A[5] = 0x00; frame53A[6] = 0x01; break;
            case ST53A_CLOSE_REQ: frame53A[5] = 0x80; frame53A[6] = 0x01; break;
            case ST53A_ESCALATED: frame53A[5] = 0x84; frame53A[6] = 0x01; break;
            case ST53A_STEADY:    frame53A[5] = 0x84; frame53A[6] = 0x00; break;
            }
            SendCAN(bmsCan, 0x53A, frame53A, 8);
        }

        // ---- 1-second tasks (every 10th call) ----
        if (tickCounter == 0)
        {
            // UDS Slow Poll
            if (!udsOneShotPending && bmsCan)
            {
                uint8_t dlc = (slowReqs[slowPollIndex] == UDS_BALANCING_STATUS ||
                               slowReqs[slowPollIndex] == UDS_READ_DTC) ? 8 : 5;
                if (uds.SendRequest(slowReqs[slowPollIndex], dlc))
                {
                    slowPollIndex++;
                    if (slowPollIndex >= numSlowReqs) slowPollIndex = 0;
                }
            }

            // Vehicle environment frames
            SendCAN(bmsCan, 0x3A0, VEH_3A0, 8);
            SendCAN(bmsCan, 0x3CA, VEH_3CA, 8);
            SendCAN(bmsCan, 0x3E8, VEH_3E8, 2);
            SendCAN(bmsCan, 0x2CA, VEH_2CA, 2);

            // 0x328 Relative time (simple incrementing seconds + day counter)
            {
                static uint32_t clockSeconds = 243785948;
                static uint16_t clockDays = 9244;
                static uint32_t secondsToDay = 0;
                clockSeconds++;
                secondsToDay++;
                if (secondsToDay > 86400) { clockDays++; secondsToDay = 0; }
                uint8_t frame328[6];
                frame328[0] = (uint8_t)(clockSeconds & 0xFF);
                frame328[1] = (uint8_t)((clockSeconds >> 8) & 0xFF);
                frame328[2] = (uint8_t)((clockSeconds >> 16) & 0xFF);
                frame328[3] = (uint8_t)((clockSeconds >> 24) & 0xFF);
                frame328[4] = (uint8_t)(clockDays & 0xFF);
                frame328[5] = (uint8_t)((clockDays >> 8) & 0xFF);
                SendCAN(bmsCan, 0x328, frame328, 6);
            }
        }
    }

    // ---- Stale cell-data detection ----
    // Cell voltages under load change at mV resolution constantly; both
    // min and max frozen for 60 minutes while >5A flows means the UDS data
    // path is stuck — force the limits invalid so charging stops.
    // (Ported from Battery-Emulator isStale, STALE_PERIOD_CONFIG = 60 min.)
    if (minCellVoltage_mV != staleLastMinCell || maxCellVoltage_mV != staleLastMaxCell)
    {
        staleLastMinCell = minCellVoltage_mV;
        staleLastMaxCell = maxCellVoltage_mV;
        cellStaleTicks = 0;
    }
    else if (cellStaleTicks < 60000)
    {
        cellStaleTicks++;
    }
    cellDataStale = (cellStaleTicks >= 36000) && // 36000 × 100ms = 60 min
                    (packCurrent_dA > 50 || packCurrent_dA < -50);

    // CAN silence — any close confirmation is no longer trustworthy
    if (timeoutCounter == 0)
        contactorsConfirmedClosed = false;

    // ---- Update BMS Params (every call) ----
    if (timeoutCounter > 0)
    {
        if (smeDemandsOpen || cellDataStale)
        {
            // SME emergency or frozen data — signal invalid cell voltages
            // to any safety monitors
            Param::SetFloat(Param::BMS_Vmin, 99.0f);
            Param::SetFloat(Param::BMS_Vmax, -1.0f);
        }
        else
        {
            Param::SetFloat(Param::BMS_Vmin, minCellVoltage_mV / 1000.0f);
            Param::SetFloat(Param::BMS_Vmax, maxCellVoltage_mV / 1000.0f);
        }
        Param::SetFloat(Param::BMS_Tmin, (float)minTemperature_C);
        Param::SetFloat(Param::BMS_Tmax, (float)maxTemperature_C);
        Param::SetInt(Param::BMS_ChargeLim, (int)MaxChargeCurrent());

        // SOC — broadcast 0x432 byte[4] (0.5%/bit), available every 200ms.
        // 0xFF = signal invalid / not available; fall back to UDS 0xDDC4.
        if (displaySoc > 0 && displaySoc < 0xFF)
            Param::SetFloat(Param::SOC, displaySoc * 0.5f);
        else if (avgSocState > 0 && avgSocState < 65535)
            Param::SetFloat(Param::SOC, avgSocState / 100.0f);

        // Average battery temperature
        Param::SetFloat(Param::BMS_Tavg, (minTemperature_C + maxTemperature_C) / 2.0f);

        // Power limits from SME broadcast 0x40D (W → kW)
        if (availChargePowerLong_W > 0)
            Param::SetFloat(Param::BMS_MaxInput, availChargePowerLong_W / 1000.0f);
        if (availDischargePowerLong_W > 0)
            Param::SetFloat(Param::BMS_MaxOutput, availDischargePowerLong_W / 1000.0f);

        // Max charge power in W (short-term limit from SME)
        if (availChargePowerShort_W > 0)
            Param::SetFloat(Param::BMS_MaxCharge, (float)availChargePowerShort_W);

        // Isolation resistance (kOhm) — internal and external from UDS 0xDD6A
        if (isoExtKOhm > 0)
            Param::SetFloat(Param::BMS_IsolationExt, (float)isoExtKOhm);
        if (isoIntKOhm > 0)
            Param::SetFloat(Param::BMS_IsolationInt, (float)isoIntKOhm);

        // Balancing status (UDS 0xAD6B: 0=inactive not needed, 1=active, 2=not resting,
        // 3=inactive, 4=unknown/qualifier invalid)
        Param::SetInt(Param::BMS_Balancing, (int)balancingStatus);

        // Pack power (V × A, W → kW)
        float packPower_kW = (packVoltage_dV / 10.0f) * (packCurrent_dA / 10.0f) / 1000.0f;
        Param::SetFloat(Param::power, packPower_kW);

        // Battery capacity from 0x431 energy max (raw/50 = kWh)
        if (energyMax_50Wh > 0)
            Param::SetFloat(Param::BattCap, energyMax_50Wh / 50.0f);

        // Remaining energy (kWh) direct from 0x239, amp-hours derived
        if (predictedEnergyRemaining_Wh > 0)
        {
            float remaining_kWh = predictedEnergyRemaining_Wh / 1000.0f;
            Param::SetFloat(Param::KWh, remaining_kWh);
            float packVolts = packVoltage_dV / 10.0f;
            if (packVolts > 0.0f)
                Param::SetFloat(Param::AMPh, remaining_kWh * 1000.0f / packVolts);
        }

        // Cell voltage spread (mV)
        Param::SetFloat(Param::BMS_CellDelta, (float)(maxCellVoltage_mV - minCellVoltage_mV));
    }
    else
    {
        Param::SetFloat(Param::BMS_Vmin, 0);
        Param::SetFloat(Param::BMS_Vmax, 0);
        Param::SetFloat(Param::BMS_Tmin, 0);
        Param::SetFloat(Param::BMS_Tmax, 0);
        Param::SetInt(Param::BMS_ChargeLim, 0);
    }

    // ---- Increment tick counter (0..9) ----
    tickCounter++;
    if (tickCounter >= 10) tickCounter = 0;
}

/*===========================================================================
 * Static Contactor/Shunt Interface - ControlContactors
 *
 * Called every 10ms from Ms10Task with the current opmode:
 *   0 = MOD_OFF       → open contactors
 *   1 = MOD_RUN       → close contactors
 *   2 = MOD_PRECHARGE → close contactors
 *   3 = MOD_PCHFAIL   → open contactors
 *   4 = MOD_CHARGE    → close contactors
 *===========================================================================*/
void BmwPhevBMS::ControlContactors(int opmode, CanHardware* can)
{
    // Sync bmsCan to the shunt CAN interface every call so Task100Ms
    // UDS polls use the same proven-working interface as 0x10B/0x12F/0x53A.
    if (can) bmsCan = can;

    // Map opmode to desired contactor state
    bool requestClose = false;
    switch (opmode)
    {
    case 1: // MOD_RUN
    case 2: // MOD_PRECHARGE
    case 4: // MOD_CHARGE
        requestClose = true;
        break;
    default:
        requestClose = false;
        break;
    }

    // SME emergency latch: cleared only once the VCU stops requesting close
    // (opmode left RUN/PRECHARGE/CHARGE, i.e. a key/charge cycle). While
    // latched, no new close request is accepted — without this, an
    // SME-demanded open in RUN would be followed by an immediate automatic
    // re-close attempt on the next 10ms tick.
    if (!requestClose)
        smeEmergencyLatched = false;

    // Handle state transitions
    if (requestClose && !contactorCloseReq && !smeEmergencyLatched)
    {
        // Request close: start pre-close balancing stop.
        // Reset UDS context in case a previous multi-frame transfer
        // was stalled (e.g. flow control went to dead bmsCan before
        // the ControlContactors sync fix).
        // Also auto-clear the user balancing toggle — balancing MUST
        // be off before the SME allows contactor close.
        Param::SetInt(Param::BMS_BalancingOn, 0);
        lastBalancingOn = false;
        balancingBurstRemaining = 0;
        contactorCloseReq = true;
        contactorsConfirmedClosed = false; // fresh cycle — must re-confirm
        shutdownPending = false;
        smeOpenDelayTicks = 0;
        preCloseStopsRemaining = PRE_CLOSE_STOP_COUNT;
        uds.Abort();
        udsOneShotPending = false;
        udsOneShotSilence = 0;
    }
    else if (!requestClose && contactorCloseReq && !shutdownPending)
    {
        // Request open — enter safe shutdown: wait for current to decay before opening
        shutdownPending = true;
        smeOpenDelayTicks = 0;
    }

    // ---- Safe contactor open (shared: normal shutdown + SME emergency) ----
    // Must NOT open contactors under load:
    //   1) cut current limits (done in MaxChargeCurrent)
    //   2) wait for |pack current| < 5A
    //   3) hold for 500ms (50 ticks × 10ms) to ensure current has truly decayed
    //   4) then force contactors open
    if ((smeDemandsOpen || shutdownPending) && contactorCloseReq)
    {
        int32_t absCurrent = packCurrent_dA;
        if (absCurrent < 0) absCurrent = -absCurrent;

        if (absCurrent < 50) // Current below 5A (50 dA) — safe to open
        {
            if (smeOpenDelayTicks < 50) // 50 * 10ms = 500ms hold-off
                smeOpenDelayTicks++;
        }
        else
        {
            smeOpenDelayTicks = 0; // Current still flowing — reset timer
        }

        if (smeOpenDelayTicks >= 50)
        {
            // Current has been low for 500ms — safe to open.
            // An SME-demanded open is latched: no automatic re-close until
            // the VCU opmode leaves RUN/CHARGE (see latch clear above).
            if (smeDemandsOpen)
                smeEmergencyLatched = true;
            contactorCloseReq = false;
            contactorsConfirmedClosed = false;
            phev53aState = ST53A_IDLE;
            smeDemandsOpen = false;
            shutdownPending = false;
            smeOpenDelayTicks = 0;
        }
    }

    // Startup counter (allow ~3.2s boot delay before closing)
    // ControlContactors is called every 10ms, so 320 * 10ms = 3.2s
    if (startupCounter < 320)
        startupCounter++;

    // ---- 0x10B Contactor Driver (send at 20ms = every 2nd call) ----
    ms10Counter++;
    if (ms10Counter >= 2 && !wakeInProgress) // no TX while bus is at 100kbps for wakeup
    {
        ms10Counter = 0;

        static uint8_t alive20 = 0;

        // Allow close only when:
        //   - contactor close requested
        //   - startup delay elapsed
        //   - 0x53A state machine reached ESCALATED or STEADY
        //   - pre-close balancing stops all sent
        //   - no SME emergency open latched this drive cycle
        bool allowClose = contactorCloseReq &&
                          (startupCounter >= 320) &&
                          (phev53aState == ST53A_ESCALATED || phev53aState == ST53A_STEADY) &&
                          (preCloseStopsRemaining == 0) &&
                          !smeDemandsOpen &&
                          !smeEmergencyLatched &&
                          !shutdownPending;

        uint8_t frame10B[3] = {0x00, 0x00, 0xFC};
        if (allowClose)
            frame10B[1] = 0x10; // Close contactors
        else
            frame10B[1] = 0x00; // Open

        frame10B[1] = (frame10B[1] & 0xF0) | alive20;
        // CRC over bytes 1..2 with init 0x3F, store in byte 0
        frame10B[0] = Crc8SAEJ1850(&frame10B[1], 2, 0x3F);

        SendCAN(can, 0x10B, frame10B, 3);
        alive20 = IncrementAlive(alive20);
    }
}

/*===========================================================================
 * AreContactorsClosed — public accessor for ProcessUdc precharge gating.
 * Returns true only when 0x10B is actively commanding close, the 0x53A
 * state machine has reached STEADY (STATE D), AND measured feedback
 * confirms the contactors physically closed: post-contactor voltage
 * (UDS 0xDD66) tracking pack voltage within 10%, or >5A pack current.
 *
 * Commanding alone is not enough — an SME that refuses to close (latched
 * balancing, DTC, HVIL) must leave udc at 0 so the VCU stays in precharge
 * and times out into PCHFAIL instead of entering RUN on a dead bus.
 *===========================================================================*/
bool BmwPhevBMS::AreContactorsClosed()
{
    return contactorCloseReq &&
           phev53aState == ST53A_STEADY &&
           contactorsConfirmedClosed;
}
