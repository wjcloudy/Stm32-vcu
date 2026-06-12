/*
 * UDS / ISO-TP Handler — implementation
 * See uds_handler.h for usage documentation.
 */

#include "uds_handler.h"
#include "canhardware.h"
#include <cstring>

/*===========================================================================
 * Construction / Init
 *===========================================================================*/
UdsHandler::UdsHandler()
{
}

void UdsHandler::Init(CanHardware* c, uint32_t requestId, uint32_t responseId,
                      ResponseCallback cb, void* ctx)
{
    can = c;
    reqId = requestId;
    respId = responseId;
    callback = cb;
    callbackCtx = ctx;
}

/*===========================================================================
 * CAN TX helper
 *===========================================================================*/
void UdsHandler::SendCan(uint32_t id, const uint8_t* data, uint8_t dlc)
{
    if (!can) return;
    uint8_t buf[8] = {0};
    for (int i = 0; i < dlc && i < 8; i++) buf[i] = data[i];
    can->Send(id, (uint32_t*)buf, dlc);
}

/*===========================================================================
 * Send a UDS request
 *===========================================================================*/
bool UdsHandler::SendRequest(const uint8_t* payload, uint8_t len)
{
    if (!can) return false;
    if (transfer.inProgress) return false;

    SendCan(reqId, payload, len);
    return true;
}

/*===========================================================================
 * Periodic 100ms tick — timeout management
 *===========================================================================*/
void UdsHandler::Tick100Ms()
{
    // Hard timeout for stuck multi-frame transfers. stuckTicks is reset on
    // every received FF/CF, so this is a per-frame gap timeout: a lost frame
    // can't wedge polling permanently.
    if (transfer.inProgress)
    {
        transfer.stuckTicks++;
        if (transfer.stuckTicks > 20)  // 20 × 100ms = 2s without a frame
        {
            Abort();
        }
    }
}

/*===========================================================================
 * Forced abort — drop any in-progress transfer.
 * Partial data is discarded, NOT delivered: parsers expect complete,
 * validated payloads.
 *===========================================================================*/
void UdsHandler::Abort()
{
    transfer.inProgress = false;
    transfer.bytesReceived = 0;
    transfer.stuckTicks = 0;
}

/*===========================================================================
 * ISO-TP Multi-Frame Helpers
 *===========================================================================*/
void UdsHandler::StartMultiFrame(uint16_t totalLen, uint8_t moduleID)
{
    transfer.inProgress = true;
    transfer.expectedLength = totalLen;
    transfer.bytesReceived = 0;
    transfer.moduleID = moduleID;
    transfer.receivedInBatch = 0;
    transfer.stuckTicks = 0;
    memset(transfer.buffer, 0, sizeof(transfer.buffer));
}

bool UdsHandler::StorePayload(const uint8_t* payload, uint8_t length)
{
    if (transfer.bytesReceived + length > sizeof(transfer.buffer))
    {
        // Overflow — abort
        transfer.inProgress = false;
        return false;
    }
    memcpy(&transfer.buffer[transfer.bytesReceived], payload, length);
    transfer.bytesReceived += length;
    transfer.stuckTicks = 0;  // reset hard timeout on each CF/FF arrival

    // Check completion
    if (transfer.bytesReceived >= transfer.expectedLength)
        transfer.inProgress = false;
    return true;
}

void UdsHandler::SendFlowControl()
{
    // BMW ISO-TP flow control: block size 3, separation time 0
    static const uint8_t fc[8] = {0x07, 0x30, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
    SendCan(reqId, fc, 8);
}

/*===========================================================================
 * Process a completed multi-frame transfer
 *===========================================================================*/
void UdsHandler::ProcessCompletedTransfer()
{
    if (!transfer.inProgress && transfer.bytesReceived > 0 && callback)
    {
        callback(transfer.moduleID, transfer.buffer, transfer.bytesReceived, callbackCtx);
        transfer.bytesReceived = 0;  // consumed
    }
}

/*===========================================================================
 * CAN RX — handle incoming UDS response frame (0x607)
 * Extended addressing: data[0] = target, data[1] = PCI
 *===========================================================================*/
void UdsHandler::HandleResponse(const uint8_t* data, uint8_t dlc)
{
    if (dlc < 2) return;

    uint8_t pciByte = data[1];
    uint8_t pciType = pciByte >> 4;

    switch (pciType)
    {
    case 0x0: // Single Frame
    {
        // SF payload: data[2..7] (up to 6 bytes after ext-addr + PCI).
        // For BMW 0x62 responses: sfBuffer[0]=0x62, [1]=paramHi, [2]=paramLo (moduleID)
        // For 0x71 routine responses: sfBuffer[0]=0x71, [1]=sub, [2]=RIDhi, [3]=RIDlo
        // For 0x59 DTC responses: moduleID=0x02
        // NOTE: SF has no length byte, so the parameter is one byte EARLIER than in FF.
        if (callback)
        {
            uint8_t sfBuffer[6];
            uint8_t sfLen = (dlc > 2) ? ((dlc - 2 < 6) ? (dlc - 2) : 6) : 0;
            if (sfLen > 0)
                memcpy(sfBuffer, &data[2], sfLen);

            uint8_t moduleID = 0;
            if (sfLen >= 3 && sfBuffer[0] == 0x62)
                moduleID = sfBuffer[2];  // SF: paramLo at offset 2 (no length byte)
            else if (sfLen >= 4 && sfBuffer[0] == 0x71)
                moduleID = sfBuffer[3];  // Routine RID lo-byte at offset 3
            else if (sfLen >= 1 && sfBuffer[0] == 0x59)
                moduleID = 0x02;         // DTC response

            callback(moduleID, sfBuffer, sfLen, callbackCtx);
        }
        break;
    }

    case 0x1: // First Frame
    {
        uint16_t totalLen = ((uint16_t)(pciByte & 0x0F) << 8) | data[2];
        uint8_t serviceResponse = (dlc > 3) ? data[3] : 0;
        uint8_t moduleID;

        // Determine module ID based on service response type
        if (serviceResponse == 0x59)
            moduleID = (dlc > 4) ? data[4] : 0;  // DTC sub-function
        else
            moduleID = (dlc > 5) ? data[5] : 0;  // BMW proprietary: parameter lo-byte

        StartMultiFrame(totalLen, moduleID);

        // FF payload starts at data[3], 5 bytes for an 8-byte CAN frame
        uint8_t ffPayloadSize = (dlc > 3) ? ((dlc - 3 < 5) ? (dlc - 3) : 5) : 0;
        if (ffPayloadSize > 0)
            StorePayload(&data[3], ffPayloadSize);

        // Request consecutive frames
        SendFlowControl();
        break;
    }

    case 0x2: // Consecutive Frame
    {
        if (!transfer.inProgress) return;

        // CF payload starts at data[2], up to 6 bytes
        uint8_t cfPayloadSize = (dlc > 2) ? ((dlc - 2 < 6) ? (dlc - 2) : 6) : 0;
        if (cfPayloadSize > 0)
            StorePayload(&data[2], cfPayloadSize);

        transfer.receivedInBatch++;

        if (transfer.receivedInBatch >= 3)
        {
            SendFlowControl();
            transfer.receivedInBatch = 0;
        }

        // Check if transfer completed during this CF
        if (!transfer.inProgress)
            ProcessCompletedTransfer();
        break;
    }

    case 0x3: // Flow Control (from ECU — rare, ignore)
        break;
    }
}
