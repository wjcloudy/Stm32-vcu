/*
 * UDS / ISO-TP Handler for ZombieVerter VCU
 *
 * Provides a reusable UDS-over-CAN transport layer:
 *   - Sends requests on a configurable CAN ID (typically 0x6F1)
 *   - Receives and reassembles multi-frame ISO-TP responses
 *   - Calls a completion callback with the reassembled payload
 *   - Hard timeout prevents stuck transfers from wedging polling
 *
 * Usage:
 *   1. Init(can, callback, ctx) — set CAN interface and callback
 *   2. HandleResponse(data, dlc) — call from CAN RX for every frame
 *      on the response ID (typically 0x607)
 *   3. SendRequest(payload, len) — queue a UDS request
 *   4. Tick100Ms() — call every 100ms for timeout management
 *   5. IsBusy() — true while a multi-frame transfer is in progress;
 *      pollers should check this before sending a new request
 */

#ifndef UDS_HANDLER_H
#define UDS_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

class CanHardware;

class UdsHandler
{
public:
    // Callback type. Called when a complete UDS response is assembled.
    //   moduleID  — BMW proprietary module/parameter identifier
    //   data      — pointer to reassembled payload
    //   len       — total payload length
    //   ctx       — opaque user pointer passed to Init()
    typedef void (*ResponseCallback)(uint8_t moduleID, const uint8_t* data,
                                     uint16_t len, void* ctx);

    UdsHandler();

    // ---- Setup ----
    void Init(CanHardware* can, uint32_t requestId, uint32_t responseId,
              ResponseCallback cb, void* ctx);

    // ---- CAN RX (call from interrupt or task context) ----
    // Feed every frame received on responseId (typically 0x607) here.
    // data[0] is the extended addressing byte (0xF1), data[1] is PCI.
    void HandleResponse(const uint8_t* data, uint8_t dlc);

    // ---- CAN TX (call from poll loop) ----
    // Sends a UDS request frame. Returns false if a multi-frame transfer
    // is already in progress (caller should wait and retry).
    bool SendRequest(const uint8_t* payload, uint8_t len);

    // ---- Periodic (call every 100ms) ----
    void Tick100Ms();

    // ---- State queries ----
    bool IsBusy() const { return transfer.inProgress; }
    uint16_t GetBytesReceived() const { return transfer.bytesReceived; }
    uint16_t GetExpectedLength() const { return transfer.expectedLength; }
    uint8_t  GetModuleID() const { return transfer.moduleID; }

    // ---- Forced abort ----
    void Abort();

private:
    CanHardware* can = nullptr;
    uint32_t reqId = 0x6F1;   // CAN ID for requests
    uint32_t respId = 0x607;  // CAN ID for responses
    ResponseCallback callback = nullptr;
    void* callbackCtx = nullptr;

    // ISO-TP multi-frame transfer state
    struct Transfer
    {
        bool inProgress = false;
        uint16_t expectedLength = 0;
        uint16_t bytesReceived = 0;
        uint8_t moduleID = 0;
        uint8_t receivedInBatch = 0;
        uint8_t buffer[256];
        uint8_t stuckTicks = 0;    // hard timeout counter (100ms units)
    };
    Transfer transfer;

    // Internal helpers
    void StartMultiFrame(uint16_t totalLen, uint8_t moduleID);
    bool StorePayload(const uint8_t* payload, uint8_t length);
    void SendFlowControl();
    void ProcessCompletedTransfer();
    void SendCan(uint32_t id, const uint8_t* data, uint8_t dlc);
};

#endif // UDS_HANDLER_H
