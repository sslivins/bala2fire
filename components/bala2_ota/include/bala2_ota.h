// Tiny HTTP server with an OTA endpoint.
//
// Endpoints:
//   GET  /          -> short HTML status page (firmware version, slot, uptime)
//   GET  /info      -> JSON: { "version", "running_slot", "uptime_ms" }
//   POST /update    -> stream the request body (a raw .bin) into the inactive
//                      OTA partition, mark it for next boot, reboot.
//                      Refuses if the balancer is currently ARMED.
//
// The same server can later host motion / control endpoints; this component
// just owns the lifecycle and the OTA bits.

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Start the HTTP server. Call after Wi-Fi reaches CONNECTED.
// Idempotent: extra calls return ESP_OK without restarting.
esp_err_t bala2_ota_start(void);

// Mark the freshly-OTA'd image as good. Call once after app_main has
// confirmed core subsystems are healthy. If you don't call this within
// the rollback window after a fresh OTA, the bootloader rolls back on
// next reset.
void bala2_ota_mark_valid(void);

#ifdef __cplusplus
}
#endif
