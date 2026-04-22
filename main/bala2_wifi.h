#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Possible Wi-Fi states shown on the display.
typedef enum {
    BALA2_WIFI_NOT_CONFIGURED,   // CONFIG_BALA2_WIFI_SSID is empty
    BALA2_WIFI_CONNECTING,
    BALA2_WIFI_CONNECTED,        // ip_str is valid
    BALA2_WIFI_FAILED,           // timed out or auth failed
} bala2_wifi_state_t;

typedef struct {
    bala2_wifi_state_t state;
    char ip_str[16];             // "192.168.x.y" or ""
} bala2_wifi_status_t;

// Kicks off Wi-Fi connect in the background (non-blocking).
// Safe to call once from app_main() after M5.begin().
void bala2_wifi_start(void);

// Snapshots the current status. Thread-safe.
void bala2_wifi_get_status(bala2_wifi_status_t *out);

#ifdef __cplusplus
}
#endif
