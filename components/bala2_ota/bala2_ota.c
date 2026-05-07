// OTA HTTP server for the Bala2 Fire.
//
// The /update handler streams the POST body straight into the next
// OTA partition without buffering the whole image in RAM (the image
// is ~1 MB; we have ~300 KB of internal heap). It refuses to flash
// while the balancer reports ARMED.

#include "bala2_ota.h"

#include <string.h>
#include <inttypes.h>

#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "balancer.h"

static const char *TAG = "bala2_ota";

static httpd_handle_t s_server = NULL;

#define OTA_BUF_SIZE 4096

static esp_err_t send_simple(httpd_req_t *req, int status_code,
                             const char *content_type, const char *body)
{
    char status[32];
    snprintf(status, sizeof(status), "%d %s", status_code,
             status_code == 200 ? "OK" :
             status_code == 400 ? "Bad Request" :
             status_code == 409 ? "Conflict" :
             status_code == 500 ? "Internal Server Error" : "Error");
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, content_type);
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    const esp_app_desc_t *desc = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    char body[512];
    int n = snprintf(body, sizeof(body),
        "<!doctype html><html><body style='font-family:sans-serif'>"
        "<h2>Bala2 Fire</h2>"
        "<p>fw version: <b>%s</b></p>"
        "<p>idf version: %s</p>"
        "<p>compile: %s %s</p>"
        "<p>running slot: <b>%s</b> (offset 0x%" PRIx32 ", size 0x%" PRIx32 ")</p>"
        "<p>uptime: %lld ms</p>"
        "<hr><p>POST a built bala2.bin to <code>/update</code> to OTA.</p>"
        "</body></html>",
        desc->version, desc->idf_ver, desc->date, desc->time,
        running ? running->label : "?",
        running ? running->address : 0,
        running ? running->size : 0,
        esp_timer_get_time() / 1000);
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, body, n);
}

static esp_err_t info_get_handler(httpd_req_t *req)
{
    const esp_app_desc_t *desc = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    char body[256];
    int n = snprintf(body, sizeof(body),
        "{\"version\":\"%s\",\"idf\":\"%s\",\"slot\":\"%s\","
        "\"uptime_ms\":%lld,\"armed\":%s}",
        desc->version, desc->idf_ver,
        running ? running->label : "?",
        esp_timer_get_time() / 1000,
        balancer_is_armed() ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, n);
}

static esp_err_t update_post_handler(httpd_req_t *req)
{
    if (balancer_is_armed()) {
        ESP_LOGW(TAG, "OTA refused: balancer is ARMED");
        return send_simple(req, 409, "text/plain",
            "balancer is ARMED; disarm before OTA\n");
    }

    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update) {
        return send_simple(req, 500, "text/plain",
            "no OTA partition available\n");
    }
    ESP_LOGI(TAG, "OTA -> partition '%s' (offset 0x%" PRIx32 ", size 0x%" PRIx32 "), "
                  "content-length=%d",
             update->label, update->address, update->size, req->content_len);

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(update, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        return send_simple(req, 500, "text/plain", "esp_ota_begin failed\n");
    }

    char *buf = malloc(OTA_BUF_SIZE);
    if (!buf) {
        esp_ota_abort(handle);
        return send_simple(req, 500, "text/plain", "out of memory\n");
    }

    int total = 0;
    while (true) {
        int r = httpd_req_recv(req, buf, OTA_BUF_SIZE);
        if (r < 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
            ESP_LOGE(TAG, "recv error: %d", r);
            free(buf);
            esp_ota_abort(handle);
            return ESP_FAIL;
        }
        if (r == 0) break;  // EOF

        err = esp_ota_write(handle, buf, r);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
            free(buf);
            esp_ota_abort(handle);
            return send_simple(req, 500, "text/plain",
                "esp_ota_write failed\n");
        }
        total += r;
        if ((total & 0x1ffff) == 0) {  // log roughly every 128 KB
            ESP_LOGI(TAG, "OTA: %d bytes received", total);
        }
    }
    free(buf);

    err = esp_ota_end(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
        return send_simple(req, 500, "text/plain",
            err == ESP_ERR_OTA_VALIDATE_FAILED
              ? "image validation failed (corrupt or wrong target)\n"
              : "esp_ota_end failed\n");
    }

    err = esp_ota_set_boot_partition(update);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(err));
        return send_simple(req, 500, "text/plain",
            "esp_ota_set_boot_partition failed\n");
    }

    char ok[128];
    int n = snprintf(ok, sizeof(ok),
        "OTA OK: %d bytes -> %s. Rebooting...\n",
        total, update->label);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, ok, n);

    ESP_LOGI(TAG, "OTA complete (%d bytes), rebooting in 500 ms...", total);
    // Give the response time to drain over TCP before we yank the rug.
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;  // not reached
}

static const httpd_uri_t uri_root = {
    .uri = "/", .method = HTTP_GET, .handler = root_get_handler,
};
static const httpd_uri_t uri_info = {
    .uri = "/info", .method = HTTP_GET, .handler = info_get_handler,
};
static const httpd_uri_t uri_update = {
    .uri = "/update", .method = HTTP_POST, .handler = update_post_handler,
};

// ----- Control surface (used by the laptop-side MCP server) -----

static esp_err_t telemetry_get_handler(httpd_req_t *req)
{
    char body[512];
    int n = balancer_format_telemetry_json(body, sizeof(body));
    if (n < 0) {
        return send_simple(req, 500, "text/plain",
            "telemetry buffer overflow\n");
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, n);
}

static esp_err_t arm_post_handler(httpd_req_t *req)
{
    balancer_arm();
    // The balancer may stay in ARMING for a moment; report current state.
    char body[256];
    int n = balancer_format_telemetry_json(body, sizeof(body));
    if (n < 0) return send_simple(req, 500, "text/plain", "telemetry overflow\n");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, n);
}

static esp_err_t disarm_post_handler(httpd_req_t *req)
{
    balancer_disarm();
    char body[256];
    int n = balancer_format_telemetry_json(body, sizeof(body));
    if (n < 0) return send_simple(req, 500, "text/plain", "telemetry overflow\n");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, n);
}

// Read up to `cap-1` bytes of request body into `buf`, NUL-terminate.
// Returns bytes read (>=0), or negative on error / truncation.
static int read_body(httpd_req_t *req, char *buf, int cap)
{
    if (req->content_len <= 0) return 0;
    if (req->content_len >= cap) return -1;  // refuse oversize bodies
    int total = 0;
    while (total < req->content_len) {
        int r = httpd_req_recv(req, buf + total, req->content_len - total);
        if (r <= 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
            return -2;
        }
        total += r;
    }
    buf[total] = '\0';
    return total;
}

static esp_err_t setpoint_post_handler(httpd_req_t *req)
{
    char body[128];
    int n = read_body(req, body, sizeof(body));
    if (n < 0) {
        return send_simple(req, 400, "text/plain",
            n == -1 ? "body too large\n" : "read error\n");
    }
    cJSON *root = cJSON_Parse(body);
    if (!root) {
        return send_simple(req, 400, "text/plain",
            "invalid JSON; expect {\"deg\": <number>}\n");
    }
    cJSON *deg = cJSON_GetObjectItemCaseSensitive(root, "deg");
    if (!cJSON_IsNumber(deg)) {
        cJSON_Delete(root);
        return send_simple(req, 400, "text/plain",
            "missing or non-numeric 'deg'\n");
    }
    float v = (float)deg->valuedouble;
    cJSON_Delete(root);

    balancer_set_setpoint(v);
    float applied = balancer_get_setpoint();

    char out[96];
    int m = snprintf(out, sizeof(out),
        "{\"setpoint_deg\":%.3f,\"requested\":%.3f}", applied, v);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, m);
}

static const httpd_uri_t uri_telemetry = {
    .uri = "/telemetry", .method = HTTP_GET, .handler = telemetry_get_handler,
};
static const httpd_uri_t uri_arm = {
    .uri = "/arm", .method = HTTP_POST, .handler = arm_post_handler,
};
static const httpd_uri_t uri_disarm = {
    .uri = "/disarm", .method = HTTP_POST, .handler = disarm_post_handler,
};
static const httpd_uri_t uri_setpoint = {
    .uri = "/setpoint", .method = HTTP_POST, .handler = setpoint_post_handler,
};

esp_err_t bala2_ota_start(void)
{
    if (s_server) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.lru_purge_enable = true;
    // OTA streams ~1 MB of POST body; bump the recv timeout generously.
    cfg.recv_wait_timeout = 30;
    cfg.send_wait_timeout = 30;
    cfg.max_uri_handlers = 8;
    cfg.stack_size = 8192;

    esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start: %s", esp_err_to_name(err));
        return err;
    }

    httpd_register_uri_handler(s_server, &uri_root);
    httpd_register_uri_handler(s_server, &uri_info);
    httpd_register_uri_handler(s_server, &uri_update);
    httpd_register_uri_handler(s_server, &uri_telemetry);
    httpd_register_uri_handler(s_server, &uri_arm);
    httpd_register_uri_handler(s_server, &uri_disarm);
    httpd_register_uri_handler(s_server, &uri_setpoint);

    ESP_LOGI(TAG, "HTTP server listening on :80");
    return ESP_OK;
}

void bala2_ota_mark_valid(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK) {
        if (state == ESP_OTA_IMG_PENDING_VERIFY) {
            esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOGI(TAG, "marked image valid: %s", esp_err_to_name(err));
        }
    }
}
