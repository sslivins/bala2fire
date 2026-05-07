// Minimal MCP server over HTTP/JSON-RPC 2.0. See header for protocol notes.

#include "mcp_server.h"

#include <string.h>
#include <stdlib.h>

#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "balancer.h"

static const char *TAG = "mcp";

static httpd_handle_t s_httpd = NULL;

// MCP protocol version we advertise. The 2025-03-26 revision introduced
// Streamable HTTP and is widely supported by current clients.
static const char *MCP_PROTOCOL_VERSION = "2025-03-26";
static const char *SERVER_NAME = "bala2-fire";
static const char *SERVER_VERSION = "0.1.0";

// ---------- Tool implementations ----------
// Each returns a cJSON object that becomes the "result" of tools/call,
// or NULL on error (caller fills in a JSON-RPC error).

static cJSON *make_text_content(const char *text)
{
    cJSON *content = cJSON_CreateArray();
    cJSON *item = cJSON_CreateObject();
    cJSON_AddStringToObject(item, "type", "text");
    cJSON_AddStringToObject(item, "text", text ? text : "");
    cJSON_AddItemToArray(content, item);
    cJSON *result = cJSON_CreateObject();
    cJSON_AddItemToObject(result, "content", content);
    cJSON_AddBoolToObject(result, "isError", false);
    return result;
}

static cJSON *tool_arm(const cJSON *args)
{
    (void)args;
    balancer_arm();
    return make_text_content("arm requested");
}

static cJSON *tool_disarm(const cJSON *args)
{
    (void)args;
    balancer_disarm();
    return make_text_content("disarmed");
}

static cJSON *tool_is_armed(const cJSON *args)
{
    (void)args;
    bool armed = balancer_is_armed();
    return make_text_content(armed ? "{\"armed\": true}" : "{\"armed\": false}");
}

static const char *state_str(balancer_state_t s)
{
    switch (s) {
        case BALANCER_BOOT:     return "boot";
        case BALANCER_DISARMED: return "disarmed";
        case BALANCER_ARMING:   return "arming";
        case BALANCER_ARMED:    return "armed";
        case BALANCER_CRASHED:  return "crashed";
        case BALANCER_FAULT:    return "fault";
    }
    return "?";
}

static cJSON *tool_get_status(const cJSON *args)
{
    (void)args;
    balancer_status_t s;
    balancer_get_status(&s);

    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "state", state_str(s.state));
    cJSON_AddBoolToObject  (o, "armed", s.state == BALANCER_ARMED);
    cJSON_AddNumberToObject(o, "angle_deg", s.angle_deg);
    cJSON_AddNumberToObject(o, "gyro_dps", s.gyro_dps);
    cJSON_AddNumberToObject(o, "accel_mag_g", s.accel_mag_g);
    cJSON_AddNumberToObject(o, "pwm_left", s.pwm_left);
    cJSON_AddNumberToObject(o, "pwm_right", s.pwm_right);
    cJSON_AddNumberToObject(o, "enc_left", s.enc_left);
    cJSON_AddNumberToObject(o, "enc_right", s.enc_right);
    cJSON_AddNumberToObject(o, "loop_dt_ms", s.loop_dt_ms);
    cJSON_AddNumberToObject(o, "loops", s.loops);
    cJSON_AddNumberToObject(o, "overrun_count", s.overrun_count);
    cJSON_AddNumberToObject(o, "i2c_err_count", s.i2c_err_count);

    char *txt = cJSON_PrintUnformatted(o);
    cJSON_Delete(o);
    cJSON *res = make_text_content(txt ? txt : "{}");
    if (txt) cJSON_free(txt);
    return res;
}

static cJSON *tool_drive(const cJSON *args)
{
    double linear = 0.0, angular = 0.0;
    cJSON *jl = cJSON_GetObjectItemCaseSensitive(args, "linear");
    cJSON *ja = cJSON_GetObjectItemCaseSensitive(args, "angular");
    if (cJSON_IsNumber(jl)) linear  = jl->valuedouble;
    if (cJSON_IsNumber(ja)) angular = ja->valuedouble;
    balancer_set_drive((float)linear, (float)angular);
    char buf[80];
    snprintf(buf, sizeof(buf), "drive linear=%.2f angular=%.2f (valid 500ms)",
             linear, angular);
    return make_text_content(buf);
}

// ---------- Tool dispatch table ----------

typedef struct {
    const char *name;
    const char *description;
    const char *input_schema_json;   // raw JSON for inputSchema
    cJSON *(*handler)(const cJSON *args);
} mcp_tool_t;

static const mcp_tool_t TOOLS[] = {
    {
        "arm",
        "Request the balancing controller to arm. Only succeeds when the robot is upright and still.",
        "{\"type\":\"object\",\"properties\":{},\"additionalProperties\":false}",
        tool_arm,
    },
    {
        "disarm",
        "Immediately disarm the balancing controller and stop the motors.",
        "{\"type\":\"object\",\"properties\":{},\"additionalProperties\":false}",
        tool_disarm,
    },
    {
        "is_armed",
        "Returns whether the balancing controller is currently armed.",
        "{\"type\":\"object\",\"properties\":{},\"additionalProperties\":false}",
        tool_is_armed,
    },
    {
        "get_status",
        "Snapshot of balancer telemetry: state, tilt angle, gyro rate, motor PWM, encoders, loop dt, error counts.",
        "{\"type\":\"object\",\"properties\":{},\"additionalProperties\":false}",
        tool_get_status,
    },
    {
        "drive",
        "Drive the robot. linear (-1..1): forward(+)/back(-). angular (-1..1): right(+)/left(-) yaw. Command expires after 500 ms; resend at >2 Hz.",
        "{\"type\":\"object\",\"properties\":{"
        "\"linear\":{\"type\":\"number\",\"minimum\":-1,\"maximum\":1,\"description\":\"Forward velocity bias\"},"
        "\"angular\":{\"type\":\"number\",\"minimum\":-1,\"maximum\":1,\"description\":\"Yaw rate bias\"}"
        "},\"required\":[\"linear\",\"angular\"],\"additionalProperties\":false}",
        tool_drive,
    },
};
static const size_t N_TOOLS = sizeof(TOOLS) / sizeof(TOOLS[0]);

// ---------- JSON-RPC plumbing ----------

static cJSON *make_error(int code, const char *message)
{
    cJSON *err = cJSON_CreateObject();
    cJSON_AddNumberToObject(err, "code", code);
    cJSON_AddStringToObject(err, "message", message);
    return err;
}

// Build an "initialize" result.
static cJSON *handle_initialize(const cJSON *params)
{
    (void)params;
    cJSON *res = cJSON_CreateObject();
    cJSON_AddStringToObject(res, "protocolVersion", MCP_PROTOCOL_VERSION);

    cJSON *caps = cJSON_CreateObject();
    cJSON *tools_cap = cJSON_CreateObject();
    cJSON_AddBoolToObject(tools_cap, "listChanged", false);
    cJSON_AddItemToObject(caps, "tools", tools_cap);
    cJSON_AddItemToObject(res, "capabilities", caps);

    cJSON *info = cJSON_CreateObject();
    cJSON_AddStringToObject(info, "name", SERVER_NAME);
    cJSON_AddStringToObject(info, "version", SERVER_VERSION);
    cJSON_AddItemToObject(res, "serverInfo", info);
    return res;
}

// Build a "tools/list" result.
static cJSON *handle_tools_list(void)
{
    cJSON *res = cJSON_CreateObject();
    cJSON *arr = cJSON_CreateArray();
    for (size_t i = 0; i < N_TOOLS; ++i) {
        cJSON *t = cJSON_CreateObject();
        cJSON_AddStringToObject(t, "name", TOOLS[i].name);
        cJSON_AddStringToObject(t, "description", TOOLS[i].description);
        cJSON *schema = cJSON_Parse(TOOLS[i].input_schema_json);
        if (!schema) schema = cJSON_CreateObject();
        cJSON_AddItemToObject(t, "inputSchema", schema);
        cJSON_AddItemToArray(arr, t);
    }
    cJSON_AddItemToObject(res, "tools", arr);
    return res;
}

// Build a "tools/call" result. *out_err is set on protocol-level error.
static cJSON *handle_tools_call(const cJSON *params, cJSON **out_err)
{
    *out_err = NULL;
    cJSON *jname = cJSON_GetObjectItemCaseSensitive(params, "name");
    if (!cJSON_IsString(jname)) {
        *out_err = make_error(-32602, "missing 'name'");
        return NULL;
    }
    cJSON *jargs = cJSON_GetObjectItemCaseSensitive(params, "arguments");
    if (jargs && !cJSON_IsObject(jargs)) {
        *out_err = make_error(-32602, "'arguments' must be object");
        return NULL;
    }
    for (size_t i = 0; i < N_TOOLS; ++i) {
        if (strcmp(TOOLS[i].name, jname->valuestring) == 0) {
            cJSON *r = TOOLS[i].handler(jargs);
            if (!r) {
                *out_err = make_error(-32603, "tool failed");
            }
            return r;
        }
    }
    *out_err = make_error(-32601, "unknown tool");
    return NULL;
}

// Dispatch a single JSON-RPC request object. Returns a fresh response
// object (caller frees) or NULL for notifications (no response).
static cJSON *dispatch_request(cJSON *req)
{
    cJSON *jid     = cJSON_GetObjectItemCaseSensitive(req, "id");
    cJSON *jmethod = cJSON_GetObjectItemCaseSensitive(req, "method");
    cJSON *jparams = cJSON_GetObjectItemCaseSensitive(req, "params");
    bool is_notification = (jid == NULL) || cJSON_IsNull(jid);

    if (!cJSON_IsString(jmethod)) {
        if (is_notification) return NULL;
        cJSON *resp = cJSON_CreateObject();
        cJSON_AddStringToObject(resp, "jsonrpc", "2.0");
        cJSON_AddItemToObject(resp, "id", jid ? cJSON_Duplicate(jid, true) : cJSON_CreateNull());
        cJSON_AddItemToObject(resp, "error", make_error(-32600, "invalid request"));
        return resp;
    }
    const char *method = jmethod->valuestring;

    cJSON *result = NULL;
    cJSON *error  = NULL;

    if (strcmp(method, "initialize") == 0) {
        result = handle_initialize(jparams);
    } else if (strcmp(method, "notifications/initialized") == 0 ||
               strcmp(method, "notifications/cancelled") == 0) {
        return NULL;        // notifications get no response
    } else if (strcmp(method, "ping") == 0) {
        result = cJSON_CreateObject();
    } else if (strcmp(method, "tools/list") == 0) {
        result = handle_tools_list();
    } else if (strcmp(method, "tools/call") == 0) {
        result = handle_tools_call(jparams, &error);
    } else if (strcmp(method, "resources/list") == 0 ||
               strcmp(method, "prompts/list") == 0) {
        // Some clients probe these; reply with an empty list rather than error.
        result = cJSON_CreateObject();
        cJSON_AddItemToObject(result,
            (strcmp(method, "resources/list") == 0) ? "resources" : "prompts",
            cJSON_CreateArray());
    } else {
        error = make_error(-32601, "method not found");
    }

    if (is_notification) {
        if (result) cJSON_Delete(result);
        if (error)  cJSON_Delete(error);
        return NULL;
    }

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddStringToObject(resp, "jsonrpc", "2.0");
    cJSON_AddItemToObject(resp, "id", cJSON_Duplicate(jid, true));
    if (error) {
        cJSON_AddItemToObject(resp, "error", error);
        if (result) cJSON_Delete(result);
    } else {
        cJSON_AddItemToObject(resp, "result", result ? result : cJSON_CreateObject());
    }
    return resp;
}

// ---------- HTTP handlers ----------

static esp_err_t mcp_post_handler(httpd_req_t *req)
{
    if (req->content_len == 0 || req->content_len > 8192) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body size");
        return ESP_OK;
    }
    char *body = (char *)malloc(req->content_len + 1);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_OK;
    }
    int total = 0;
    while (total < (int)req->content_len) {
        int r = httpd_req_recv(req, body + total, req->content_len - total);
        if (r <= 0) {
            free(body);
            return ESP_FAIL;
        }
        total += r;
    }
    body[total] = '\0';

    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req,
            "{\"jsonrpc\":\"2.0\",\"id\":null,\"error\":{\"code\":-32700,\"message\":\"parse error\"}}");
        return ESP_OK;
    }

    cJSON *response = NULL;
    if (cJSON_IsArray(root)) {
        // Batch request.
        response = cJSON_CreateArray();
        cJSON *item = NULL;
        cJSON_ArrayForEach(item, root) {
            cJSON *r = dispatch_request(item);
            if (r) cJSON_AddItemToArray(response, r);
        }
        if (cJSON_GetArraySize(response) == 0) {
            // All notifications -> 202 with empty body.
            cJSON_Delete(response);
            cJSON_Delete(root);
            httpd_resp_set_status(req, "202 Accepted");
            httpd_resp_send(req, NULL, 0);
            return ESP_OK;
        }
    } else if (cJSON_IsObject(root)) {
        response = dispatch_request(root);
        if (!response) {
            cJSON_Delete(root);
            httpd_resp_set_status(req, "202 Accepted");
            httpd_resp_send(req, NULL, 0);
            return ESP_OK;
        }
    } else {
        cJSON_Delete(root);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req,
            "{\"jsonrpc\":\"2.0\",\"id\":null,\"error\":{\"code\":-32600,\"message\":\"invalid request\"}}");
        return ESP_OK;
    }

    cJSON_Delete(root);
    char *out = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    httpd_resp_set_type(req, "application/json");
    if (out) {
        httpd_resp_sendstr(req, out);
        cJSON_free(out);
    } else {
        httpd_resp_sendstr(req, "{}");
    }
    return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req,
        "Bala2 Fire MCP server\n"
        "POST /mcp  Content-Type: application/json   (JSON-RPC 2.0)\n");
    return ESP_OK;
}

// MCP Streamable HTTP clients may probe with GET /mcp to open an SSE
// stream. We don't push notifications, so just refuse cleanly.
static esp_err_t mcp_get_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "405 Method Not Allowed");
    httpd_resp_set_hdr(req, "Allow", "POST");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

esp_err_t mcp_server_start(void)
{
    if (s_httpd) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 8080;
    cfg.ctrl_port = 32769;          // must differ from OTA server's ctrl_port (default 32768)
    cfg.max_uri_handlers = 6;
    cfg.lru_purge_enable = true;
    cfg.recv_wait_timeout = 10;
    cfg.send_wait_timeout = 10;

    esp_err_t err = httpd_start(&s_httpd, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        s_httpd = NULL;
        return err;
    }

    httpd_uri_t root_uri = {
        .uri = "/", .method = HTTP_GET, .handler = root_get_handler, .user_ctx = NULL,
    };
    httpd_uri_t mcp_post = {
        .uri = "/mcp", .method = HTTP_POST, .handler = mcp_post_handler, .user_ctx = NULL,
    };
    httpd_uri_t mcp_get = {
        .uri = "/mcp", .method = HTTP_GET, .handler = mcp_get_handler, .user_ctx = NULL,
    };
    httpd_register_uri_handler(s_httpd, &root_uri);
    httpd_register_uri_handler(s_httpd, &mcp_post);
    httpd_register_uri_handler(s_httpd, &mcp_get);

    ESP_LOGI(TAG, "MCP server listening on http://<bot-ip>:8080/mcp");
    return ESP_OK;
}
