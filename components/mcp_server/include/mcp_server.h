// On-device MCP (Model Context Protocol) server.
//
// Speaks JSON-RPC 2.0 over a single HTTP POST endpoint at /mcp.
// This is a minimal subset of the MCP "Streamable HTTP" transport:
// no SSE, no resources, no prompts, no notifications back to client.
// Just enough for an MCP client (Claude Desktop, MCP Inspector,
// FastMCP-as-client, etc.) to discover and call our tools.
//
// Exposed tools:
//   arm           - request balancer ARM
//   disarm        - force DISARMED
//   is_armed      - returns {"armed": true/false}
//   get_status    - returns full balancer telemetry + battery
//   drive         - {linear: -1..1, angular: -1..1}
//
// Endpoint:  POST http://<bot-ip>/mcp   Content-Type: application/json
// Health:    GET  http://<bot-ip>/      -> "Bala2 MCP server"

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Start the HTTP server on port 80. Idempotent.
esp_err_t mcp_server_start(void);

#ifdef __cplusplus
}
#endif
