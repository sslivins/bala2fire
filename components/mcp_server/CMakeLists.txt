idf_component_register(SRCS "mcp_server.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_http_server json esp_timer esp_wifi esp_netif
                       PRIV_REQUIRES balancer)
