idf_component_register(SRCS "bala2_ota.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_http_server app_update esp_app_format
                                esp_partition esp_event nvs_flash log
                                esp_timer
                       PRIV_REQUIRES balancer)
