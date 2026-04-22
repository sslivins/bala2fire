idf_component_register(SRCS "bala2_main.cpp"
                            "bala2_wifi.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_timer
                                bala2_base balancer)
