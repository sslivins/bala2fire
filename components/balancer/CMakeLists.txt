idf_component_register(SRCS "balancer.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos esp_timer bala2_base
                       PRIV_REQUIRES m5stack__M5Unified)
