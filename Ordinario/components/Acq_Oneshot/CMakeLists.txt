idf_component_register(SRCS "acq_oneshot.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_adc esp_driver_gptimer esp_timer Acq_Common)
