idf_component_register(SRCS "acq_dma.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_adc esp_timer Acq_Common)
