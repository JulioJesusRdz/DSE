idf_component_register(SRCS "experiment.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES INA219 Acq_Common Acq_Oneshot Acq_Dma esp_timer)
