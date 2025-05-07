set(SOURCES
    "I2Cdev.cpp"
)
if(NOT IDF_TARGET STREQUAL "esp32")
    list(APPEND SOURCES "soft_i2c_master.c")
endif()

idf_component_register(SRCS "${SOURCES}"
                       INCLUDE_DIRS "."
		       REQUIRES driver
)
