idf_component_register(
    SRCS "GalvoSystem.cpp"
    INCLUDE_DIRS "include"
    REQUIRES mcp4725 
             i2c_bus 
             freertos
    PRIV_REQUIRES log
)
