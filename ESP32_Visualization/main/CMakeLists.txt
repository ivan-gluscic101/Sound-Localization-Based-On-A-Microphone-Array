set(include_dirs
    "."
    "../components/Inc"
    "../../Sound Localization/Core/CustomDriverSet/IncDriver"
)

idf_component_register(SRCS "main.c"
                            "../components/Src/uart_driver.c"
                            "../components/Src/sound_loc_processor.c"
                            "../components/Src/wifi_manager.c"
                            "../components/Src/web_server.c"
                       INCLUDE_DIRS ${include_dirs}
)