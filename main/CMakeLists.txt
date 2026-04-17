idf_component_register(SRCS "hello_world_main.c"
                             "User/wifi_connect.c"
                             "User/camera.c"
                             "User/rtsp_server.c"
                             "User/tcp_uart_server.c"
                             "User/eth_connect.c"
                             "User/tf_card.c"
                       PRIV_REQUIRES
                           spi_flash
                           esp_wifi
                           esp_netif
                           esp_event
                           nvs_flash
                           esp_wifi_remote
                           esp_hosted
                           esp_video
                           esp_driver_jpeg
                           esp_driver_uart
                           esp_driver_sdmmc
                           esp_h264
                           esp_eth
                           esp_timer
                           fatfs
                           sdmmc
                       INCLUDE_DIRS "User"
)
