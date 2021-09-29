F:
cd F:\VSN_workspace\dual_mode_mouse\nRF5_SDK_12.3.0_d7731ad\nRF5_SDK_12.3.0_d7731ad\examples\ble_peripheral\ble_app_hids_mouse\pca10028\s130\arm5_no_packs\_build\ 
:start
nrfutil pkg generate --hw-version 51 --application-version 2 --application nrf51822_xxac.hex --sd-req 0x87 --key-file private.key app_dfu_packet.zip
pause
goto start
exit
