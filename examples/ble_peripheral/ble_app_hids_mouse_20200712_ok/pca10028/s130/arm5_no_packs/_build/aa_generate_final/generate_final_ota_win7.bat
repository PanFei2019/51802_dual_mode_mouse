Z:
cd Z:\VSN_workspace\dual_mode_mouse\nRF5_SDK_12.3.0_d7731ad\nRF5_SDK_12.3.0_d7731ad\examples\ble_peripheral\ble_app_hids_mouse\pca10028\s130\arm5_no_packs\_build\ 
:start
nrfutil settings generate --family NRF51 --application nrf51822_xxac.hex --application-version 0 --bootloader-version 0 --bl-settings-version 1 --no-backup bootloader_setting.hex
mergehex -m s130_nrf51_2.0.1_softdevice.hex bootloader.hex -o softdevice_bootloader.hex
mergehex -m softdevice_bootloader.hex nrf51822_xxac.hex -o softdevice_bootloader_app.hex
mergehex -m softdevice_bootloader_app.hex bootloader_setting.hex -o aa_final_ota.hex
pause
goto start
exit