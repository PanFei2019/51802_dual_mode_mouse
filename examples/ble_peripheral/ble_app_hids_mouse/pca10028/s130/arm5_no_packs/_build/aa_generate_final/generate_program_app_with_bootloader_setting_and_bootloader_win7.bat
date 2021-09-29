I:
cd I:\51802\dual_mode_mouse\nRF5_SDK_12.3.0_d7731ad\nRF5_SDK_12.3.0_d7731ad\examples\ble_peripheral\ble_app_hids_mouse\pca10028\s130\arm5_no_packs\_build\ 
:start
nrfutil settings generate --family NRF51 --application nrf51822_xxac.hex --application-version 0 --bootloader-version 0 --bl-settings-version 1 --no-backup bootloader_setting.hex
mergehex -m nrf51822_xxac.hex bootloader_setting.hex -o aa_nrf51822_xxac_with_bootloader_setting.hex
mergehex -m aa_nrf51822_xxac_with_bootloader_setting.hex bootloader.hex -o aa_nrf51822_xxac_with_bootloader_setting_and_bootloader.hex
nrfjprog --program "aa_nrf51822_xxac_with_bootloader_setting_and_bootloader.hex" --sectorerase --family NRF51
nrfjprog --verify "aa_nrf51822_xxac_with_bootloader_setting_and_bootloader.hex" --family NRF51
nrfjprog --reset --family NRF51
pause
goto start
exit