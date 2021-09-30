Z:
cd Z:\VSN_workspace\dual_mode_mouse\nRF5_SDK_12.3.0_d7731ad\nRF5_SDK_12.3.0_d7731ad\examples\ble_peripheral\ble_app_hids_mouse\pca10028\s130\arm5_no_packs\_build\ 
:start
mergehex -m s130_nrf51_2.0.1_softdevice.hex bootloader.hex -o softdevice_bootloader.hex
nrfjprog --program "softdevice_bootloader.hex" --chiperase --family NRF51
nrfjprog --verify "softdevice_bootloader.hex" --family NRF51
nrfjprog --reset --family NRF51
pause
goto start
exit