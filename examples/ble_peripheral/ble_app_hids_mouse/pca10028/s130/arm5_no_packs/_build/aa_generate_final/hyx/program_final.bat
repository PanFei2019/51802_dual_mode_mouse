Z:
cd Z:\VSN_workspace\dual_mode_mouse\nRF5_SDK_12.3.0_d7731ad\nRF5_SDK_12.3.0_d7731ad\examples\ble_peripheral\ble_app_hids_mouse\pca10028\s130\arm5_no_packs\_build\ 
:start
nrfjprog --program "aa_final_ota.hex" --chiperase --family NRF51
nrfjprog --verify "aa_final_ota.hex" --family NRF51
nrfjprog --reset --family NRF51
pause
goto start
exit