; generated by Component: ARM Compiler 5.05 (build 41) Tool: ArmCC [4d0eb9]
; commandline ArmCC [--c99 --split_sections --debug -c -S -o.\_build\nrf_drv_qdec.s --depend=.\_build\nrf_drv_qdec.d --cpu=Cortex-M0 --apcs=interwork -O0 -I..\..\..\config\ble_app_hids_mouse_pca10028_s130 -I..\..\..\config -I..\..\..\..\..\..\components -I..\..\..\..\..\..\components\ble\ble_advertising -I..\..\..\..\..\..\components\ble\ble_dtm -I..\..\..\..\..\..\components\ble\ble_racp -I..\..\..\..\..\..\components\ble\ble_services\ble_ancs_c -I..\..\..\..\..\..\components\ble\ble_services\ble_ans_c -I..\..\..\..\..\..\components\ble\ble_services\ble_bas -I..\..\..\..\..\..\components\ble\ble_services\ble_bas_c -I..\..\..\..\..\..\components\ble\ble_services\ble_cscs -I..\..\..\..\..\..\components\ble\ble_services\ble_cts_c -I..\..\..\..\..\..\components\ble\ble_services\ble_dfu -I..\..\..\..\..\..\components\ble\ble_services\ble_dis -I..\..\..\..\..\..\components\ble\ble_services\ble_gls -I..\..\..\..\..\..\components\ble\ble_services\ble_hids -I..\..\..\..\..\..\components\ble\ble_services\ble_hrs -I..\..\..\..\..\..\components\ble\ble_services\ble_hrs_c -I..\..\..\..\..\..\components\ble\ble_services\ble_hts -I..\..\..\..\..\..\components\ble\ble_services\ble_ias -I..\..\..\..\..\..\components\ble\ble_services\ble_ias_c -I..\..\..\..\..\..\components\ble\ble_services\ble_lbs -I..\..\..\..\..\..\components\ble\ble_services\ble_lbs_c -I..\..\..\..\..\..\components\ble\ble_services\ble_lls -I..\..\..\..\..\..\components\ble\ble_services\ble_nus -I..\..\..\..\..\..\components\ble\ble_services\ble_nus_c -I..\..\..\..\..\..\components\ble\ble_services\ble_rscs -I..\..\..\..\..\..\components\ble\ble_services\ble_rscs_c -I..\..\..\..\..\..\components\ble\ble_services\ble_tps -I..\..\..\..\..\..\components\ble\common -I..\..\..\..\..\..\components\ble\nrf_ble_qwr -I..\..\..\..\..\..\components\ble\peer_manager -I..\..\..\..\..\..\components\boards -I..\..\..\..\..\..\components\drivers_nrf\adc -I..\..\..\..\..\..\components\drivers_nrf\clock -I..\..\..\..\..\..\components\drivers_nrf\common -I..\..\..\..\..\..\components\drivers_nrf\comp -I..\..\..\..\..\..\components\drivers_nrf\delay -I..\..\..\..\..\..\components\drivers_nrf\gpiote -I..\..\..\..\..\..\components\drivers_nrf\hal -I..\..\..\..\..\..\components\drivers_nrf\i2s -I..\..\..\..\..\..\components\drivers_nrf\lpcomp -I..\..\..\..\..\..\components\drivers_nrf\pdm -I..\..\..\..\..\..\components\drivers_nrf\power -I..\..\..\..\..\..\components\drivers_nrf\ppi -I..\..\..\..\..\..\components\drivers_nrf\pwm -I..\..\..\..\..\..\components\drivers_nrf\qdec -I..\..\..\..\..\..\components\drivers_nrf\rng -I..\..\..\..\..\..\components\drivers_nrf\rtc -I..\..\..\..\..\..\components\drivers_nrf\saadc -I..\..\..\..\..\..\components\drivers_nrf\spi_master -I..\..\..\..\..\..\components\drivers_nrf\spi_slave -I..\..\..\..\..\..\components\drivers_nrf\swi -I..\..\..\..\..\..\components\drivers_nrf\timer -I..\..\..\..\..\..\components\drivers_nrf\twi_master -I..\..\..\..\..\..\components\drivers_nrf\twis_slave -I..\..\..\..\..\..\components\drivers_nrf\uart -I..\..\..\..\..\..\components\drivers_nrf\usbd -I..\..\..\..\..\..\components\drivers_nrf\wdt -I..\..\..\..\..\..\components\libraries\bsp -I..\..\..\..\..\..\components\libraries\button -I..\..\..\..\..\..\components\libraries\crc16 -I..\..\..\..\..\..\components\libraries\crc32 -I..\..\..\..\..\..\components\libraries\csense -I..\..\..\..\..\..\components\libraries\csense_drv -I..\..\..\..\..\..\components\libraries\experimental_section_vars -I..\..\..\..\..\..\components\libraries\fds -I..\..\..\..\..\..\components\libraries\fstorage -I..\..\..\..\..\..\components\libraries\gpiote -I..\..\..\..\..\..\components\libraries\hardfault -I..\..\..\..\..\..\components\libraries\hci -I..\..\..\..\..\..\components\libraries\led_softblink -I..\..\..\..\..\..\components\libraries\log -I..\..\..\..\..\..\components\libraries\log\src -I..\..\..\..\..\..\components\libraries\low_power_pwm -I..\..\..\..\..\..\components\libraries\mem_manager -I..\..\..\..\..\..\components\libraries\pwm -I..\..\..\..\..\..\components\libraries\queue -I..\..\..\..\..\..\components\libraries\scheduler -I..\..\..\..\..\..\components\libraries\sensorsim -I..\..\..\..\..\..\components\libraries\slip -I..\..\..\..\..\..\components\libraries\timer -I..\..\..\..\..\..\components\libraries\twi -I..\..\..\..\..\..\components\libraries\uart -I..\..\..\..\..\..\components\libraries\usbd -I..\..\..\..\..\..\components\libraries\usbd\class\audio -I..\..\..\..\..\..\components\libraries\usbd\class\cdc -I..\..\..\..\..\..\components\libraries\usbd\class\cdc\acm -I..\..\..\..\..\..\components\libraries\usbd\class\hid -I..\..\..\..\..\..\components\libraries\usbd\class\hid\generic -I..\..\..\..\..\..\components\libraries\usbd\class\hid\kbd -I..\..\..\..\..\..\components\libraries\usbd\class\hid\mouse -I..\..\..\..\..\..\components\libraries\usbd\class\msc -I..\..\..\..\..\..\components\libraries\usbd\config -I..\..\..\..\..\..\components\libraries\util -I..\..\..\..\..\..\components\softdevice\common\softdevice_handler -I..\..\..\..\..\..\components\softdevice\s130\headers -I..\..\..\..\..\..\components\softdevice\s130\headers\nrf51 -I..\..\..\..\..\..\components\toolchain -I..\..\..\..\..\..\external\segger_rtt -I..\config -I..\..\..\..\..\..\components\drivers_nrf\qdec -IE:\other\Nordic\51822\nRF5_SDK_12.3.0_d7731ad\nRF5_SDK_12.3.0_d7731ad\examples\ble_peripheral\ble_app_hids_mouse\pca10028\s130\arm5_no_packs\RTE -ID:\Keil_v5\ARM\PACK\ARM\CMSIS\4.5.0\CMSIS\Include -ID:\Keil_v5\ARM\PACK\NordicSemiconductor\nRF_DeviceFamilyPack\8.11.1\Device\Include -D__MICROLIB -D_RTE_ -DNRF51 -DBLE_STACK_SUPPORT_REQD -DNRF51822 -DBOARD_PCA10028 -DNRF_SD_BLE_API_VERSION=2 -DS130 -DNRF51 -DSOFTDEVICE_PRESENT -DSWI_DISABLE0 --omf_browse=.\_build\nrf_drv_qdec.crf ..\..\..\..\..\..\components\drivers_nrf\qdec\nrf_drv_qdec.c]
        THUMB
        PRESERVE8

        AREA ||.arm_vfe_header||, DATA, READONLY, NOALLOC, ALIGN=2

        DCD      0x00000000

;*** Start embedded assembler ***

#line 1 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\qdec\\nrf_drv_qdec.c"
	AREA ||.rev16_text||, CODE
	THUMB
	EXPORT |__asm___14_nrf_drv_qdec_c____REV16|
#line 388 "D:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.5.0\\CMSIS\\Include\\cmsis_armcc.h"
|__asm___14_nrf_drv_qdec_c____REV16| PROC
#line 389

 rev16 r0, r0
 bx lr
	ENDP
	AREA ||.revsh_text||, CODE
	THUMB
	EXPORT |__asm___14_nrf_drv_qdec_c____REVSH|
#line 402
|__asm___14_nrf_drv_qdec_c____REVSH| PROC
#line 403

 revsh r0, r0
 bx lr
	ENDP

;*** End   embedded assembler ***

        IMPORT ||Lib$$Request$$armlib|| [CODE,WEAK]

        ATTR FILESCOPE
        ATTR SETVALUE Tag_ABI_PCS_wchar_t,2
        ATTR SETVALUE Tag_ABI_enum_size,1
        ATTR SETVALUE Tag_ABI_optimization_goals,6
        ATTR SETSTRING Tag_conformance,"2.06"
        ATTR SETVALUE AV,18,1

        ASSERT {ENDIAN} = "little"
        ASSERT {INTER} = {TRUE}
        ASSERT {ROPI} = {FALSE}
        ASSERT {RWPI} = {FALSE}
        ASSERT {IEEE_FULL} = {FALSE}
        ASSERT {IEEE_PART} = {FALSE}
        ASSERT {IEEE_JAVA} = {FALSE}
        END
