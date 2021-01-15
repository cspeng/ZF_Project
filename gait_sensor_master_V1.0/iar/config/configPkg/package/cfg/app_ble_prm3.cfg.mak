# invoke SourceDir generated makefile for app_ble.prm3
app_ble.prm3: .libraries,app_ble.prm3
.libraries,app_ble.prm3: package/cfg/app_ble_prm3.xdl
	$(MAKE) -f C:\ti\simplelink\ble_sdk_2_02_01_18\examples\cc2650iot\gait_sensor_master_V1.0\iar\config/src/makefile.libs

clean::
	$(MAKE) -f C:\ti\simplelink\ble_sdk_2_02_01_18\examples\cc2650iot\gait_sensor_master_V1.0\iar\config/src/makefile.libs clean

