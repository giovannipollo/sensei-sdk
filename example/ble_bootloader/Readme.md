
## NRF

Make sure your `SENSEI_SDK_ROOT` environment variable is set to the path of the SENSEI SDK.

First build the bootloader itself and flash it to the device. This can be done by running the following commands:

```bash
$> west build -b nrf5340_senseiv1_cpuapp_bootloader --build-dir build_mcuboot $ZEPHYR_BASE/../bootloader/mcuboot/boot/zephyr -- -DEXTRA_ZEPHYR_MODULES=$SENSEI_SDK_ROOT/NRF
$> west flash -d build_mcuboot
```

Then build the application by running:
```bash
$> west build -b nrf5340_senseiv1_cpuapp
```

Next, you can flash the application via the serial recovery bootloader over USB by running:
```bash
$> mcumgr-client upload build/zephyr/app_update.bin
```

Finally, flash the application via the bootloader the device by running:
```bash
$> mcumgr --conntype ble --connstring 'peer_name=mcuboot' upload build/zephyr/app_update.bin
```

Alternatively you can use [mcumgr-web](https://github.com/boogie/mcumgr-web/tree/main) to upload, test and then confirm the update.

1. Open mcumgr-web in a browser (Online Version: https://boogie.github.io/mcumgr-web/)
2. Connect to the device
3. Upload the application binary
4. Enable test mode for the bi
5. Reboot the device
6. Confirm the update
7. Optionally erase the old application

I recommend using the  Chrome browser.

## GAP9