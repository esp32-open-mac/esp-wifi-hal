# ESP32-S3 reviewed MAC reference

`src/hal_mac.c` replaces the 57-function `hal_mac.o` member from ESP-IDF v5.4's
ESP32-S3 `libpp.a`. See [the review](REVIEW.md) for exact input hashes, corrected
ABI/dataflow errors, register observations, hardware results and remaining work.
The build helper rejects a different `libpp.a` and preserves all other members.

## Host regression checks

On Linux x86-64:

```sh
cc -std=c11 -O2 -Wall -Wextra -Werror -Itests tests/test_hal.c -o /tmp/s3-hal-test
/tmp/s3-hal-test
```

The test allocates low-address memory to keep the target's 32-bit pointer fields
without changing their offsets for the host. MMIO is captured in an ordered
trace. It does not access hardware.

## Signed hardware comparison

These scripts use an existing S3 project's signing key, console settings and
partition layout. The tested factory slot begins at `0x20000` and is 1 MiB;
the partition table is at `0x10000`. A stock test must be built first so the
replacement is compiled with that project's actual SDK compilation command.
The scripts are standalone test/build helpers, not reconstruction tools.

```sh
source /path/to/esp-idf/export.sh
S3_WORK=/absolute/path/to/test-output
mkdir -p "$S3_WORK/private"
chmod 700 "$S3_WORK/private"
# Create private/network.json with {"ssid":"...","password":"..."}; mode 600.

python build.py --variant stock --workspace "$S3_WORK" \
  --previous-project /path/to/existing-secured-s3-project
python flash_test.py --variant stock --workspace "$S3_WORK" --port /dev/ttyACM0

python build.py --variant reviewed --workspace "$S3_WORK" \
  --previous-project /path/to/existing-secured-s3-project
python validate.py --workspace "$S3_WORK"
python flash_test.py --variant reviewed --workspace "$S3_WORK" --port /dev/ttyACM0
```

`build.py` does not flash. It references the signing key in place, builds only
the application, verifies its signature and partition fit, and produces build
and archive-integrity JSON. A local component overlay changes the imported
`pp` archive; it does not modify the SDK. Credentials appear only in a private
header and the local application binary. Do not publish build directories.

`flash_test.py` verifies the image against its build report and signing key,
confirms S3 secure boot with flash encryption disabled, then writes only the
application at `0x20000`. It records the ROM security summary, flash log, serial
log and parsed pass/fail result. The supplied application uses RAM Wi-Fi
configuration and runs three scan/connect/DHCP/ping/TSF/deinit cycles. The PHY
may update its calibration data in NVS.

Recorded artifacts include `compile-command.json`, `archive-integrity.json`,
`stock/build-report.json`, `reviewed/build-report.json`, and timestamped device
logs. A passing comparison requires the map to show the replacement archive's
`hal_mac.o`; a firmware label alone does not establish that it was linked.
