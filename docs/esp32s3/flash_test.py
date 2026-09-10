#!/usr/bin/env python3
"""Flash only a verified signed factory app and record its three-cycle Wi-Fi test."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import time
import serial

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--workspace",type=Path,default=Path.cwd()/"artifacts/esp32s3/manual-review")
    p.add_argument("--variant",choices=["stock","reviewed"],required=True)
    p.add_argument("--port",default="/dev/ttyACM0")
    a=p.parse_args(); work=a.workspace.resolve(); base=work/a.variant
    report=json.loads((base/"build-report.json").read_text()); app=Path(report["app"])
    assert hashlib.sha256(app.read_bytes()).hexdigest()==report["app_sha256"]
    assert report["offset"]=="0x20000" and app.stat().st_size<=0x100000
    config=(base/"project/sdkconfig").read_text()
    assert 'CONFIG_PARTITION_TABLE_OFFSET=0x10000\n' in config
    assert '# CONFIG_SECURE_FLASH_ENC_ENABLED is not set\n' in config
    key=json.loads(next(s.split('=',1)[1] for s in config.splitlines() if s.startswith('CONFIG_SECURE_BOOT_SIGNING_KEY=')))
    subprocess.run([sys.executable,"-m","espsecure","verify_signature","--version","2","--keyfile",key,str(app)],check=True)
    stamp=time.strftime('%Y%m%d-%H%M%S',time.gmtime()); out=work/"device";out.mkdir(exist_ok=True)
    prefix=out/f"{a.variant}-{stamp}"
    command=[sys.executable,"-m","esptool","--chip","esp32s3","--port",a.port,
             "--before","default_reset","--after","no_reset","--no-stub","get_security_info"]
    info=subprocess.run(command,capture_output=True,text=True,check=True).stdout
    prefix.with_suffix('.security.txt').write_text(info)
    assert "Chip is ESP32-S3" in info and "Secure Boot: Enabled" in info
    assert "Flash Encryption: Disabled" in info
    command=[sys.executable,"-m","esptool","--chip","esp32s3","--port",a.port,"--baud","460800",
             "--before","no_reset","--after","hard_reset","--no-stub","write_flash","0x20000",str(app)]
    with prefix.with_suffix('.flash.txt').open('w') as log:
        subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
    output=bytearray(); end=time.monotonic()+60
    # On this native USB interface, opening the console after ROM reset permits
    # the application to run. No second monitor may hold the device open.
    with serial.Serial(a.port,115200,timeout=.2) as port:
        port.dtr=False;port.rts=False
        while time.monotonic()<end:
            chunk=port.read(4096);output.extend(chunk)
            # Stream concise application milestones; never print credentials.
            if b"stage=complete" in output or b"stage=fail" in output or b"Guru Meditation" in output:break
    text=output.decode(errors='replace'); prefix.with_suffix('.serial.txt').write_text(text)
    milestones=[s for s in text.splitlines() if 's3_hal_e2e:' in s]
    print('\n'.join(milestones))
    replies=[int(s) for s in re.findall(r'stage=traffic replies=(\d+)',text)]
    delta=[int(s) for s in re.findall(r'stage=tsf value=\d+ delta=(\d+)',text)]
    heaps=[int(s) for s in re.findall(r'stage=cycle_complete round=\d+ free_heap=(\d+)',text)]
    passed=(f'stage=complete variant={a.variant}_hal cycles=3' in text and
            'stage=fail' not in text and 'Guru Meditation' not in text and
            len(replies)==3 and min(replies)>=18 and len(delta)==3 and min(delta)>0 and
            len(heaps)==3 and text.count('stage=associated')==3)
    result={"passed":passed,"variant":a.variant,"app_sha256":report["app_sha256"],
        "hal_source_sha256":report["hal_source_sha256"],"ping_replies":replies,
        "tsf_delta_us":delta,"heap_after_cycles":heaps,"serial_log":str(prefix.with_suffix('.serial.txt')),
        "application_only":True,"flash_offset":"0x20000","signing_key_accepted_by_device":
            'Signature verified successfully!' in text and 'stage=boot' in text}
    prefix.with_suffix('.result.json').write_text(json.dumps(result,indent=2))
    (work/f'{a.variant}-device-result.json').write_text(json.dumps(result,indent=2))
    if not passed:raise SystemExit('Device test failed; inspect the retained serial log.')

if __name__=='__main__':main()
