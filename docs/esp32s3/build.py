#!/usr/bin/env python3
"""Build signed stock/reviewed HAL test apps; no device access or SDK mutation."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

SOURCE = Path(__file__).resolve().parent
WORK = Path.cwd() / "artifacts/esp32s3/manual-review"
IDF = Path(os.environ.get("IDF_PATH", str(Path.home()/"esp/esp-idf")))
OLD = Path.home()/"esp/hello_world"

def run(args, **kw):
    subprocess.run([str(a) for a in args], check=True, **kw)

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def archive():
    database=WORK/"stock/build/compile_commands.json"
    entries=json.loads(database.read_text())
    entry=next(e for e in entries if Path(e["file"])==WORK/"stock/project/main/main.c")
    args=entry.get("arguments") or shlex.split(entry["command"])
    for flag, path in {"-c":SOURCE/"src/hal_mac.c", "-o":WORK/"hal_mac.o", "-MF":WORK/"hal_mac.d"}.items():
        if flag in args: args[args.index(flag)+1] = str(path)
    (WORK/"compile-command.json").write_text(json.dumps(args,indent=2))
    run(args, cwd=entry["directory"])
    original = IDF/"components/esp_wifi/lib/esp32s3/libpp.a"
    if sha(original)!="284e57fbc83ce1b57733733a536ff09bdae99a6ff464eaefb71119080bb1cb39":
        raise RuntimeError("This reconstruction requires the pinned IDF v5.4 libpp.a")
    destination=WORK/"libpp.a"
    shutil.copy2(original,destination)
    ar = str(Path(args[0]).with_name(Path(args[0]).name.removesuffix("gcc")+"ar"))
    run([ar,"r",destination,WORK/"hal_mac.o"])
    members=subprocess.check_output([ar,"t",original],text=True).splitlines()
    assert subprocess.check_output([ar,"t",destination],text=True).splitlines()==members
    assert members.count("hal_mac.o")==1
    untouched=[]
    for member in members:
        if member=="hal_mac.o": continue
        before=subprocess.check_output([ar,"p",original,member])
        after=subprocess.check_output([ar,"p",destination,member])
        assert before==after,member
        untouched.append(member)
    (WORK/"archive-integrity.json").write_text(json.dumps({"original_sha256":sha(original),
        "replacement_sha256":sha(destination),"source_sha256":sha(SOURCE/"src/hal_mac.c"),
        "replaced_member":"hal_mac.o","unchanged_members":untouched},indent=2))
    return destination

def main():
    global WORK, IDF, OLD
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant",choices=["stock","reviewed"],required=True)
    parser.add_argument("--workspace",type=Path,default=WORK)
    parser.add_argument("--idf",type=Path,default=IDF)
    parser.add_argument("--previous-project",type=Path,default=OLD)
    a=parser.parse_args()
    WORK=a.workspace.resolve(); IDF=a.idf.resolve(); OLD=a.previous_project.resolve()
    WORK.mkdir(parents=True,exist_ok=True)
    private=WORK/"private"; network=json.loads((private/"network.json").read_text())
    assert 0<len(network["ssid"].encode())<=32 and 8<=len(network["password"].encode())<=63
    header=private/"network.h"
    header.write_text('#define S3_TEST_SSID '+json.dumps(network["ssid"])+
        '\n#define S3_TEST_PASSWORD '+json.dumps(network["password"])+ '\n')
    header.chmod(0o600)
    base=WORK/a.variant; project=base/"project"; build=base/"build"
    shutil.copytree(SOURCE/"probe",project,dirs_exist_ok=True)
    shutil.copy2(OLD/"partitions.csv",project/"partitions.csv")
    settings=[]
    for line in (OLD/"sdkconfig").read_text().splitlines():
        if line.startswith(("CONFIG_SECURE_","# CONFIG_SECURE_","CONFIG_PARTITION_TABLE",
                            "# CONFIG_PARTITION_TABLE","CONFIG_ESP_CONSOLE_","# CONFIG_ESP_CONSOLE_")):
            if line.startswith("CONFIG_SECURE_BOOT_SIGNING_KEY="):
                line="CONFIG_SECURE_BOOT_SIGNING_KEY="+json.dumps(str(OLD/"secure_boot_signing_key.pem"))
            settings.append(line)
    settings += ['CONFIG_IDF_TARGET="esp32s3"','CONFIG_COMPILER_OPTIMIZATION_SIZE=y',
                 '# CONFIG_BT_ENABLED is not set','# CONFIG_SPIRAM is not set',
                 'CONFIG_ESPTOOLPY_FLASHSIZE_2MB=y','CONFIG_ESP_WIFI_NVS_ENABLED=n']
    (project/"sdkconfig.defaults").write_text('\n'.join(settings)+'\n')
    cmd=[sys.executable, IDF/"tools/idf.py", "-C",project,"-B",build,"-DIDF_TARGET=esp32s3",
         "-DS3_PRIVATE_DIR="+str(private)]
    if a.variant=="reviewed":
        cmd += ["-DS3_HAL_ARCHIVE="+str(archive())]
        # The pp imported target is scoped to its component directory. Override
        # that component locally and change only the archive property there.
        # All other component files still resolve to the unchanged SDK files.
        component=IDF/"components/esp_wifi"
        overlay=project/"components/esp_wifi"; overlay.mkdir(parents=True,exist_ok=True)
        for child in component.iterdir():
            if child.name=="CMakeLists.txt":continue
            link=overlay/child.name
            if not link.exists():link.symlink_to(child,target_is_directory=child.is_dir())
        component_cmake=(component/"CMakeLists.txt").read_text().replace(
            "../wpa_supplicant",str(IDF/"components/wpa_supplicant"))
        (overlay/"CMakeLists.txt").write_text(component_cmake+
            '\nset_property(TARGET pp PROPERTY IMPORTED_LOCATION "${S3_HAL_ARCHIVE}")\n')
    run(cmd+["app"])
    app=build/"s3_hal_e2e.bin"
    assert app.stat().st_size<=0x100000
    run([sys.executable,"-m","espsecure","verify_signature","--version","2","--keyfile",OLD/"secure_boot_signing_key.pem",app])
    (base/"build-report.json").write_text(json.dumps({"variant":a.variant,
        "app":str(app),"app_sha256":sha(app),"bytes":app.stat().st_size,"offset":"0x20000",
        "elf_sha256":sha(build/"s3_hal_e2e.elf"),"hal_source_sha256":sha(SOURCE/"src/hal_mac.c"),
        "signed":True,"device_tested":False},indent=2))

if __name__=="__main__":main()
