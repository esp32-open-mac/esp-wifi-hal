#!/usr/bin/env python3
"""Run host regressions and verify replacement membership, placement and linkage."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile

def output(args):return subprocess.check_output([str(a) for a in args],text=True)
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,default=Path.cwd()/'artifacts/esp32s3/manual-review')
    a=parser.parse_args(); work=a.workspace.resolve(); source=Path(__file__).resolve().parent
    with tempfile.TemporaryDirectory(prefix='s3-hal-host-') as temp:
        executable=Path(temp)/'test-hal'
        subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror',
            '-I'+str(source/'tests'),str(source/'tests/test_hal.c'),'-o',str(executable)],check=True)
        host=output([executable]);print(host,end='');(work/'host-tests.txt').write_text(host)
    compile_command=json.loads((work/'compile-command.json').read_text())
    compiler=Path(compile_command[0]);prefix=compiler.name.removesuffix('gcc')
    ar=compiler.with_name(prefix+'ar');nm=compiler.with_name(prefix+'nm');dump=compiler.with_name(prefix+'objdump')
    archive=json.loads((work/'archive-integrity.json').read_text())
    build=json.loads((work/'reviewed/build-report.json').read_text())
    assert digest(source/'src/hal_mac.c')==archive['source_sha256']==build['hal_source_sha256']
    assert digest(work/'libpp.a')==archive['replacement_sha256']
    assert subprocess.check_output([str(ar),'p',str(work/'libpp.a'),'hal_mac.o'])==(work/'hal_mac.o').read_bytes()
    assert digest(work/'reviewed/build/s3_hal_e2e.bin')==build['app_sha256']
    project=work/'reviewed/project/components/esp_wifi/lib/esp32s3/libpp.a'
    assert digest(project)==archive['original_sha256']
    old=work/'original_hal_mac.o'
    old.write_bytes(subprocess.check_output([str(ar),'p',str(project),'hal_mac.o']))
    def globals(p):
        result={}
        for line in output([nm,'-g','-S','--defined-only',p]).splitlines():
            parts=line.split()
            if len(parts)==4:result[parts[3]]=(parts[2],int(parts[1],16))
        return result
    before,after=globals(old),globals(work/'hal_mac.o')
    assert before.keys()==after.keys()
    assert sum(v[0]=='T' for v in after.values())==57
    for name,record in before.items():
        if record[0]!='T':assert after[name]==record,(name,record,after[name])
    def sections(p):
        result={}
        for line in output([dump,'-t',p]).splitlines():
            s=line.split()
            if len(s)==6 and s[2]=='F':result[s[5]]=s[3]
        return result
    old_sections,new_sections=sections(old),sections(work/'hal_mac.o')
    for name,section in old_sections.items():
        assert new_sections[name]==section,(name,section,new_sections[name])
    elf=work/'reviewed/build/s3_hal_e2e.elf'
    names={}; rom_names={}
    for line in output([nm,'--defined-only',elf]).splitlines():
        s=line.split()
        if len(s)==3 and s[2] in new_sections:
            if s[1] in ('T','t'):names[s[2]]=s[0]
            elif s[1]=='A':rom_names[s[2]]=s[0]
            else:raise AssertionError(('Unexpected HAL symbol type',s))
    map_text=elf.with_suffix('.map').read_text()
    hal_lines=[s for s in map_text.splitlines() if 'libpp.a(hal_mac.o)' in s]
    assert hal_lines and all(str(work/'libpp.a')+'(hal_mac.o)' in s for s in hal_lines)
    placements=[]
    for line in hal_lines:
        match=re.match(r'^\s*(?:\.\S+\s+)?(0x[0-9a-f]+)\s+(0x[0-9a-f]+)\s+',line)
        if match:placements.append(tuple(int(v,16) for v in match.groups()))
    for name,address in names.items():
        assert any(start<=int(address,16)<start+size for start,size in placements),('Missing member placement',name,address)
    result={'passed':True,'host_test_groups':host.count('PASS '),'functions':57,
        'bss_symbols':{n:size for n,(kind,size) in after.items() if kind=='B'},
        'all_function_sections_preserved':True,'stock_hal_member_present':False,
        'retained_functions':names,'rom_resolved_functions':rom_names,
        'source_sha256':digest(source/'src/hal_mac.c'),
        'app_sha256':build['app_sha256'],'other_members_unchanged':len(archive['unchanged_members'])}
    (work/'validation.json').write_text(json.dumps(result,indent=2))
    print(f"PASS archive/link: 57 functions, three BSS symbols, {len(names)} replacement functions retained, {len(rom_names)} names resolved to ROM")

if __name__=='__main__':main()
