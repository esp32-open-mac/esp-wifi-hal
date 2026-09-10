# ESP32-S3 MAC member review

This is a C reference implementation of the 57 functions and three BSS symbols
in `libpp.a/hal_mac.o`, checked against ESP-IDF v5.4's ESP32-S3 blob. It replaces
that member in a copy of the archive. The other archive members, ROM routines,
PHY, coexistence, crypto and IDF WPA2 stack remain in use.

## Evidence and scope

- IDF: `67c1de1eebe095d554d281952fde63c16ee2dca0` (v5.4).
- Wi-Fi library: `a29b11bf0fe019ca0ade5459714b0b2426dfe020`.
- Original `libpp.a` SHA-256:
  `284e57fbc83ce1b57733733a536ff09bdae99a6ff464eaefb71119080bb1cb39`.
- Original `hal_mac.o` SHA-256:
  `1c574bef587883aee00a6bf4a469e3cba4d43dd57dab50b96fae21308635d863`.
- Analysis ELF uses a synthetic text base of `0x40080000`. Addresses below
  identify instructions in that analysis layout; they are not runtime call targets.
- Initial candidates came from the Rust reconstruction agent using Binary Ninja
  decompilation, assembly and existing open-MAC references. They compiled, with
  9/57 normalized matches. Manual correction used instruction operands, memory
  widths, callback ABI and register-window calling conventions. Decompiled C
  and instruction-alignment scores were insufficient for several functions.

`src/hal_mac.c` contains no absolute RAM or firmware-function addresses.
External data and functions link by symbol. Hardware register addresses remain
explicit. In particular, `wDevCtrl`, `g_wifi_menuconfig` and `g_osi_funcs_p` retain
their original owners. Special IRAM/Wi-Fi sections match the original member;
access helpers are forced inline so an interrupt routine cannot acquire a helper
call into flash accidentally.

## Confirmed corrections

| Function | Assembly evidence | Correction |
|---|---|---|
| `mac_tx_set_duration` | `0x400800cf`–`0x400800e0`: nested word loads followed by `l16ui ...,2` | Dereference the pointer at buffer+44 before testing bit 18; retain both table indirections and the halfword load at entry+2. |
| `mac_tx_set_pti` | `0x4008010b`–`0x40080141`: OS table load, saved `a3`, reload of info, `s32i ...,a1,0` | One OS-table pointer dereference; queue comes from outer TX object+4; minimum uses the pre-callback priority; four arguments use the reloaded priority; seventh argument is the halfword at info+34. |
| `mac_tx_set_plcp0` | `l16si ...,20`, `l8ui ...,12`, explicit masks/shifts | Preserve signed retry field, wrapped byte comparison, all flag branches and the descending 8-byte queue-control stride. |
| `hal_mac_tx_config_edca` | `l8ui ...,5`, `l16ui ...,6` | Distinguish byte and halfword fields; retain unrelated register bits and both RMW operations. |
| `mac_txrx_init` | `l32i` followed by 16-bit masking | Use 32-bit MMIO accesses, even where HLIL renders a halfword conversion. |
| `mac_last_rxbuf_init` | `0x400808fc`–`0x400809a3` | Preserve the interleaved 32-bit filter/config/mask stores. HLIL's two memcpy expressions lose the hardware transaction ordering. |
| `hal_mac_tsf_get_time`, `hal_get_tsf_time` | `a2/a3` return pair; high latch read at `0x6003501c` | Return 64 bits. Preserve interface-specific latch bits and the intentional zero high word for interface 0 of `hal_mac_tsf_get_time`. |
| `hal_mac_tsf_set_time` | Writes from `a4/a5`, with `a3` unused | Model the aligned 64-bit second argument, rather than inventing a meaningful padding argument. |
| `hal_mac_ftm_get_t3` | `muluh`, carry/borrow branches, `a2/a3` return | Keep full-width `(ticks * 80 + fractional) * 8 - 5120`, including unsigned wraparound. |
| `hal_mac_deinit` | Calls at `0x40081175` and `0x4008118d` | Preserve both 20 µs and 5 µs delays and the intervening ready-bit poll. |

The original `hal_init` intentionally passes **zero** to `hal_set_rx_active_pti`.
It passes the event-3 callback byte to RX ACK priority and the event-15 byte to
default priority. The slow-clock callback result is the second argument of
`hal_timer_update_by_rtc(1, value)`. The SDK OS-adapter offsets are checked at
compile time (`0x148`, `0x1a8`).

Several original routines return a leftover register despite being used as void
operations. The reference does not turn those incidental values into an API.
The unusual `hal_mac_is_dma_enable` behavior is retained: read `0x60035128`, then
return zero. It was not “fixed” according to its name.

## Register layout established by this member

All accesses in this table are 32-bit. Names describe observed use, not a complete
hardware specification. `slot` and `interface` mean the indices used by the blob.

[`register-accesses.csv`](register-accesses.csv) records 256 automatically
extracted MMIO observations, including instruction offsets, widths and indexed
address expressions. Extraction is partial; it is not a complete register map.

| Address / expression | Observed use |
|---|---|
| `0x60033000 + 8*interface` | BSSID low word; high word at +4; masks at +0x20/+0x24; enable bit 16 in mask-high. |
| `0x60033040 + 8*interface` | Receiver-address bank with the same mask spacing. |
| `0x60033084` | RX enable, bit 31. |
| `0x60033088`, `0x6003308c`, `0x60033090` | RX descriptor base, next and last. |
| `0x6003309c` | RX configuration; CSI bit 23; last-buffer filtering enable bit 27. |
| `0x600330a0` | Station beacon filter, low three bits. |
| `0x600330d8 + 4*interface` | RX policy; BSSID check bit 1. Init touches four words; `hal_mac_rx_set_policy` rejects indices >=3. |
| `0x6003311c` | Last-buffer filter enables (`0x3f00`, then `0x7e`). |
| `0x60033120 + 4*i`, `0x6003313c + 4*i`, `0x60033158 + 4*i` | Six interleaved filter/config/mask entries. |
| `0x600332b0`, `0x600332b4` | Beacon IE CRC read and write registers. |
| `0x60033404/408`, `0x60033410/414` | Rate tables changed by low-rate enable/disable. |
| `0x60033c34`, `0x60033c3c`, `0x60033c40` | MAC interrupt enable, status and clear. |
| `0x60033ca0` | MAC state; init mask `0xff00efff`, deinit set mask `0x00ff1000`, busy mask `0x6000`. |
| `0x60033d04 - 8*slot` | Timeout low 12 bits; EDCA fields at bits 24–27 and 12–21. |
| `0x60033d08 - 8*slot` | PLCP0 / queue control; valid bit 30, enabled bit 31. |
| `0x60033d14` | Reset/control handshake: set bit 1, poll bit 0. |
| `0x60034318 - 76*slot` | TX duration duplicated into both halfwords. |
| `0x60034334 - 76*slot` | Block-ACK TID/sequence information; bitmap words from `0x60034328/324 - 76*slot`. |
| `0x60035000` | 32-bit MAC time read by `hal_now`. |
| `0x6003500c` | TSF latch/load control. |
| `0x60035010/014` | TSF low/high load words. |
| `0x60035018/01c` | TSF low/high latched read words. |
| `0x60035028/034` | TSF reset/enable control words. |
| `0x6003507c` | Random word read by `hal_random`. |
| `0x6001c860/87c` | PHY low-rate state checks; outside the MAC block. |

## Validation and limits

On 2026-09-10, both the stock member and the replacement passed the same signed
ESP-IDF application on an ESP32-S3 revision 0.1:

- Three passive scans, WPA2-PSK associations and DHCP leases.
- Twenty 512-byte gateway echo requests per cycle: **60/60 replies for each build**.
- TSF values above 32 bits advanced on every cycle.
- Three stop/deinit/reinit cycles completed, with at most a 12-byte span in the reported
  post-cycle free heap for each build.
- Secure-boot verification succeeded. Deployment replaced only the factory
  application at `0x20000`; bootloader, partition table and eFuses were not written.
  The SDK PHY did refresh its calibration data in NVS.

All 57 functions are present in the replacement object. The test firmware's
linker retains 30 replacement functions; this is not per-function execution coverage.
Five other names resolve to ROM: `mac_tx_set_duration`, `mac_tx_set_plcp0`,
`hal_mac_is_low_rate_enabled`, `hal_mac_tx_get_blockack`, and
`hal_mac_is_dma_enable`. Their replacement bodies are discarded in this build.
Host regressions cover the corrected duration and PLCP0 bodies; the device run
does not establish that those bodies work on hardware. The
remaining routines include paths that normal station traffic does not exercise.
Ten host regression groups cover pointer chains, callback mutation, argument
layout, TX flags, EDCA masks, timestamp arithmetic, filter ordering, init and
deinit. These complement device tests; they do not prove hardware equivalence.

The map was checked for the replacement archive path and absence of the stock
`hal_mac.o`. All 29 other archive members were compared byte-for-byte.
See [the recorded validation summary](validation-summary.json) for source/image
hashes, retained symbols and per-cycle results.
This C comparison alone establishes neither a full Wi-Fi deblob nor Rust driver
support. The subsequent Rust port and its separate hardware results are in
[RUST.md](RUST.md).
CSI, FTM hardware exchanges, mesh, AP mode, power saving and mixed Bluetooth/Wi-Fi
coexistence still need dedicated device tests.

## Upstream direction

This evidence addresses [esp-wifi-hal issue #1](https://github.com/esp32-open-mac/esp-wifi-hal/issues/1).
The existing [C3 port PR #22](https://github.com/esp32-open-mac/esp-wifi-hal/pull/22)
already establishes a pattern for chip-specific register layouts, OS callbacks,
TSF and RX validation. Coordinate the S3 port with that work.

The accompanying Rust port adds the chip feature, initialization translation,
S3 RX layout and TX encoding, with FoA station tests. Its separate
[PAC draft](https://github.com/esp-rs/esp-pacs/pull/511) supplies the missing Wi-Fi
register block. The port remains experimental: dedicated power-management,
coexistence, HT rate-sweep and multi-interface tests are still outstanding.
