// Hardware test of the web app's core logic (the <script id="lisa-core">
// block of web/index.html) against a real demo board, through bridge.py.
//
//   python3 bridge.py /dev/cu.usbmodemXXXX &     # TCP 5555 <-> the board
//   node hw_test.mjs                             # all scenarios
//   node hw_test.mjs connect debug disconnect    # some of them
//
// Scenarios: connect, direct, vialisa, verify, debug, sdcc, ttlc, tick, elevator, disconnect.  "direct"
// and "vialisa" program the bring-up demo (an altered copy, then the
// original) into the flash at address 0 - the flash contents are replaced.
// "sdcc" programs firmware/sdcc_hello.ihx (C, sdcc -mlisa).  "ihx=<file>"
// programs any Intel HEX file, runs it and waits for it to print
// "ALL PASSED" or "SOME FAILED" (the lisa-tools/sdcc_test programs), e.g.
//   node hw_test.mjs connect ihx=../../../lisa-tools/sdcc_test/test_core.ihx disconnect
// "spiram=<file>" does the same with the data cache on the RP2040's emulated
// SPI RAM (needs the mbell_micropython lisa_spi_ram build on the board);
// "monitor=<file>" starts a program that way and leaves the console on the
// bridge for a terminal (lisa-tools/lisa_monitor).
import fs from 'node:fs';
import net from 'node:net';
import vm from 'node:vm';
import path from 'node:path';
import { once } from 'node:events';
import { fileURLToPath } from 'node:url';

const WEB = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', 'web');
const html = fs.readFileSync(WEB + '/index.html', 'utf8');
const core = html.match(/<script id="lisa-core">([\s\S]*?)<\/script>/)[1];
const assetsJs = fs.readFileSync(WEB + '/assets.js', 'utf8');
globalThis.window = globalThis;
vm.runInThisContext(assetsJs);
vm.runInThisContext(core);
const { LisaCommander, parseFirmware, hex2, hex4 } = globalThis.LisaCore;

class TcpSerialPort {
  async open() {
    this.sock = net.connect(5555, '127.0.0.1');
    this.sock.setNoDelay(true);
    await once(this.sock, 'connect');
    const sock = this.sock;
    this.readable = new ReadableStream({
      start(ctrl) {
        sock.on('data', d => ctrl.enqueue(new Uint8Array(d)));
        sock.on('close', () => { try { ctrl.close(); } catch (e) {} });
        sock.on('error', e => { try { ctrl.error(e); } catch (e2) {} });
      },
      cancel() {},
    });
    this.writable = new WritableStream({ write: chunk => new Promise(res => sock.write(Buffer.from(chunk), res)) });
  }
  async setSignals() {}
  async close() { this.sock.destroy(); }
}

const t0 = Date.now();
const stamp = () => ((Date.now() - t0) / 1000).toFixed(2).padStart(6);
const log = (m, cls) => console.log(`${stamp()} ${cls === 'err' ? '!! ' : cls === 'dim' ? '   ' : '-- '}${m}`);

let consoleBuf = '';
const cmdr = new LisaCommander({ assets: globalThis.LISA_ASSETS, log, onConsole: u8 => { consoleBuf += Buffer.from(u8).toString('latin1'); } });
const sleep = ms => new Promise(r => setTimeout(r, ms));
async function waitConsole(re, timeout = 3000) {
  const t = Date.now();
  while (!re.test(consoleBuf)) { if (Date.now() - t > timeout) throw new Error(`console never matched ${re}; have ${JSON.stringify(consoleBuf.slice(0, 300))} ... ${JSON.stringify(consoleBuf.slice(-120))}`); await sleep(20); }
  return consoleBuf;
}
const check = (cond, msg) => { if (!cond) throw new Error('CHECK FAILED: ' + msg); console.log(`${stamp()} OK ${msg}`); };

const fw = parseFirmware('bringup_tt07', new TextEncoder().encode(globalThis.LISA_ASSETS.firmware.bringup_tt07));
// altered image: "LISA!" -> "LISA*"
const alt = Uint8Array.from(fw.bytes);
{ const w = globalThis.LisaCore.toWords(fw.bytes); const i = w.findIndex((v, k) => v === 0x8041 && w[k + 2] === 0x8021); alt[2 * (i + 2)] = 0x2a; }

const scenarios = process.argv.slice(2).length ? process.argv.slice(2) : ['connect', 'direct', 'vialisa', 'verify', 'debug', 'ttlc', 'tick', 'elevator', 'disconnect'];
// "clock=<MHz>[/<RP2040 MHz>]" before "connect": the project clock to run at
// instead of the SDK's 50 MHz (lisa_select raises the RP2040 to twice it for
// its PWM, or to the system clock given)
let clockHz = 0, sysclkHz = 0;
try {
  for (const sc of scenarios) {
    console.log(`\n===== ${sc} =====`);
    if (sc.startsWith('clock=')) {
      const [c, s] = sc.slice(6).split('/');
      clockHz = Math.round(+c * 1e6);
      sysclkHz = s ? Math.round(+s * 1e6) : 0;
      log(`project clock ${clockHz / 1e6} MHz${sysclkHz ? `, RP2040 at ${sysclkHz / 1e6} MHz,` : ''} for the connect that follows`);
    } else if (sc === 'connect') {
      await cmdr.connect(new TcpSerialPort(), { clock: clockHz, sysclk: sysclkHz });
      check(cmdr.state().pass && cmdr.info.debugger === 'lisav1.2', `connected; debugger ${cmdr.info.debugger}, flash ${cmdr.info.flashId}, clock ${cmdr.info.clock}`);
    } else if (sc === 'direct') {
      const t = Date.now();
      await cmdr.programDirect(alt, 0, (d, n) => { if (d === n) log(`progress ${d}/${n}`); });
      log(`direct programming took ${(Date.now() - t) / 1000} s`);
      consoleBuf = '';
      await cmdr.run();
      await cmdr.consoleSend(new TextEncoder().encode('?'));
      const out = await waitConsole(/Hello from TT07 LISA./);
      check(/Hello from TT07 LISA\*/.test(out), 'banner after direct programming says LISA* (altered image)');
      console.log(out.replace(/\r/g, ''));
    } else if (sc === 'vialisa') {
      const t = Date.now();
      let last = 0;
      await cmdr.programViaLisa(fw.bytes, 0, (d, n) => { if (d - last >= 200 || d === n) { last = d; log(`progress ${d}/${n}`); } });
      log(`via-LISA programming took ${(Date.now() - t) / 1000} s`);
      consoleBuf = '';
      await cmdr.run();
      await cmdr.consoleSend(new TextEncoder().encode('?'));
      const out = await waitConsole(/Hello from TT07 LISA./);
      check(/Hello from TT07 LISA!/.test(out), 'banner after via-LISA programming says LISA! (original image)');
    } else if (sc === 'sdcc') {
      const demo = parseFirmware('sdcc_hello.ihx', new TextEncoder().encode(globalThis.LISA_ASSETS.firmware['sdcc_hello.ihx']));
      check(demo.format === 'Intel HEX' && demo.words > 1000, `sdcc_hello.ihx parsed as Intel HEX: ${demo.words} words`);
      const t = Date.now();
      await cmdr.programDirect(demo.bytes, 0, (d, n) => { if (d === n) log(`progress ${d}/${n}`); });
      log(`direct programming took ${(Date.now() - t) / 1000} s`);
      consoleBuf = '';
      await cmdr.run();
      await waitConsole(/sdcc_hello ready/, 4000);
      check(true, 'C firmware printed its reset banner');
      await cmdr.consoleSend(new TextEncoder().encode('?'));
      await waitConsole(/compiled by sdcc -mlisa/, 4000);
      check(/jgs/.test(consoleBuf), 'owl banner from the C firmware');
      await cmdr.consoleSend(new TextEncoder().encode('s'));
      // wait for the end of the line: the digits arrive one UART byte at a time
      const out = await waitConsole(/Count: \d+ sum: \d+\r?\n/, 4000);
      const m = out.match(/Count: (\d+) sum: (\d+)\r?\n/);
      check(+m[2] === '?'.charCodeAt(0) + 's'.charCodeAt(0), `printf %u works on the chip: ${m[0]} (sum of '?' and 's')`);
    } else if (sc.startsWith('ihx=')) {
      const file = sc.slice(4);
      const prog = parseFirmware(path.basename(file), fs.readFileSync(file));
      check(prog.format === 'Intel HEX', `${file}: ${prog.words} words`);
      const t = Date.now();
      await cmdr.programDirect(prog.bytes, 0, (d, n) => { if (d === n) log(`progress ${d}/${n}`); });
      log(`direct programming took ${(Date.now() - t) / 1000} s`);
      consoleBuf = '';
      await cmdr.run();
      const out = await waitConsole(/ALL PASSED|SOME FAILED/, 20000);
      console.log(out.replace(/\r/g, ''));
      check(/ALL PASSED/.test(out), `${path.basename(file)} reports ALL PASSED on the chip`);
    } else if (sc.startsWith('run=')) {
      // program an interactive image for the 128-byte RAM, start it and leave
      // the console on the bridge (lisa_monitor's monitor_small.ihx)
      const file = sc.slice(4);
      const prog = parseFirmware(path.basename(file), fs.readFileSync(file));
      check(prog.format === 'Intel HEX', `${file}: ${prog.words} words`);
      await cmdr.programDirect(prog.bytes, 0, () => {});
      consoleBuf = '';
      await cmdr.run();
      log(`${path.basename(file)} is running; the console stays on the bridge:`);
      log('    socat -,raw,echo=0 TCP:localhost:5555      (or: nc localhost 5555; press Enter for the banner)');
      process.exit(0);
    } else if (sc.startsWith('spiram=') || sc.startsWith('monitor=')) {
      // like ihx=, with the data cache on the RP2040's emulated SPI RAM (the
      // mbell_micropython lisa_spi_ram build: rp2.enable_sim_spi_ram): CE1 on
      // uio[4], CS1 plain SPI with 16-bit addresses in SPI mode 1, the
      // debugger's QSPI port on CS1 for a pattern check, then the cache on.
      // "monitor=<file>" starts an interactive program the same way and
      // leaves the console attached to the bridge (socat/nc to port 5555).
      const monitor = sc.startsWith('monitor=');
      // <file>[@<project MHz>][,<reg>=<value>...], e.g. spiram=t.ihx,0x1e=0x1ff1
      // overrides a register below (0x1e: spi_mode[12:11], ce_delay[10:4], clk_div[3:0])
      const [spec, ...overrides] = sc.slice(sc.indexOf('=') + 1).split(',');
      const [file, mhz] = spec.split('@');
      const prog = parseFirmware(path.basename(file), fs.readFileSync(file));
      check(prog.format === 'Intel HEX', `${file}: ${prog.words} words`);
      await cmdr.programDirect(prog.bytes, 0, () => {});
      if (mhz) await cmdr.lisa.sideband(`tt.clock_project_PWM(${Math.round(+mhz * 1e6)}); print("@ok=1")`);
      if (overrides.length) {
        // the app's spiRamCache with other register values (0x1e: spi_mode[12:11], ce_delay[10:4], clk_div[3:0])
        const { r } = await cmdr.lisa.sideband('import rp2; print("@ok=%d" % rp2.enable_sim_spi_ram())');
        check(r.ok === '1', 'simulated SPI RAM enabled on the RP2040');
        const regs = [[0x1c, 0x0003], [0x17, 0x0024], [0x1e, 0x1ff1], [0x16, 0x0002]];
        for (const o of overrides) {
          const [a, v] = o.split('=').map(Number);
          const reg = regs.find(([ra]) => ra === a);
          check(reg !== undefined, `register 0x${a.toString(16)} = 0x${v.toString(16)} (override)`);
          reg[1] = v;
        }
        for (const [a, v] of regs) await cmdr.lisa.writeReg(a, v);
        for (const [a, v] of regs) check((await cmdr.lisa.readReg(a)) === v, `register 0x${a.toString(16)} = 0x${v.toString(16)}`);
        const ce = await cmdr.lisa.readReg(0x15);
        await cmdr.lisa.writeReg(0x15, (ce & 0xfffc) | 2);     // the data cache (lisa2) on CS1
        await cmdr.lisa.writeReg(0x1d, 0x0013);                // invalidate it, cache on, 32K map
        await cmdr.lisa.writeReg(0x16, 0x0001);                // the debugger back on the flash
      } else {
        await cmdr.spiRamCache(true);                          // what the app does for a --tt07-cache image
      }
      check(((await cmdr.lisa.readReg(0x1d)) & 0x7) === 0x3, 'data cache enabled on CS1');
      consoleBuf = '';
      await cmdr.run();
      if (monitor) {
        log(`${path.basename(file)} is running with the data cache on the SPI RAM; the console stays on the bridge:`);
        log('    socat -,raw,echo=0 TCP:localhost:5555      (or: nc localhost 5555; press Enter for the banner)');
        process.exit(0);
      }
      let out;
      try {
        out = await waitConsole(/ALL PASSED|SOME FAILED/, 60000);
      } catch (e) {
        // hung: is LISA still talking to the RAM?
        for (let k = 0; k < 3; k++) {
          const { r: rp } = await cmdr.lisa.sideband('import rp2; print("@rep=%d,%d,%d" % rp2.report_sim_spi_ram())');
          log(`SPI RAM counters ${rp.rep}`, 'dim');
          await sleep(500);
        }
        throw e;
      }
      console.log(out.replace(/\r/g, ''));
      const { r: rep } = await cmdr.lisa.sideband('import rp2; print("@rep=%d,%d,%d" % rp2.report_sim_spi_ram())');
      log(`SPI RAM saw ${rep.rep} (commands, last, unknown)`, 'dim');
      check(/ALL PASSED/.test(out), `${path.basename(file)} reports ALL PASSED on the chip with the data cache on the SPI RAM`);
      await cmdr.lisa.halt();
      await cmdr.lisa.writeReg(0x1d, 0x0007);                // cache off again
    } else if (sc === 'verify') {
      let t = Date.now();
      let bad = await cmdr.verifyViaLisa(fw.bytes, 0);
      check(bad.length === 0, `verify via LISA: 0 mismatches (${(Date.now() - t) / 1000} s)`);
      t = Date.now();
      bad = await cmdr.verifyDirect(fw.bytes, 0);
      check(bad.length === 0, `verify direct: 0 mismatches (${(Date.now() - t) / 1000} s)`);
      bad = await cmdr.verifyViaLisa(alt, 0);
      check(bad.length === 1, `verify via LISA against the altered image reports exactly 1 mismatch: ${JSON.stringify(bad)}`);
    } else if (sc === 'debug') {
      consoleBuf = '';
      await cmdr.run();
      await sleep(300);
      await cmdr.consoleSend(new TextEncoder().encode('s'));
      await waitConsole(/Count:/);
      check(true, 'firmware answered "s" while running: ' + JSON.stringify(consoleBuf.slice(-40)));
      const r1 = await cmdr.readRegs();
      check(!r1.halted && cmdr.state().lisaHasUart, `registers read while running (PC ${r1.pc.toString(16)}), UART handed back to LISA`);
      await cmdr.halt();
      const r2 = await cmdr.readRegs();
      check(r2.halted, `halted at PC 0x${r2.pc.toString(16)} SP 0x${r2.sp.toString(16)}`);
      // single steps: each must complete, and land on an address the opcode (from the image) can lead to
      const words = globalThis.LisaCore.toWords(fw.bytes);
      let prev = r2;
      for (let i = 0; i < 12; i++) {
        const r = await cmdr.step();
        // a store runs on past itself (the TT07 breakpoint hazard): then the
        // step's own targets, which it moved, are the addresses to expect
        const D = globalThis.LisaCore.LisaDebug;
        const exp = D.isStore(words[prev.pc]) ? r.step.targets : D.nextPCs(words[prev.pc], prev.pc, prev.ra, prev.ix);
        check(r.step.completed && exp.includes(r.pc) && r.halted,
          `step ${i + 1}: ${words[prev.pc].toString(16).padStart(4, '0')} at ${prev.pc.toString(16)} -> PC ${r.pc.toString(16)} (expected one of ${exp.map(x => x.toString(16)).join('/')})`);
        prev = r;
      }
      // data RAM: write a pattern through the debugger's address path, read it back through IX
      for (const [a, v] of [[7, 0x10], [6, 0x11], [6, 0x22], [6, 0x33], [6, 0x44]]) await cmdr.lisa.writeReg(a, v);
      const ixBefore = await cmdr.lisa.readReg(5);
      const got = await cmdr.lisa.readRam(0x10, 4);
      check(got.join() === '17,34,51,68' && await cmdr.lisa.readReg(5) === ixBefore, `RAM read-back via IX: ${got.map(b => b.toString(16)).join(' ')}, IX restored`);
      const s = await cmdr.readRam();
      check(s.bytes.length === 128, `128-byte RAM dump, SP ${s.sp.toString(16)}: ${s.bytes.slice(0x60, 0x80).map(b => b.toString(16).padStart(2, '0')).join(' ')}`);
      await cmdr.reset();
      const r5 = await cmdr.readRegs();
      check(r5.pc === 0, `reset: PC ${r5.pc}`);
      // from reset: br +8 -> ldx (two words) -> xchg_sp; the step must skip ldx's operand word
      const pcs = []; for (let i = 0; i < 3; i++) pcs.push((await cmdr.step()).pc);
      check(pcs.join() === '8,10,11', `steps from reset land at ${pcs.map(p => p.toString(16)).join(' ')} (br, ldx+operand, xchg_sp)`);
      const r6 = await cmdr.readRegs();
      check(r6.code && r6.code.words.length === 8, 'code window read: ' + globalThis.LisaCore.disassembleWindow(r6.code.words, r6.code.start).map(l => l.text.trim()).join(' | '));
      await cmdr.reset();
      consoleBuf = '';
      await cmdr.resume();
      await cmdr.consoleSend(new TextEncoder().encode('?'));
      await waitConsole(/Hello from TT07 LISA[!*]/);     // whichever image is in the flash
      check(true, 'banner after resume');
      await cmdr.reclaim();
      check(!cmdr.state().lisaHasUart, 'UART reclaimed');
      await cmdr.grant();
      check(cmdr.state().lisaHasUart, 'UART granted');
      // typed debugger command through the console path (after reclaiming)
      await cmdr.reclaim();
      consoleBuf = '';
      await cmdr.consoleSend(new TextEncoder().encode('v'));
      await waitConsole(/lisav1\.2/);
      check(true, 'manual "v" typed in the console answered ' + JSON.stringify(consoleBuf));
      await cmdr.initLisa();
      check(cmdr.state().halted, 're-init works');
    } else if (sc === 'ttlc') {
      const { ttlcDisassemble } = globalThis.LisaCore;
      const enc = s => new TextEncoder().encode(s);
      const lb = parseFirmware('ttlc_loopback.hex', enc(globalThis.LISA_ASSETS.ttlc_firmware['ttlc_loopback.hex']));
      let t = Date.now();
      await cmdr.programTtlc(lb.bytes, 0x10000, 3, true);
      log(`TTLC loopback programmed directly at 0x10000 in ${(Date.now() - t) / 1000} s`);
      const st0 = await cmdr.ttlcStatus();
      check(!st0.run && st0.pc === 0 && cmdr.state().ttlc.enabled, `TTLC halted at PC 0 after programming; code: ${st0.code.words.slice(0, 3).map(w => ttlcDisassemble(w).trim()).join(' | ')}`);
      const s1 = await cmdr.ttlcStep();
      check(s1.completed && s1.newPC === 1, `TTLC step: ${ttlcDisassemble(s1.op).trim()} at 0 -> PC ${s1.newPC} (nopo stalls, so exactly one)`);
      let p = 1;
      for (let i = 0; i < 4; i++) {
        const s2 = await cmdr.ttlcStep();
        check(s2.completed && (s2.newPC === p + 1 || s2.newPC === p + 2), `TTLC step: ${ttlcDisassemble(s2.op).trim()} at ${p} -> PC ${s2.newPC}`);
        p = s2.newPC;
      }
      // one scan: runs to the nopo at 0 and through it
      const sc = await cmdr.ttlcScan([0]);
      check(sc.completed && sc.newPC === 1, `TTLC scan: stopped after the nopo at PC ${sc.newPC}`);
      await cmdr.ttlcRun(0x123456789abcn);
      check(cmdr.state().ttlc.sim, 'I/O emulator started with the TTLC');
      for (const pat of [0x123456789abcn, 1n, 0x800000000000n, 0xaaaaaaaaaaaan, 0xffffffffffffn, 0n]) {
        await cmdr.ttlcSetInputs(pat); await sleep(60);
        const out = await cmdr.ttlcOutputs();
        check(out === pat, `loopback: inputs ${pat.toString(16).padStart(12, '0')} -> outputs ${out.toString(16).padStart(12, '0')}`);
      }
      consoleBuf = '';
      await cmdr.run();
      await cmdr.consoleSend(enc('?'));
      await waitConsole(/Hello from TT07 LISA./);
      check(true, 'LISA runs and answers on the console while the TTLC scans');
      const out2 = await cmdr.ttlcOutputs();
      check(out2 === 0n, 'emulator still answers (sideband) while LISA owns the UART');
      const h = await cmdr.ttlcHalt();
      check(!h.run, `TTLC halted at PC 0x${h.pc.toString(16)}`);
      // the elevator controller, programmed through LISA at another base
      const el = parseFirmware('elevator6x2.hex', enc(globalThis.LISA_ASSETS.ttlc_firmware['elevator6x2.hex']));
      t = Date.now();
      await cmdr.programTtlc(el.bytes, 0x20000, 3, false);
      log(`elevator controller programmed via LISA at 0x20000 in ${(Date.now() - t) / 1000} s`);
      await cmdr.ttlcRun(0n, 100); await sleep(150);
      let o = await cmdr.ttlcOutputs();
      check(((o >> 32n) & 0x3fn) === 1n && ((o >> 38n) & 0x3fn) === 1n, `both cars at floor 0 (outputs ${o.toString(16)})`);
      const poll = async (pred, ms, what) => {
        const t0 = Date.now(); let v;
        while (Date.now() - t0 < ms) { v = await cmdr.ttlcOutputs(); if (pred(v)) return v; await sleep(25); }
        throw new Error(`${what}: not within ${ms} ms (outputs ${v.toString(16)})`);
      };
      await cmdr.ttlcSetInputs(1n); await sleep(40); await cmdr.ttlcSetInputs(0n);   // F0 up: a car is already there
      o = await poll(v => (v & 1n) || ((v >> 22n) & 1n), 400, 'F0-up latched or served');
      check(true, `F0-up seen (outputs ${o.toString(16)})`);
      o = await poll(v => !(v & 1n) && ((v >> 22n) & 1n), 600, 'served on the spot by car 1');
      check(true, `served on the spot by car 1: indicator off, its door open (outputs ${o.toString(16)})`);
      o = await poll(v => !((v >> 22n) & 1n), 600, 'door closes again');
      check(((o >> 32n) & 0x3fn) === 1n, `door closed, car 1 still at floor 0 (outputs ${o.toString(16)})`);
      await cmdr.ttlcTick(0);
      await cmdr.ttlcHalt();
      // step from a scan boundary through the straight-line controller
      const elWords = globalThis.LisaCore.toWords(el.bytes);
      const nopos = elWords.map((w, a) => (w & 0xf) === 0 ? a : -1).filter(a => a >= 0);
      const sc2 = await cmdr.ttlcScan(nopos);
      check(sc2.completed && sc2.newPC === 1, `elevator: scan stops after the nopo at PC ${sc2.newPC}`);
      let steps = 0, last = sc2.newPC;
      for (let i = 0; i < 12; i++) { const r = await cmdr.ttlcStep(); if (r.completed && r.newPC > last) steps++; last = r.newPC; }
      check(steps === 12, `12 single steps advanced through the program to PC 0x${last.toString(16)}`);
      await cmdr.disableTtlc();
      check(!cmdr.state().ttlc.enabled && !cmdr.state().ttlc.sim, 'TTLC disabled, emulator stopped, uo_out back to LISA');
    } else if (sc === 'tick') {
      // the emulator's tick on input 47, seen through the loopback program; and the
      // uncompensated (16-sample) feed, started fresh, which shows the silicon's 17th sample
      const enc = x => new TextEncoder().encode(x);
      const lb = parseFirmware('ttlc_loopback.hex', enc(globalThis.LISA_ASSETS.ttlc_firmware['ttlc_loopback.hex']));
      await cmdr.programTtlc(lb.bytes, 0x10000, 3, true);
      const toggles = async (label) => {
        const seen = new Set();
        for (let i = 0; i < 12; i++) { seen.add(Number((await cmdr.ttlcOutputs() >> 47n) & 1n)); await sleep(30); }
        check(seen.size === 2, `${label}: output 47 toggles (saw ${[...seen].join(',')})`);
      };
      await cmdr.ttlcRun(0n, 100);
      await toggles('tick 100 ms');
      await cmdr.ttlcTick(0); await sleep(100);
      const off = new Set(); for (let i = 0; i < 6; i++) { off.add(Number((await cmdr.ttlcOutputs() >> 47n) & 1n)); await sleep(30); }
      check(off.size === 1, 'tick off: output 47 steady');
      await cmdr.ttlcHalt(); await cmdr.ttlcSim(false);
      cmdr.ttlc.compensate = false;
      await cmdr.ttlcRun(0x000100010001n);                   // emulator starts fresh in 16-sample mode
      await sleep(60);
      const raw = await cmdr.ttlcOutputs();
      check(raw === 0x000300030003n, `16-sample feed on this silicon reads one position high: ${raw.toString(16).padStart(12, '0')}`);
      await cmdr.ttlcHalt(); await cmdr.ttlcSim(false);
      cmdr.ttlc.compensate = true;
      await cmdr.ttlcRun(0x000100010001n); await sleep(60);
      check((await cmdr.ttlcOutputs()) === 0x000100010001n, '17-sample feed (default): exact');
      await cmdr.ttlcHalt(); await cmdr.disableTtlc();
    } else if (sc === 'elevator') {
      // the 6-floor / 2-car controller, ticking fast (100 ms per floor)
      const enc = x => new TextEncoder().encode(x);
      const fw = parseFirmware('elevator6x2.hex', enc(globalThis.LISA_ASSETS.ttlc_firmware['elevator6x2.hex']));
      await cmdr.programTtlc(fw.bytes, 0x10000, 3, true);
      const floor = (o, base) => { const v = Number((o >> BigInt(base)) & 0x3fn); return v ? Math.log2(v) : -1; };
      const show = o => `car1@${floor(o, 32)} car2@${floor(o, 38)} doors ${(o >> 22n) & 1n}/${(o >> 23n) & 1n} req ${(o & 0x3fffffn).toString(2).padStart(22, '0')}`;
      const press = async n => { await cmdr.ttlcSetInputs(1n << BigInt(n)); await sleep(40); await cmdr.ttlcSetInputs(0n); };
      const until = async (pred, ms, what) => {
        const t0 = Date.now(); let o;
        while (Date.now() - t0 < ms) { o = await cmdr.ttlcOutputs(); if (pred(o)) return o; await sleep(40); }
        throw new Error(`${what}: not within ${ms} ms; last state ${show(o)}`);
      };
      await cmdr.ttlcRun(0n, 100);
      await sleep(150);
      let o = await cmdr.ttlcOutputs();
      check(floor(o, 32) === 0 && floor(o, 38) === 0, 'both cars start at floor 0: ' + show(o));
      await press(5);                                            // hall call: floor 3, up
      o = await until(o => (o >> 5n) & 1n, 500, 'F3-up indicator');
      check(true, 'momentary hall call latched: ' + show(o));
      o = await until(o => !((o >> 5n) & 1n), 3000, 'F3 call serviced');
      check(floor(o, 32) === 3 || floor(o, 38) === 3, 'a car reached floor 3 and cleared the call: ' + show(o));
      check(floor(o, 32) === 0 || floor(o, 38) === 0, 'only one car was dispatched; the other stayed at floor 0: ' + show(o));
      check(((o >> 22n) & 1n) || ((o >> 23n) & 1n), 'its door is open: ' + show(o));
      await sleep(400);                                          // doors close, cars settle
      await press(10);                                           // car 1 cabin: floor 0
      o = await until(o => (o >> 10n) & 1n, 500, 'cabin-1 floor-0 indicator');
      o = await until(o => !((o >> 10n) & 1n), 3000, 'car 1 back at floor 0');
      check(floor(o, 32) === 0, 'car 1 answered its cabin button and is at floor 0: ' + show(o));
      await sleep(400);
      await press(21);                                           // car 2 cabin: floor 5
      o = await until(o => !((o >> 21n) & 1n), 3000, 'car 2 at floor 5');
      check(floor(o, 38) === 5, 'car 2 went to floor 5: ' + show(o));
      await sleep(400);
      await press(2);                                            // hall call: floor 1, down
      o = await until(o => !((o >> 2n) & 1n), 3000, 'F1-down call serviced');
      check(floor(o, 32) === 1 && floor(o, 38) === 5, 'F1-down dispatched to car 1 (idle at 0, within two floors); car 2 stayed at 5: ' + show(o));
      await cmdr.ttlcTick(0);
      await cmdr.ttlcHalt();
      await cmdr.disableTtlc();
    } else if (sc.startsWith('backup=')) {
      // the RP2040's flash read back as the app's "Back up RP2040" does
      // (rp2FlashInfo + readRp2Flash), written as a .uf2; the firmware part
      // is compared with the embedded lisa_spi_ram build when that is what
      // the board runs
      const file = sc.slice(7);
      const info = await cmdr.rp2FlashInfo();
      check(info.bytes >= (2 << 20) && info.uid === cmdr.info.uid, `RP2040 flash ${info.bytes >> 20} MB, unique id ${info.uid}`);
      const t = Date.now();
      let last = 0;
      const bytes = await cmdr.readRp2Flash(info.bytes, (d, n) => { if (d - last >= (256 << 10) || d === n) { last = d; log(`progress ${d >> 10}/${n >> 10} KB`); } });
      const secs = (Date.now() - t) / 1000;
      log(`read back in ${secs.toFixed(1)} s (${(info.bytes / 1024 / secs).toFixed(0)} KB/s)`);
      const uf2 = globalThis.LisaCore.makeUf2(bytes);
      fs.writeFileSync(file, uf2);
      check(globalThis.LisaCore.isUf2(uf2) && uf2.length === info.bytes * 2, `${file}: ${uf2.length} bytes, ${uf2.length / 512} UF2 blocks`);
      check(bytes[0] !== 0xff && bytes.subarray(0, 256).some(b => b !== bytes[0]), 'block 0 holds code (boot2), not blank flash');
      const emb = Buffer.from(globalThis.LISA_ASSETS.rp2040.uf2, 'base64');
      let same = 0, blocks = emb.length / 512;
      for (let i = 0; i < blocks; i++) {
        const target = emb.readUInt32LE(i * 512 + 12) - 0x10000000;
        if (Buffer.compare(emb.subarray(i * 512 + 32, i * 512 + 288), Buffer.from(bytes.subarray(target, target + 256))) === 0) same++;
      }
      log(`${same} of the embedded build's ${blocks} blocks match the flash: the board ${same === blocks ? 'runs' : 'does not run'} ${globalThis.LISA_ASSETS.rp2040.name}`);
      check(same === blocks || !cmdr.info.spiRam, 'a board with the emulation runs the embedded build');
    } else if (sc.startsWith('hazard=')) {
      // the TT07 breakpoint hazard on sdcc_test's test_core.ihx: stop at
      // line 31 (0x2b5: ldi #4; push a; ldi #3; jal shl), Step across the
      // push - it must land (a halt right after it would lose it) - run on
      // to ALL PASSED, then Halt the final loop by breakpoint
      const file = sc.slice(7);
      const prog = parseFirmware(path.basename(file), fs.readFileSync(file));
      await cmdr.programDirect(prog.bytes, 0, () => {});
      const L = cmdr.lisa;
      await L.haltNow();
      await L.send('t'); await sleep(50); await L.version();
      for (let i = 0; i < 4; i++) await L.writeReg(8 + i, 0);
      await L.writeReg(8, 0x8000 | 0x2b5);
      consoleBuf = '';
      await L.resume(); await L.grant();
      await sleep(1500);
      await L.reclaim();
      let r = await L.readRegs();
      check(r.halted && r.pc === 0x2b5, `stopped at line 31 (PC 0x${hex4(r.pc)}) after ${JSON.stringify(consoleBuf.trim())}`);
      await L.writeReg(8, 0);
      await L.writeReg(7, 0x75); await L.writeReg(6, 0x99); await L.writeReg(7, 0);   // a marker in the argument's slot
      const s1 = await L.step();
      check(s1.completed && s1.newPC === 0x2b6 && !s1.past, `step ldi #4 -> 0x${hex4(s1.newPC)}`);
      const s2 = await L.step();
      check(s2.completed && s2.past && s2.newPC === 0x2b8, `step push a ran on past the store to 0x${hex4(s2.newPC)}`);
      const slot = (await L.readRam(0x75, 1))[0];
      check(slot === 4, `the push landed: RAM[0x75] = 0x${hex2(slot)} (0x99 = lost)`);
      consoleBuf = '';
      await L.resume(); await L.grant();
      await waitConsole(/ALL PASSED|SOME FAILED/, 8000);
      check(/ALL PASSED/.test(consoleBuf) && !/FAIL/.test(consoleBuf), 'test_core still passes after the steps: ' + consoleBuf.trim().split('\n').slice(-2).join(' | '));
      await L.reclaim();
      const h = await L.halt();
      r = await L.readRegs();
      check(h.safe && r.halted, `Halt by breakpoint on the running loop (PC sampled ${h.samples.map(hex4).join(' ')}, planted ${h.cand.map(hex4).join(' ')}) -> PC 0x${hex4(r.pc)}`);
    } else if (sc === 'disconnect') {
      await cmdr.disconnect();
      check(!cmdr.connected, 'disconnected');
    }
  }
  console.log('\nALL SCENARIOS PASSED');
} catch (e) {
  console.log('\nFAILED:', e.stack || e);
  try { await cmdr.disconnect(); } catch (e2) {}
  process.exit(1);
}
process.exit(0);
