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
const { LisaCommander, parseFirmware } = globalThis.LisaCore;

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
  while (!re.test(consoleBuf)) { if (Date.now() - t > timeout) throw new Error(`console never matched ${re}; have ${JSON.stringify(consoleBuf.slice(-200))}`); await sleep(20); }
  return consoleBuf;
}
const check = (cond, msg) => { if (!cond) throw new Error('CHECK FAILED: ' + msg); console.log(`${stamp()} OK ${msg}`); };

const fw = parseFirmware('bringup_tt07', new TextEncoder().encode(globalThis.LISA_ASSETS.firmware.bringup_tt07));
// altered image: "LISA!" -> "LISA*"
const alt = Uint8Array.from(fw.bytes);
{ const w = globalThis.LisaCore.toWords(fw.bytes); const i = w.findIndex((v, k) => v === 0x8041 && w[k + 2] === 0x8021); alt[2 * (i + 2)] = 0x2a; }

const scenarios = process.argv.slice(2).length ? process.argv.slice(2) : ['connect', 'direct', 'vialisa', 'verify', 'debug', 'ttlc', 'tick', 'elevator', 'disconnect'];
try {
  for (const sc of scenarios) {
    console.log(`\n===== ${sc} =====`);
    if (sc === 'connect') {
      await cmdr.connect(new TcpSerialPort());
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
        const exp = globalThis.LisaCore.LisaDebug.nextPCs(words[prev.pc], prev.pc, prev.ra, prev.ix);
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
