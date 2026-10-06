// Hardware test of the web app's core logic (the <script id="lisa-core">
// block of web/index.html) against a real demo board, through bridge.py.
//
//   python3 bridge.py /dev/cu.usbmodemXXXX &     # TCP 5555 <-> the board
//   node hw_test.mjs                             # all scenarios
//   node hw_test.mjs connect debug disconnect    # some of them
//
// Scenarios: connect, direct, vialisa, verify, debug, disconnect.  "direct"
// and "vialisa" program the bring-up demo (an altered copy, then the
// original) into the flash at address 0 - the flash contents are replaced.
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

const scenarios = process.argv.slice(2).length ? process.argv.slice(2) : ['connect', 'direct', 'vialisa', 'verify', 'debug', 'disconnect'];
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
