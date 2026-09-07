#!/usr/bin/env node
// Read-only live validation: subscribe through Foxglove; never contend for IPC consumer slots.
// Requires Node.js and ws (Debian node-ws, or npm install --prefix /tmp/talos-protocol-check ws).
// For a temporary npm install, set NODE_PATH=/tmp/talos-protocol-check/node_modules.
// Usage: node scripts/check_talos_protocol.mjs [seconds] [ws://localhost:8765]
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
const WebSocket = createRequire(import.meta.url)('ws');
import { readFileSync, statSync } from 'node:fs';

const seconds = Number(process.argv[2] ?? 15);
assert(Number.isFinite(seconds) && seconds > 0 && seconds <= 300, 'duration must be 0..300 seconds');
const header = readFileSync('/tmp/talos_ipc_meta').subarray(0, 64);
assert.equal(header.readUInt32LE(4), 7, 'Talos protocol mismatch: expected v7');
assert.equal(header.readUInt32LE(0), 0x54414c07, 'invalid v7 magic');
assert.equal(statSync('/tmp/talos_ipc_meta').size, 74752, 'v7 metadata ABI size mismatch');
const topics = ['/referee/self', '/simulation/combat/evaluation'];
const socket = new WebSocket(process.argv[3] ?? 'ws://127.0.0.1:8765', ['foxglove.sdk.v1', 'foxglove.websocket.v1']);
socket.binaryType = 'arraybuffer';
const received = [0, 0];
const rounds = new Map();
const frames = new Map();
let matchedFrames = 0;
let failure;
let schemas = 0;
let timer;
function roundRecord(id) {
  if (!rounds.has(id)) rounds.set(id, { samples: new Map(), states: new Map(), events: new Map(), repeated_samples: 0 });
  return rounds.get(id);
}
socket.addEventListener('open', () => { timer = setTimeout(() => socket.close(), seconds * 1000); });
socket.addEventListener('error', event => { failure = String(event.error ?? event.message ?? 'WebSocket connection failed'); });
socket.addEventListener('message', ({ data }) => {
  try {
    if (typeof data === 'string') {
      const message = JSON.parse(data);
      if (message.op === 'advertise') {
        const subscriptions = [];
        for (const channel of message.channels) {
          const id = topics.indexOf(channel.topic);
          if (id < 0) continue;
          assert.equal(channel.encoding, 'json');
          const schema = JSON.parse(channel.schema);
          assert(schema.properties.round_id && schema.properties.sequence, 'missing association fields');
          schemas++;
          subscriptions.push({ id: id + 1, channelId: channel.id });
        }
        if (subscriptions.length) socket.send(JSON.stringify({ op: 'subscribe', subscriptions }));
      }
      return;
    }
    const buffer = Buffer.from(data);
    if (buffer[0] !== 1) return;
    const channel = buffer.readUInt32LE(1) - 1;
    if (channel < 0 || channel >= 2) return;
    const message = JSON.parse(buffer.subarray(13).toString());
    received[channel]++;
    assert(message.round_id > 0 && message.round_started_ns <= message.sim_time_ns);
    const association = JSON.stringify([message.round_id, message.sim_time_ns, message.round_started_ns, message.timestamp]);
    if (frames.has(message.sequence)) {
      assert.equal(frames.get(message.sequence), association, 'combat channels disagree on source frame');
      matchedFrames++;
    } else frames.set(message.sequence, association);
    // Only recent association keys are needed; data is never retained indefinitely.
    while (frames.size > 64) frames.delete(frames.keys().next().value);
    const round = roundRecord(message.round_id);
    if (channel === 0) {
      assert(!('robots' in message) && !('events' in message), 'enemy truth in referee channel');
      assert(message.sample_time_ns >= message.round_started_ns && message.sample_time_ns <= message.sim_time_ns);
      if (!message.valid) return;
      assert(!('damage_dealt' in message.self), 'evaluation attribution in referee channel');
      const signature = JSON.stringify([message.sample_time_ns, message.self]);
      const previous = round.samples.get(message.sample_sequence);
      if (previous) { assert.equal(signature, previous.signature, 'sample changed without new sample sequence'); round.repeated_samples++; }
      round.samples.set(message.sample_sequence, { time: message.sample_time_ns, signature });
      assert(message.self.allowance_mode !== 'unlimited' || message.self.allowance_remaining === null);
    } else {
      for (const robot of message.robots) {
        assert(robot.hp >= 0 && robot.hp <= robot.max_hp);
        assert.equal(robot.fire_permitted, robot.fire_blocks === 0);
        assert.equal(robot.alive, robot.hp > 0);
        const previous = round.states.get(robot.robot_id);
        if (previous) {
          assert(robot.actual_shots >= previous.actual_shots);
          assert(robot.rejected_requests >= previous.rejected_requests);
          assert(robot.damage_dealt >= previous.damage_dealt);
        }
        round.states.set(robot.robot_id, robot);
      }
      assert(message.events.length <= 64);
      for (const event of message.events) {
        assert.equal(event.round_id, message.round_id);
        assert(event.round_time_ns <= message.sim_time_ns - message.round_started_ns);
        const signature = JSON.stringify(event);
        if (round.events.has(event.id)) assert.equal(round.events.get(event.id), signature);
        round.events.set(event.id, signature);
      }
    }
  } catch (error) { failure = error.stack; socket.close(); }
});
await new Promise(resolve => socket.addEventListener('close', resolve));
clearTimeout(timer);
assert(!failure, failure);
assert.equal(schemas, 2, 'both combat schemas must be advertised');
assert(received.every(n => n > 0), 'both combat channels must publish');
assert(matchedFrames > 0, 'no matching frames across combat channels');
const summary = { version: 7, subprotocol: socket.protocol, metadata_bytes: 74752, received, matched_frames: matchedFrames, rounds: [] };
for (const [id, round] of rounds) {
  const entries = [...round.samples.entries()].sort((a, b) => a[0] - b[0]);
  let hz = null;
  if (entries.length > 2) {
    hz = (entries.at(-1)[0] - entries[0][0]) * 1e9 / (entries.at(-1)[1].time - entries[0][1].time);
    assert(hz >= 9.7 && hz <= 10.3, `unexpected simulation sample rate ${hz}`);
  }
  summary.rounds.push({ id, sample_count: entries.length, sample_hz: hz, repeated_samples: round.repeated_samples,
    event_count: round.events.size, robots: [...round.states.values()] });
}
console.log(JSON.stringify(summary, null, 2));
