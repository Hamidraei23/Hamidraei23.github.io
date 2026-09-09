import test from 'node:test';
import assert from 'node:assert/strict';
import { ease, sampleJoints } from '../assets/trajectory.js';

const close = (a, b, tolerance = 1e-8) => assert.ok(Math.abs(a - b) < tolerance, `${a} != ${b}`);

test('timing starts and ends at rest, progresses monotonically, and is symmetric', () => {
  close(ease(0), 0);
  close(ease(1), 1);
  close(ease(.5), .5);
  const h = 1e-4;
  close((ease(h) - ease(0)) / h, 0, 1e-6);
  close((ease(1) - ease(1 - h)) / h, 0, 1e-6);
  for (let i = 1; i <= 100; i++) {
    assert.ok(ease(i / 100) >= ease((i - 1) / 100));
    close(ease(i / 100), 1 - ease(1 - i / 100));
  }
});

test('joint moves reproduce exact endpoints and hold unchanged joints', () => {
  const a = [0, -1, .3], b = [2, -2, .3];
  assert.deepEqual(sampleJoints([a, b], 0), a);
  assert.deepEqual(sampleJoints([a, b], 1), b);
  assert.deepEqual(sampleJoints([a, b], .5), [1, -1.5, .3]);
});

test('Cartesian joint interpolation stays inside each interval, including reversals', () => {
  const samples = [[0, 2], [.1, 1], [.1, -1], [-.5, 3], [1, 3]];
  for (let i = 0; i < 4; i++) {
    for (let k = 0; k <= 100; k++) {
      const q = sampleJoints(samples, (i + k / 100) / 4);
      q.forEach((value, j) => {
        assert.ok(value >= Math.min(samples[i][j], samples[i+1][j]) - 1e-12);
        assert.ok(value <= Math.max(samples[i][j], samples[i+1][j]) + 1e-12);
      });
    }
  }
});

test('joint velocity is continuous across Cartesian sample boundaries', () => {
  const samples = [[0], [.2], [.6], [.4], [.1]], h = 1e-6;
  for (const t of [.25, .5, .75]) {
    const left = (sampleJoints(samples, t)[0] - sampleJoints(samples, t - h)[0]) / h;
    const right = (sampleJoints(samples, t + h)[0] - sampleJoints(samples, t)[0]) / h;
    close(left, right, 1e-4);
  }
});
