export function readNumber(input) {
  return Number.parseFloat(input.value);
}

export function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

export function clampInt(value, min, max) {
  return Math.round(clamp(value, min, max));
}

export function lerp(a, b, t) {
  return a + (b - a) * t;
}

export function almostEqual(a, b) {
  return Math.abs(a - b) < 1e-6;
}

export function generateSeed() {
  return Math.floor(Date.now() % 1000000);
}

export function createRng(seed) {
  let value = seed >>> 0;
  return () => {
    value = (1664525 * value + 1013904223) >>> 0;
    return value / 4294967296;
  };
}
