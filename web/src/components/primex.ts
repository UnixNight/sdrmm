import type { PrimexFrame } from "../lib/types";

export const PRIMEX_ERRORS: Record<NonNullable<PrimexFrame["error"]>, string> = {
  checksum: "Checksum failed",
  invalid_time: "Invalid time",
  manchester: "Manchester error",
  timing: "Timing lost",
  truncated: "Truncated",
};

const pad = (value: number) => String(value).padStart(2, "0");

export function primexStation(frame: PrimexFrame): string | undefined {
  const bytes = frame.station_id;
  if (bytes == null) return undefined;
  const end = bytes.indexOf(0);
  return String.fromCharCode(...bytes.slice(0, end < 0 ? bytes.length : end)) || undefined;
}

export function primexTime(time: PrimexFrame["time"]): string | undefined {
  if (time == null) return undefined;
  return `${time.year}-${pad(time.month)}-${pad(time.day)} ${pad(time.hour)}:${pad(time.minute)}:${pad(time.second)}`;
}

export function primexSummary(frame: PrimexFrame): string {
  if (frame.error != null) return `Primex ${frame.message_type} · ${PRIMEX_ERRORS[frame.error]}`;
  if (!frame.checksum_ok) return `Primex ${frame.message_type} · Checksum failed`;
  const time = primexTime(frame.time);
  return time == null ? `Primex control ${frame.message_number ?? "?"}` : `Primex · ${time}`;
}

export function primexBytes(frame: PrimexFrame): string {
  return frame.bytes
    .slice(0, frame.length)
    .map((byte) => byte.toString(16).padStart(2, "0"))
    .join(" ")
    .toUpperCase();
}

export function primexReadings(frames: readonly PrimexFrame[]) {
  const time = frames.find(
    (frame) => frame.checksum_ok && frame.error == null && frame.time != null,
  );
  const control = frames.find(
    (frame) => frame.checksum_ok && frame.error == null && frame.message_number != null,
  );
  const station = frames.find((frame) => primexStation(frame) != null);
  return {
    time: primexTime(time?.time),
    control: control?.message_number,
    station: station == null ? undefined : primexStation(station),
  };
}
