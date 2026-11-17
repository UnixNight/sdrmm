import { describe, expect, it } from "vitest";
import type { PrimexFrame } from "../lib/types";
import { eventDetail } from "./decoderDetail";
import { eventStation, eventSummary } from "./eventFacts";
import { primexBytes, primexReadings, primexSummary, primexTime } from "./primex";

const frame: PrimexFrame = {
  message_type: 129,
  length: 10,
  bytes: [129, 10, 1, 5, 28, 5, 5, 17, 5, 205],
  checksum_ok: true,
  time: { year: 2017, month: 5, day: 5, hour: 1, minute: 5, second: 28, weekday: 5 },
};

describe("Primex", () => {
  it("shows transmitter-local time without inventing a timezone", () => {
    expect(primexTime(frame.time)).toBe("2017-05-05 01:05:28");
    expect(primexSummary(frame)).toBe("Primex · 2017-05-05 01:05:28");
    const event = { kind: "primex", data: frame } as const;
    expect(eventSummary(event)).toBe(primexSummary(frame));
    expect(eventStation(event)).toBe("Primex");
    expect(eventDetail(event).fields).toContainEqual(["Local time", "2017-05-05 01:05:28"]);
  });

  it("shows control zero and only received bytes", () => {
    const control: PrimexFrame = {
      message_type: 131,
      length: 7,
      bytes: [131, 7, 0, 0, 0, 0, 138, 0, 0, 0],
      checksum_ok: true,
      message_number: 0,
    };
    expect(primexSummary(control)).toBe("Primex control 0");
    expect(primexBytes(control)).toBe("83 07 00 00 00 00 8A");
    expect(eventDetail({ kind: "primex", data: control }).fields).toContainEqual(["Control", "0"]);
  });

  it("surfaces checksum and framing errors", () => {
    expect(primexSummary({ ...frame, checksum_ok: false, error: "checksum" })).toBe(
      "Primex 129 · Checksum failed",
    );
    expect(primexSummary({ ...frame, checksum_ok: false, error: "truncated" })).toBe(
      "Primex 129 · Truncated",
    );
    expect(primexSummary({ ...frame, checksum_ok: false })).toBe("Primex 129 · Checksum failed");
  });

  it("keeps the last valid time visible during control messages and errors", () => {
    const control: PrimexFrame = {
      ...frame,
      time: undefined,
      message_type: 131,
      message_number: 0,
    };
    const failed: PrimexFrame = { ...frame, checksum_ok: false, error: "checksum" };
    expect(primexReadings([failed, control, frame])).toEqual({
      time: "2017-05-05 01:05:28",
      control: 0,
      station: undefined,
    });
    expect(primexReadings([failed])).toEqual({
      time: undefined,
      control: undefined,
      station: undefined,
    });
  });

  it("shows and retains the Morse station ID on Primex messages", () => {
    const identified: PrimexFrame = {
      ...frame,
      station_id: [86, 70, 75, 55, 55, 55, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    };
    const event = { kind: "primex", data: identified } as const;
    expect(eventStation(event)).toBe("VFK777");
    expect(eventDetail(event).fields).toContainEqual(["Station ID", "VFK777"]);
    expect(primexReadings([frame, identified]).station).toBe("VFK777");
  });
});
