use std::sync::LazyLock;

use num_complex::Complex;
use sdrmm_dsp::{DcBlocker, Decimator, FmDemod, ToneCorrelator, design_lowpass};
use sdrmm_modem::symbolcode::{ManchesterConvention, manchester_decode_pair};
use sdrmm_wire::{
    ChannelDescriptor, ChannelParams, ChannelSettings, DecoderEvent, DecoderFamily, PrimexError,
    PrimexFrame, PrimexParams, PrimexTime,
};

use crate::{
    AUDIO_RATE, ChannelCtx, ChannelError, ChannelFilter, ChannelOutputs, ChannelRx,
    check_input_rate, clamp_full_scale,
};

const RATE: f64 = 48_000.0;
const BLOCK: usize = 256;

static DESCRIPTOR: LazyLock<ChannelDescriptor> = LazyLock::new(|| ChannelDescriptor {
    type_id: "primex".to_owned(),
    name: "Primex time sync".to_owned(),
    summary: "Primex 72 MHz time and control messages".to_owned(),
    family: DecoderFamily::Paging,
    bandwidth_hz: 20_000.0,
    input_rate_hz: RATE,
    has_audio: true,
    decoder_kind: Some("primex".to_owned()),
    ..ChannelDescriptor::default()
});

fn params(settings: &ChannelSettings) -> Result<PrimexParams, ChannelError> {
    match settings.params {
        ChannelParams::Primex(p) => {
            check_params(&p)?;
            Ok(p)
        }
        ref other => Err(ChannelError::InvalidSettings(format!(
            "primex channel got {} params",
            other.type_id()
        ))),
    }
}

fn check_params(p: &PrimexParams) -> Result<(), ChannelError> {
    if !p.bandwidth_hz.is_finite() || !(10_000.0..=40_000.0).contains(&p.bandwidth_hz) {
        return Err(ChannelError::InvalidSettings(
            "Primex bandwidth must be 10000..=40000 Hz".to_owned(),
        ));
    }
    if !p.baud.is_finite() || !(900.0..=2_100.0).contains(&p.baud) {
        return Err(ChannelError::InvalidSettings(
            "Primex baud must be 900..=2100".to_owned(),
        ));
    }
    Ok(())
}

pub(crate) fn occupied_band(p: &PrimexParams) -> (f64, f64) {
    (-p.bandwidth_hz / 2.0, p.bandwidth_hz / 2.0)
}

pub(crate) fn channel_filter(p: &PrimexParams) -> Result<ChannelFilter, ChannelError> {
    check_params(p)?;
    Ok(ChannelFilter::Symmetric(Decimator::new(
        &design_lowpass(129, p.bandwidth_hz / (2.0 * RATE)),
        1,
    )))
}

pub struct PrimexChannel {
    demod: FmDemod,
    audio_dc: DcBlocker,
    audio: Vec<f32>,
    decoder: Decoder,
    station: StationId,
    baud: f64,
}

impl ChannelRx for PrimexChannel {
    fn descriptor() -> &'static ChannelDescriptor {
        &DESCRIPTOR
    }

    fn new(ctx: ChannelCtx, settings: ChannelSettings) -> Result<Self, ChannelError> {
        check_input_rate(ctx, &DESCRIPTOR)?;
        let p = params(&settings)?;
        Ok(Self {
            demod: FmDemod::new(RATE, 5_000.0),
            audio_dc: DcBlocker::new(),
            audio: Vec::with_capacity(BLOCK),
            decoder: Decoder::new(RATE, p.baud),
            station: StationId::new(),
            baud: p.baud,
        })
    }

    fn apply(&mut self, settings: ChannelSettings) -> Result<(), ChannelError> {
        let p = params(&settings)?;
        if p.baud != self.baud {
            self.baud = p.baud;
            self.retuned();
        }
        Ok(())
    }

    fn retuned(&mut self) {
        self.demod = FmDemod::new(RATE, 5_000.0);
        self.audio_dc = DcBlocker::new();
        self.decoder = Decoder::new(RATE, self.baud);
        self.station.reset();
        self.audio.clear();
    }

    fn process(&mut self, iq: &[Complex<f32>], out: &mut ChannelOutputs) {
        out.audio_pcm.clear();
        out.audio_rate = if iq.is_empty() { 0 } else { AUDIO_RATE };
        for block in iq.chunks(BLOCK) {
            let first_event = out.events.len();
            self.demod.process(block, &mut self.audio);
            for &sample in &self.audio {
                self.station.feed(sample);
                self.decoder.feed(sample, out);
            }
            for event in &mut out.events[first_event..] {
                if let DecoderEvent::Primex(frame) = event {
                    frame.station_id = self.station.decoded;
                }
            }
            self.audio_dc.process(&mut self.audio);
            clamp_full_scale(&mut self.audio);
            out.audio_pcm.extend_from_slice(&self.audio);
        }
    }
}

struct StationId {
    tone: ToneCorrelator,
    tick: usize,
    peak: f32,
    keyed: bool,
    run: u32,
    dot: f32,
    pattern: u16,
    text: [u8; 16],
    length: usize,
    decoded: Option<[u8; 16]>,
}

impl StationId {
    fn new() -> Self {
        Self {
            tone: ToneCorrelator::new(RATE, 1_024.0, 960),
            tick: 0,
            peak: 0.0,
            keyed: false,
            run: 0,
            dot: 12.0,
            pattern: 1,
            text: [0; 16],
            length: 0,
            decoded: None,
        }
    }

    fn reset(&mut self) {
        self.tone.reset();
        self.tick = 0;
        self.peak = 0.0;
        self.keyed = false;
        self.run = 0;
        self.dot = 12.0;
        self.clear();
        self.decoded = None;
    }

    fn clear(&mut self) {
        self.pattern = 1;
        self.text.fill(0);
        self.length = 0;
    }

    fn feed(&mut self, sample: f32) {
        let amplitude = self
            .tone
            .push(if sample.is_finite() { sample } else { 0.0 });
        self.tick += 1;
        if self.tick < 240 {
            return;
        }
        self.tick = 0;
        self.peak = amplitude.max(self.peak * 0.999);
        let threshold = (self.peak * if self.keyed { 0.5 } else { 0.65 }).max(0.18);
        let keyed = amplitude > threshold;
        if keyed == self.keyed {
            self.run = self.run.saturating_add(1);
            if !keyed && self.run as f32 >= self.dot * 2.0 {
                self.finish_letter();
            }
            if self.run as f32 > self.dot * 8.0 {
                self.clear();
            }
            return;
        }
        if self.keyed {
            self.finish_mark();
        } else if self.run as f32 > self.dot * 4.5 {
            self.clear();
        }
        self.keyed = keyed;
        self.run = 1;
    }

    fn finish_mark(&mut self) {
        let duration = self.run as f32;
        if !(self.dot * 0.5..=self.dot * 4.0).contains(&duration) || self.pattern >= 64 {
            self.clear();
            return;
        }
        let dash = duration >= self.dot * 2.0;
        let observed = duration / if dash { 3.0 } else { 1.0 };
        self.dot = (self.dot * 0.9 + observed * 0.1).clamp(6.0, 24.0);
        self.pattern = self.pattern * 2 + u16::from(dash);
    }

    fn finish_letter(&mut self) {
        if self.pattern == 1 {
            return;
        }
        let letter = crate::morse::decode_pattern(self.pattern)
            .filter(|text| text.len() == 1)
            .and_then(|text| text.as_bytes().first().copied())
            .filter(u8::is_ascii_alphanumeric);
        self.pattern = 1;
        let Some(letter) = letter else {
            self.clear();
            return;
        };
        if self.length == self.text.len() {
            self.clear();
            return;
        }
        self.text[self.length] = letter;
        self.length += 1;
        let prefix = self.text[..self.length]
            .iter()
            .take_while(|byte| byte.is_ascii_uppercase())
            .count();
        if (2..=3).contains(&prefix)
            && self.length >= prefix + 3
            && self.text[prefix..self.length]
                .iter()
                .all(u8::is_ascii_digit)
        {
            self.decoded = Some(self.text);
        }
    }
}

struct Decoder {
    half_samples: f32,
    relax: f32,
    low: f32,
    high: f32,
    level: bool,
    have_edge: bool,
    run: u32,
    pairs: [PairDecoder; 2],
}

impl Decoder {
    fn new(rate: f64, baud: f64) -> Self {
        Self {
            half_samples: (rate / (2.0 * baud)) as f32,
            relax: (1.0 / (rate * 0.2)) as f32,
            low: 0.0,
            high: 0.0,
            level: false,
            have_edge: false,
            run: 0,
            pairs: [PairDecoder::new(false), PairDecoder::new(true)],
        }
    }

    fn feed(&mut self, sample: f32, out: &mut ChannelOutputs) {
        if !sample.is_finite() {
            self.interrupt(PrimexError::Timing, out);
            self.low = 0.0;
            self.high = 0.0;
            self.have_edge = false;
            return;
        }
        self.low = sample.min(self.low + (sample - self.low) * self.relax);
        self.high = sample.max(self.high + (sample - self.high) * self.relax);
        let span = self.high - self.low;
        let center = (self.high + self.low) * 0.5;
        let hysteresis = span * 0.02;
        let next = if sample > center + hysteresis {
            true
        } else if sample < center - hysteresis {
            false
        } else {
            self.level
        };
        self.run = self.run.saturating_add(1);
        if next != self.level && span > 0.000_01 {
            if self.have_edge {
                self.finish_run(out);
            }
            self.have_edge = true;
            self.level = next;
            self.run = 0;
        } else if self.have_edge && self.run as f32 > self.half_samples * 2.5 {
            self.interrupt(PrimexError::Truncated, out);
            self.have_edge = false;
        }
    }

    fn finish_run(&mut self, out: &mut ChannelOutputs) {
        let chips = self.run as f32 / self.half_samples;
        let count = chips.round();
        if !(1.0..=2.0).contains(&count) || (chips - count).abs() > 0.4 {
            self.interrupt(PrimexError::Timing, out);
            return;
        }
        for _ in 0..count as u8 {
            for pair in &mut self.pairs {
                if let Some(frame) = pair.feed(self.level) {
                    out.events.push(DecoderEvent::Primex(frame));
                }
            }
        }
    }

    fn interrupt(&mut self, error: PrimexError, out: &mut ChannelOutputs) {
        for pair in &mut self.pairs {
            if let Some(frame) = pair.framer.abort(error) {
                out.events.push(DecoderEvent::Primex(frame));
            }
        }
        self.pairs = [PairDecoder::new(false), PairDecoder::new(true)];
    }
}

struct PairDecoder {
    skip: bool,
    first: Option<bool>,
    framer: Framer,
}

impl PairDecoder {
    fn new(skip: bool) -> Self {
        Self {
            skip,
            first: None,
            framer: Framer::default(),
        }
    }

    fn feed(&mut self, level: bool) -> Option<PrimexFrame> {
        if self.skip {
            self.skip = false;
            return None;
        }
        match self.first.take() {
            None => {
                self.first = Some(level);
                None
            }
            Some(first) => {
                match manchester_decode_pair(ManchesterConvention::GeThomas, first, level) {
                    Some(bit) => self.framer.feed(bit),
                    None => self.framer.abort(PrimexError::Manchester),
                }
            }
        }
    }
}

#[derive(Default)]
struct Framer {
    header: u16,
    seen: u8,
    bytes: [u8; 10],
    length: usize,
    received: usize,
    bits: u8,
    invert: bool,
}

impl Framer {
    fn feed(&mut self, bit: bool) -> Option<PrimexFrame> {
        if self.length == 0 {
            self.header = (self.header << 1) | u16::from(bit);
            self.seen = self.seen.saturating_add(1);
            if self.seen >= 16 {
                let (header, invert) = match self.header {
                    0x810a | 0x8307 => (self.header, false),
                    0x7ef5 | 0x7cf8 => (!self.header, true),
                    _ => return None,
                };
                self.bytes = [0; 10];
                self.bytes[0] = (header >> 8) as u8;
                self.bytes[1] = header as u8;
                self.length = usize::from(self.bytes[1]);
                self.received = 2;
                self.bits = 0;
                self.invert = invert;
            }
            return None;
        }
        self.bytes[self.received] = (self.bytes[self.received] << 1) | u8::from(bit ^ self.invert);
        self.bits += 1;
        if self.bits == 8 {
            self.bits = 0;
            self.received += 1;
            if self.received == self.length {
                let frame = decode(self.bytes, self.length);
                *self = Self::default();
                return Some(frame);
            }
        }
        None
    }

    fn abort(&mut self, error: PrimexError) -> Option<PrimexFrame> {
        let frame = (self.length > 0).then_some(PrimexFrame {
            station_id: None,
            message_type: self.bytes[0],
            length: self.received as u8,
            bytes: self.bytes,
            checksum_ok: false,
            time: None,
            message_number: None,
            error: Some(error),
        });
        *self = Self::default();
        frame
    }
}

fn decode(bytes: [u8; 10], length: usize) -> PrimexFrame {
    let checksum = bytes[..length - 1]
        .iter()
        .fold(0u8, |sum, byte| sum.wrapping_add(*byte));
    let checksum_ok = checksum == bytes[length - 1];
    let mut frame = PrimexFrame {
        station_id: None,
        message_type: bytes[0],
        length: length as u8,
        bytes,
        checksum_ok,
        time: None,
        message_number: None,
        error: (!checksum_ok).then_some(PrimexError::Checksum),
    };
    if checksum_ok {
        if bytes[0] == 129 {
            frame.time = decode_time(&bytes);
            if frame.time.is_none() {
                frame.error = Some(PrimexError::InvalidTime);
            }
        } else {
            frame.message_number = Some(bytes[2]);
        }
    }
    frame
}

fn decode_time(bytes: &[u8; 10]) -> Option<PrimexTime> {
    let time = PrimexTime {
        year: 2000 + u16::from(bytes[7]),
        month: bytes[5],
        day: bytes[6],
        hour: bytes[2],
        minute: bytes[3],
        second: bytes[4],
        weekday: bytes[8],
    };
    let leap = time.year.is_multiple_of(4)
        && (!time.year.is_multiple_of(100) || time.year.is_multiple_of(400));
    let days = [
        31,
        if leap { 29 } else { 28 },
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];
    let max_day = *days.get(usize::from(time.month.checked_sub(1)?))?;
    (bytes[7] <= 99
        && time.day > 0
        && time.day <= max_day
        && time.hour < 24
        && time.minute < 60
        && time.second < 60
        && time.weekday < 7)
        .then_some(time)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    #[global_allocator]
    static ALLOC: sdrmm_test_support::CountingAlloc = sdrmm_test_support::CountingAlloc::new();

    const TIME: [u8; 10] = [129, 10, 15, 59, 22, 1, 20, 18, 6, 24];
    const CAPTURE_TIME: [u8; 10] = [129, 10, 1, 5, 28, 5, 5, 17, 5, 205];

    fn bits(bytes: &[u8]) -> impl Iterator<Item = bool> + '_ {
        bytes
            .iter()
            .flat_map(|byte| (0..8).rev().map(move |shift| byte & (1 << shift) != 0))
    }

    fn corrected(mut bytes: [u8; 10]) -> [u8; 10] {
        bytes[9] = bytes[..9]
            .iter()
            .fold(0u8, |sum, byte| sum.wrapping_add(*byte));
        bytes
    }

    #[test]
    fn decodes_diagram_and_capture_as_binary_local_time() {
        assert_eq!(
            decode(TIME, 10).time.unwrap().to_string(),
            "2018-01-20 15:59:22"
        );
        assert_eq!(
            decode(CAPTURE_TIME, 10).time.unwrap().to_string(),
            "2017-05-05 01:05:28"
        );
        assert!(decode(CAPTURE_TIME, 10).checksum_ok);
    }

    #[test]
    fn decodes_all_four_control_messages_and_wraparound() {
        for number in [53u8, 54, 55, 56, 99, 0] {
            let bytes = [131, 7, number, 0, 0, 0, 138u8.wrapping_add(number), 0, 0, 0];
            let frame = decode(bytes, 7);
            assert_eq!(frame.message_number, Some(number));
            assert_eq!(frame.time, None);
            assert_eq!(frame.error, None);
        }
    }

    #[test]
    fn checksum_failure_preserves_raw_bytes_without_time() {
        let mut bytes = TIME;
        bytes[3] ^= 1;
        let frame = decode(bytes, 10);
        assert_eq!(frame.bytes, bytes);
        assert_eq!(frame.error, Some(PrimexError::Checksum));
        assert!(!frame.checksum_ok);
        assert!(frame.time.is_none());
    }

    #[test]
    fn rejects_invalid_dates_and_time_fields() {
        for (index, value) in [
            (2, 24),
            (3, 60),
            (4, 60),
            (5, 0),
            (5, 13),
            (6, 0),
            (7, 100),
            (8, 7),
        ] {
            let mut bytes = TIME;
            bytes[index] = value;
            assert_eq!(
                decode(corrected(bytes), 10).error,
                Some(PrimexError::InvalidTime)
            );
        }
        let mut bytes = TIME;
        bytes[5] = 2;
        bytes[6] = 29;
        assert_eq!(
            decode(corrected(bytes), 10).error,
            Some(PrimexError::InvalidTime)
        );
        bytes[7] = 24;
        assert_eq!(decode(corrected(bytes), 10).error, None);
    }

    #[test]
    fn hunts_headers_at_any_bit_offset_and_polarity() {
        for offset in 0..16 {
            for invert in [false, true] {
                let mut framer = Framer::default();
                let mut count = 0;
                for bit in (0..offset).map(|_| false).chain(bits(&TIME)) {
                    if let Some(frame) = framer.feed(bit ^ invert) {
                        count += 1;
                        assert_eq!(frame.bytes, TIME);
                        assert_eq!(frame.error, None);
                        assert_eq!(framer.length, 0);
                    }
                }
                assert_eq!(framer.received, 0);
                assert_eq!(count, 1);
            }
        }
    }

    #[test]
    fn interrupted_packet_surfaces_and_next_packet_recovers() {
        let mut framer = Framer::default();
        for bit in bits(&TIME[..4]) {
            assert!(framer.feed(bit).is_none());
        }
        let failed = framer.abort(PrimexError::Truncated).unwrap();
        assert_eq!(failed.length, 4);
        assert_eq!(failed.error, Some(PrimexError::Truncated));
        let frames: Vec<_> = bits(&TIME).filter_map(|bit| framer.feed(bit)).collect();
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].error, None);
    }

    fn waveform(bytes: &[u8], rate: f64, baud: f64, invert: bool) -> Vec<f32> {
        let levels: Vec<_> = std::iter::repeat_n(false, 32)
            .chain(bits(bytes))
            .chain(std::iter::repeat_n(false, 32))
            .flat_map(|bit| {
                let (a, b) = ManchesterConvention::GeThomas.halves(bit ^ invert);
                [a, b]
            })
            .collect();
        let half_samples = rate / (2.0 * baud);
        (0..(levels.len() as f64 * half_samples) as usize)
            .map(|i| {
                if levels[(i as f64 / half_samples) as usize] {
                    0.8
                } else {
                    -0.8
                }
            })
            .collect()
    }

    #[test]
    fn manchester_tracks_baud_error_both_alignments_and_polarities() {
        for baud in [1_000.0, 1_024.0, 1_040.0, 2_000.0] {
            for invert in [false, true] {
                for delay in [0, 3, 17] {
                    let audio = waveform(&TIME, RATE, baud, invert);
                    let mut decoder =
                        Decoder::new(RATE, if baud > 1_500.0 { 2_000.0 } else { 1_024.0 });
                    let mut out = ChannelOutputs::default();
                    for sample in std::iter::repeat_n(0.0, delay).chain(audio) {
                        decoder.feed(sample, &mut out);
                    }
                    assert_eq!(
                        out.events,
                        [DecoderEvent::Primex(decode(TIME, 10))],
                        "baud {baud}, invert {invert}, delay {delay}"
                    );
                }
            }
        }
    }

    #[test]
    fn fm_iq_decodes_across_arbitrary_block_boundaries() {
        let mut phase = 0.0;
        let iq: Vec<_> = waveform(&CAPTURE_TIME, RATE, 1_024.0, false)
            .into_iter()
            .map(|audio| {
                phase += TAU * (5_000.0 * f64::from(audio) + 1_200.0) / RATE;
                Complex::from_polar(1.0, phase as f32)
            })
            .collect();
        for block in [1, 7, 127, 1024] {
            let settings = ChannelSettings::default_for("primex").unwrap();
            let mut channel =
                PrimexChannel::new(ChannelCtx { input_rate: RATE }, settings).unwrap();
            let mut out = ChannelOutputs::default();
            let mut audio = Vec::new();
            for samples in iq.chunks(block) {
                channel.process(samples, &mut out);
                assert_eq!(out.audio_pcm.len(), samples.len());
                assert_eq!(out.audio_rate, AUDIO_RATE);
                audio.extend_from_slice(&out.audio_pcm);
            }
            assert_eq!(out.events, [DecoderEvent::Primex(decode(CAPTURE_TIME, 10))]);
            assert_eq!(audio.len(), iq.len());
            assert!(
                audio
                    .iter()
                    .all(|sample| sample.is_finite() && sample.abs() <= 1.0)
            );
            assert!(audio.iter().any(|sample| sample.abs() > 0.5));
            channel.retuned();
            out.reset();
            out.events.reserve(4);
            out.audio_pcm.reserve(iq.len());
            sdrmm_test_support::assert_no_alloc("Primex FM decode", || {
                channel.process(&iq, &mut out);
            });
            assert_eq!(out.events, [DecoderEvent::Primex(decode(CAPTURE_TIME, 10))]);
            assert_eq!(out.audio_pcm.len(), audio.len());
            for (index, (actual, expected)) in out.audio_pcm.iter().zip(&audio).enumerate() {
                assert!(
                    (actual - expected).abs() < 0.001,
                    "block {block}, sample {index}: {actual} != {expected}"
                );
            }
            channel.process(&[], &mut out);
            assert!(out.audio_pcm.is_empty());
            assert_eq!(out.audio_rate, 0);
        }
    }

    #[test]
    fn monitor_audio_removes_carrier_offset() {
        let iq = crate::testutil::complex_tone(1_200.0 / RATE, 4_096);
        let settings = ChannelSettings::default_for("primex").unwrap();
        let mut channel = PrimexChannel::new(ChannelCtx { input_rate: RATE }, settings).unwrap();
        let audio = crate::testutil::run_ragged(&mut channel, &iq);
        assert_eq!(audio.len(), iq.len());
        assert!(audio[2_048..].iter().all(|sample| sample.abs() < 0.001));
    }

    fn station_audio(wpm: f64) -> Vec<f32> {
        let dot = (RATE * 1.2 / wpm) as usize;
        let mut audio = vec![0.0; dot * 7];
        for pattern in ["...-", "..-.", "-.-", "--...", "--...", "--..."] {
            for mark in pattern.bytes() {
                for _ in 0..dot * if mark == b'-' { 3 } else { 1 } {
                    let phase = TAU * 1_024.0 * audio.len() as f64 / RATE;
                    audio.push(0.55 * phase.sin() as f32 + 0.2);
                }
                audio.extend(std::iter::repeat_n(0.2, dot));
            }
            audio.extend(std::iter::repeat_n(0.2, dot * 2));
        }
        audio.extend(std::iter::repeat_n(0.2, dot * 7));
        audio
    }

    #[test]
    fn decodes_station_id_without_allocating_and_resets_on_retune() {
        for wpm in [15.0, 20.0, 25.0] {
            let audio = station_audio(wpm);
            let mut station = StationId::new();
            sdrmm_test_support::assert_no_alloc("Primex Morse station ID", || {
                for sample in audio {
                    station.feed(sample);
                }
            });
            assert_eq!(
                station.decoded.as_ref().map(|bytes| &bytes[..6]),
                Some(&b"VFK777"[..]),
                "{wpm}"
            );
            station.reset();
            assert_eq!(station.decoded, None);
        }
    }

    #[test]
    fn packet_data_does_not_invent_a_station_id() {
        let mut station = StationId::new();
        for sample in waveform(&CAPTURE_TIME, RATE, 1_024.0, false)
            .into_iter()
            .cycle()
            .take(480_000)
        {
            station.feed(sample);
        }
        assert_eq!(station.decoded, None);
    }

    #[test]
    fn rejects_invalid_settings() {
        for p in [
            PrimexParams {
                baud: f64::NAN,
                ..PrimexParams::default()
            },
            PrimexParams {
                baud: 0.0,
                ..PrimexParams::default()
            },
            PrimexParams {
                bandwidth_hz: f64::INFINITY,
                ..PrimexParams::default()
            },
            PrimexParams {
                bandwidth_hz: RATE,
                ..PrimexParams::default()
            },
        ] {
            assert!(check_params(&p).is_err());
            assert!(channel_filter(&p).is_err());
        }
    }

    #[test]
    #[ignore = "requires SDRMM_PRIMEX_IQ pointing to the supplied cf32_le IQ recording"]
    fn supplied_iq_recording_decodes_station_id_and_preserves_messages() {
        use std::io::Read;

        let path = std::env::var_os("SDRMM_PRIMEX_IQ").expect("SDRMM_PRIMEX_IQ");
        let file = std::fs::File::open(path).unwrap();
        let mut remaining = file.metadata().unwrap().len() as usize;
        assert_eq!(remaining % 8, 0);
        let mut reader = std::io::BufReader::new(file);
        let mut bytes = vec![0; 65_536 * 8];
        let mut iq = Vec::with_capacity(65_536);
        let mut ddc = sdrmm_dsp::Ddc::new(2_400_000.0, RATE, -131_214.0).unwrap();
        let mut tuned = Vec::new();
        let mut filtered = Vec::new();
        let settings = ChannelSettings::default_for("primex").unwrap();
        let mut filter = channel_filter(&PrimexParams::default()).unwrap();
        let mut channel = PrimexChannel::new(ChannelCtx { input_rate: RATE }, settings).unwrap();
        let mut out = ChannelOutputs::default();
        let mut times = 0;
        let mut controls = 0;
        let mut errors = 0;
        let mut identified = 0;
        while remaining > 0 {
            let count = remaining.min(bytes.len());
            reader.read_exact(&mut bytes[..count]).unwrap();
            remaining -= count;
            iq.clear();
            for sample in bytes[..count].as_chunks::<8>().0 {
                iq.push(Complex::new(
                    f32::from_le_bytes(sample[..4].try_into().unwrap()),
                    f32::from_le_bytes(sample[4..].try_into().unwrap()),
                ));
            }
            ddc.process(&iq, &mut tuned);
            filter.process(&tuned, &mut filtered);
            out.reset();
            channel.process(&filtered, &mut out);
            for event in &out.events {
                if let DecoderEvent::Primex(frame) = event {
                    times += usize::from(frame.time.is_some());
                    controls += usize::from(frame.message_number.is_some());
                    errors += usize::from(frame.error.is_some());
                    if let Some(station_id) = frame.station_id {
                        assert_eq!(&station_id[..6], b"VFK777");
                        assert!(station_id[6..].iter().all(|byte| *byte == 0));
                        identified += 1;
                    }
                }
            }
        }
        println!(
            "{times} times, {controls} controls, {errors} errors, {identified} identified messages"
        );
        assert_eq!((times, controls, errors), (29, 113, 3));
        assert!(identified > 0, "station ID not decoded");
    }

    #[test]
    #[ignore = "requires SDRMM_PRIMEX_AF pointing to the supplied SDRSharp PCM recording"]
    fn supplied_af_recording_decodes_time_and_control() {
        let path = std::env::var_os("SDRMM_PRIMEX_AF").expect("SDRMM_PRIMEX_AF");
        let wav = std::fs::read(path).unwrap();
        assert_eq!(&wav[..4], b"RIFF");
        assert_eq!(&wav[8..12], b"WAVE");
        let mut offset = 12;
        let mut rate = 0;
        let mut channels = 0;
        let mut pcm = &[][..];
        while offset + 8 <= wav.len() {
            let size = u32::from_le_bytes(wav[offset + 4..offset + 8].try_into().unwrap()) as usize;
            let chunk = &wav[offset + 8..offset + 8 + size];
            match &wav[offset..offset + 4] {
                b"fmt " => {
                    assert_eq!(u16::from_le_bytes(chunk[..2].try_into().unwrap()), 1);
                    channels = u16::from_le_bytes(chunk[2..4].try_into().unwrap()) as usize;
                    rate = u32::from_le_bytes(chunk[4..8].try_into().unwrap());
                    assert_eq!(u16::from_le_bytes(chunk[14..16].try_into().unwrap()), 16);
                }
                b"data" => pcm = chunk,
                _ => {}
            }
            offset += 8 + size + size % 2;
        }
        let mut decoder = Decoder::new(f64::from(rate), 1_024.0);
        let mut out = ChannelOutputs::default();
        for sample in pcm.chunks_exact(2 * channels) {
            decoder.feed(
                f32::from(i16::from_le_bytes([sample[0], sample[1]])) / 32768.0,
                &mut out,
            );
        }
        let frames: Vec<_> = out
            .events
            .iter()
            .filter_map(|event| match event {
                DecoderEvent::Primex(frame) if frame.error.is_none() => Some(frame),
                _ => None,
            })
            .collect();
        let times: Vec<_> = frames.iter().filter_map(|frame| frame.time).collect();
        let controls = frames
            .iter()
            .filter(|frame| frame.message_number.is_some())
            .count();
        println!(
            "{} valid time messages, {controls} valid control messages, {} total events",
            times.len(),
            out.events.len()
        );
        for event in &out.events {
            if let DecoderEvent::Primex(frame) = event
                && frame.error.is_some()
            {
                println!("Rejected: {frame:?}");
            }
        }
        assert!(times.len() >= 30, "{times:?}");
        assert!(controls >= 120);
        assert!(times.contains(&decode(CAPTURE_TIME, 10).time.unwrap()));
        assert!(
            times
                .iter()
                .all(|time| time.year == 2017 && time.month == 5 && time.day == 5)
        );
    }
}
