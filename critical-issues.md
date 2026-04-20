# Critical Issues: Flash Detector Firmware vs. AS3935 Datasheet

Date: 2026-04-20

Scope: Review of the current firmware in `src/main.ino` against the AS3935 datasheet and the project README.

Assessment: The current implementation does not match the AS3935 datasheet closely enough for safety-sensitive use. Several mismatches can materially affect false positives, false negatives, and the reliability of the displayed distance estimate. Because this device may influence decisions around lightning exposure, these issues should be treated as critical until corrected and validated on hardware.

## Critical Findings

### 1. Invalid startup calibration and probe sequence

Code:

- `src/main.ino:336` writes `0b00010010` to `REG0x00`
- `src/main.ino:339` writes register address `0x3A`
- `src/main.ino:344` writes register address `0x3B`
- `src/main.ino:349` requests one byte and treats any available response as "FOUND!"

Issue:

- The AS3935 datasheet defines direct-command registers `0x3C` and `0x3D`, not `0x3A` and `0x3B`.
- The datasheet states that direct commands are sent by writing `0x96` to those registers.
- `REG0x3D` is the `CALIB_RCO` command and should be used after POR because oscillator calibration is stored only in volatile memory.

Why this matters:

- The current code does not appear to send a valid calibration command.
- The current "device found" logic is not a meaningful probe of correct device state.
- If oscillator calibration is missing or invalid, timing-dependent detection and distance estimation can be degraded.

Datasheet basis:

- Direct command registers: `REG0x3C` and `REG0x3D`
- `CALIB_RCO`: write `0x96` to `REG0x3D`
- Calibration must be repeated after power-on reset because the result is stored in volatile memory

Risk:

- False negatives
- False positives
- Unreliable distance estimation

### 2. High-noise fault is hidden instead of surfaced

Code:

- `src/main.ino:379` handles `INT_NH`
- `src/main.ino:381` calls `raiseNoiseFloor()`
- `src/main.ino:100` to `src/main.ino:106` increases `noiseFloor`

Issue:

- The datasheet says `INT_NH` means the received noise exceeds the acceptable level and the AS3935 cannot operate properly under that condition.
- The firmware responds by repeatedly increasing the noise-floor threshold until the complaint stops.

Why this matters:

- This behavior suppresses a degraded-operating-state warning instead of treating it as a fault condition.
- The user can continue seeing apparently normal readings while the detector is no longer operating within acceptable input-noise conditions.

Datasheet basis:

- `INT_NH` indicates noise level too high
- The chip reports this when the input noise is above the configured acceptable threshold
- The datasheet explicitly states the AS3935 cannot operate properly under high-noise conditions

Risk:

- Hidden detection failure
- Reduced trustworthiness of all subsequent lightning reports
- User may rely on readings that should be considered invalid

### 3. Front-end is hard-coded to indoor mode for an outdoor lightning detector

Code:

- `src/main.ino:336` writes `0b00010010` to `REG0x00` with the comment `indoor AFE`

Issue:

- The project describes a handheld lightning detector intended for outdoor use.
- The AS3935 supports different AFE gain settings for indoor and outdoor operation.
- The current firmware unconditionally selects indoor mode.

Why this matters:

- The front-end mode changes the detector's sensitivity and disturber behavior.
- Using the wrong AFE configuration can worsen both missed detections and nuisance detections depending on the environment.

Datasheet basis:

- `REG0x00[5:1]` selects the AFE mode
- `10010` corresponds to indoor operation
- `01110` corresponds to outdoor operation

Risk:

- Mismatch between actual operating environment and configured detection behavior
- Reduced reliability of strike detection outdoors

## High Findings

### 4. Interrupt handling does not follow the documented timing model

Code:

- `src/main.ino:376` reads `REG0x03` continuously in the polling loop
- `src/main.ino:391` delays only for lightning handling and not for all interrupts

Issue:

- The datasheet says the AS3935 raises the IRQ pin when an event occurs and that the external unit should wait 2 ms before reading the interrupt register.
- The current firmware polls `REG0x03` every loop iteration instead of using the IRQ line.
- Reading `REG0x03` clears the interrupt condition.

Why this matters:

- Polling can race the device's event timing and classification.
- Clearing the interrupt register by polling makes behavior less deterministic and harder to validate.

Datasheet basis:

- The interrupt is signaled on the IRQ pin
- The interrupt register should be read after a 2 ms wait
- Reading the interrupt register clears the IRQ condition

Risk:

- Missed or misclassified events
- Hard-to-reproduce behavior during noise/disturber/lightning transitions

### 5. Register `0x02` is written without preserving documented control and reserved bits

Code:

- `src/main.ino:365` writes `writeReg(0x02, spikeRej);`
- `src/main.ino:79` later masks only the lower nibble in `maskWrite(0x02, 0x0F, spikeRej & 0x0F);`

Issue:

- The initial full-byte write clears all upper control bits in `REG0x02`.
- The datasheet defines upper bits in this register for minimum lightning count and statistics clearing, and also shows a reserved/default bit state that should not be casually overwritten.

Why this matters:

- Blind full-byte writes can put the part into undocumented or unintended states.
- This is especially problematic in a safety-sensitive detector where configuration traceability matters.

Datasheet basis:

- `REG0x02[5:4]` controls minimum number of lightning events in the last 15 minutes before a valid lightning interrupt is issued
- `REG0x02[6]` is used for clearing the lightning statistics via a high-low-high toggle
- Reserved/default bits should be preserved unless the datasheet explicitly authorizes modification

Risk:

- Unexpected detection behavior
- Loss of datasheet-defined defaults

### 6. Dynamic sensitivity changes are not communicated to the user

Code:

- `src/main.ino:82` to `src/main.ino:98` adjusts `watchdog` and `spikeRej`
- `src/main.ino:410` to `src/main.ino:413` relaxes tuning every 15 seconds
- `src/main.ino:224` displays `WD` and `SR`, but without any safety interpretation

Issue:

- The firmware changes rejection thresholds at runtime in response to detected disturbers and quiet periods.
- The datasheet states that larger watchdog and spike-rejection values increase robustness against disturbers at the cost of lower detection efficiency.

Why this matters:

- The detector's effective behavior is shifting over time.
- The UI does not explain that "tuning up" can reduce sensitivity to real events, especially more distant ones.

Datasheet basis:

- Higher `SREJ` increases disturber rejection but decreases detection efficiency
- Watchdog threshold also affects signal validation behavior

Risk:

- Operator may assume stable detection performance when the sensitivity envelope has changed

## Medium Findings

### 7. Display language overstates what the AS3935 distance register means

Code and docs:

- `README.md:8` says the device shows "strike distance"
- `src/main.ino:230` onward presents the distance as the main hero value for each event

Issue:

- The datasheet says the AS3935 estimates the distance to the head or leading edge of the storm, not the distance to the individual lightning discharge that triggered the interrupt.

Why this matters:

- The UI and README wording imply more precision than the sensor provides.
- Users may treat the displayed number as the exact distance to the last strike.

Datasheet basis:

- `REG0x07[5:0]` is the estimated distance to the head of the storm
- The estimate is statistical and can change as old events are purged

Risk:

- Misinterpretation of displayed risk distance

### 8. Invalid distance value `0x00` is coerced into "overhead"

Code:

- `src/main.ino:393` reads `REG0x07`
- `src/main.ino:394` forces `0x00` to `0x01`

Issue:

- The datasheet defines `0x01` as storm overhead and `0x3F` as out of range.
- It does not define `0x00` as overhead in the reviewed material.
- The firmware converts `0x00` into the most severe valid case.

Why this matters:

- An undocumented value should be treated as invalid or unknown, not silently remapped to a semantically strong state.

Risk:

- Incorrect user alarm state
- Loss of traceability when diagnosing sensor behavior

## Overall Conclusion

The current implementation should not be treated as trustworthy for safety-sensitive decisions around lightning exposure. The most serious issues are:

1. Invalid calibration and probing of the AS3935
2. Suppressing high-noise fault conditions by raising the threshold
3. Using indoor AFE mode in an outdoor handheld detector

These problems are sufficient on their own to justify corrective work before relying on the device. After code fixes, the device should be validated on hardware under controlled test conditions with the exact Grove AS3935 module, antenna setup, and intended enclosure before any field use.

## Sources

- Local firmware: `src/main.ino`
- Local project documentation: `README.md`
- AS3935 datasheet, ams / austriamicrosystems, revision 1.2:
  `https://www.digikey.com/htmldatasheets/production/1124289/0/0/1/as3935.html`
