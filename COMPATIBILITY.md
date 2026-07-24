# M1 Firmware ↔ ESP32 Firmware Compatibility

The M1 has **two** processors that must run **matching** firmware:

- **M1 (STM32H573)** — the main firmware (the UI, radios drivers, etc.)
- **ESP32-C6** — a co-processor that handles WiFi / Bluetooth / 802.15.4

They talk over an internal SPI link. **Each M1 firmware expects one specific
"kind" of ESP32 firmware.** If they don't match, WiFi/BLE simply won't work
(the ESP shows as offline). Nothing is damaged — you just need to flash the
ESP32 firmware that matches the M1 firmware you're running.

## The two ESP32 firmware types in use

| Type | What it is | Used by |
|------|------------|---------|
| **ESP-Hosted** (`network_adapter` / `StealthHybrid`) | ESP acts as a network co-processor; the M1 runs its own IP stack | **Genuine stock Monstatek** firmware |
| **Brain** (`m1-esp32-brain`) | ESP runs the radio features natively over a custom binary link | **C3** firmware |

> An older **ESP-AT** experiment (`esp32-at-monstatek-m1`) has been **retired** —
> C3 uses Brain instead. Don't flash ESP-AT builds; they aren't compatible with
> current C3 or with genuine stock.

## Compatibility matrix

| M1 firmware | Required ESP32 firmware | ESP type | Where to get the ESP image |
|-------------|-------------------------|----------|-----------------------------|
| **Stock v0.8.0.0** | `network_adapter` | ESP-Hosted | Monstatek (factory) |
| **Stock v0.8.0.2** | `network_adapter` | ESP-Hosted | Monstatek (factory) |
| **Stock v0.8.0.4** | `StealthHybrid` | ESP-Hosted | Monstatek (factory) |
| **C3 v0.8.0.0-C3.107 and later** | `m1-esp32-brain` **v1.0.0+** | Brain | [m1-esp32-brain releases](https://github.com/bedge117/m1-esp32-brain/releases) |

## Rules of thumb

- **Running C3?** Flash the latest **brain** (`Download latest (SPI brain)` in
  qMonstatek). This is what you want 99% of the time.
- **Returning to genuine stock?** Use **Factory Restore** in qMonstatek, which
  flashes the matching stock ESP image for the stock M1 version you pick — do
  **not** use our ESP-AT build for this; it is not stock-compatible.
- **ESP shows offline after switching M1 firmware?** That's a mismatch. Flash
  the ESP firmware that matches your current M1 firmware (per the table above).

## Notes

- The genuine stock ESP firmware is Espressif **ESP-Hosted** (`network_adapter`),
  customized by Monstatek — a different protocol from both ESP-AT and Brain. The
  retired ESP-AT build was never compatible with it, which is one reason C3 moved
  to the Brain architecture.
- Monstatek changed the stock ESP firmware between **v0.8.0.2**
  (`network_adapter`) and **v0.8.0.4** (`StealthHybrid`), so the correct stock
  ESP image depends on the stock M1 version.
- ESP factory images flash at **0x0**. All our published factory images are
  trimmed (not padded to 4 MB) so they flash reliably over the M1's SPI bridge.

_Last updated: 2026-07-24_
