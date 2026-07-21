# IR Database — Sources & Attribution

## Overview

This directory contains infrared (IR) remote control files in Flipper Zero
`.ir` format.  Copy files to `IR/` on the M1's SD card to use with the
Universal Remote feature.

## Sources

### M1 Original (pre-existing)

The following **116 files** were created or curated for the M1 firmware and
have been tested on hardware.  They live at the **top level** of each
category directory (e.g. `TV/Samsung.ir`, `Audio/Bose.ir`).

| Category  | Files | Notes |
|-----------|------:|-------|
| TV        |    18 | Samsung, LG, Sony, Philips, Panasonic, Vizio, TCL, etc. |
| AC        |    80 | Daikin, LG, Mitsubishi, Samsung, Fujitsu, Gree, etc. |
| Audio     |    16 | Bose, Denon, Yamaha, Onkyo, Pioneer, Marantz, etc. |
| Fan       |     1 | Universal fan remote |
| Projector |     1 | Universal projector remote |
| **Total** | **116** | |

### Flipper-IRDB (community database)

The remaining **1,296 files** were imported from the
[Flipper-IRDB](https://github.com/Lucaslhm/Flipper-IRDB) community
database.  They live in **brand subdirectories** under each category
(e.g. `TV/Samsung/Samsung_BN59-01358D.ir`).

- **Repository**: <https://github.com/Lucaslhm/Flipper-IRDB>
- **Commit SHA**: `d126fb1b6f1e114c52b4a8c19839ea65e3a9c24d`
- **Import date**: 2026-06-09
- **License**: **CC0 1.0 Universal** (public domain)

| Category           | IRDB Source Directory          | Files |
|--------------------|--------------------------------|------:|
| TV/                | TVs/                           |   398 |
| AC/                | ACs/                           |   158 |
| Audio/Receivers/   | Audio_and_Video_Receivers/     |   103 |
| Audio/SoundBars/   | SoundBars/                     |   106 |
| Audio/Speakers/    | Speakers/                      |    69 |
| Fan/               | Fans/                          |   155 |
| Projector/         | Projectors/                    |   126 |
| LED/               | LED_Lighting/                  |   168 |
| Streaming/         | Streaming_Devices/             |    25 |
| **Total**          |                                | **1,308** |

IRDB categories **not imported** (niche or low demand): Air_Purifiers,
Bidet, Blu-Ray, Cable_Boxes, Cameras, Car_Multimedia, CCTV, CD_Players,
Clocks, Computers, Consoles, Converters, DVB-T, DVD_Players, Digital_Signs,
Dust_Collectors, Fireplaces, Handicap_Ceiling_Lifts, Head_Units, Heaters,
Humidifiers, KVM, Laserdisc, MiniDisc, Miscellaneous, Monitors, Multimedia,
Picture_Frames, Touchscreen_Displays, Toys, TV_Tuner, Universal_TV_Remotes,
VCR, Vacuum_Cleaners, Videoconferencing, Whiteboards, Window_cleaners,
_Converted_.

## Updating

To refresh from Flipper-IRDB:

```bash
git clone --depth=1 https://github.com/Lucaslhm/Flipper-IRDB.git /tmp/irdb
# Compare brands/models, copy new files into the appropriate category
# Update the commit SHA and date above
```

## License

- **M1 original files**: Part of the M1 Hapax project (GPLv3).
- **Flipper-IRDB files**: CC0 1.0 Universal — no restrictions.
