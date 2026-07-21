M1 IR Remote Database
=====================

Copy the contents of this directory to the SD card at 0:/IR/

The M1 will browse these directories and .ir files from:
  Infrared > Universal Remote

Directory Structure
-------------------
  IR/
  ├── TV/
  │   ├── Samsung.ir           M1 universal Samsung TV remote
  │   ├── LG.ir                M1 universal LG TV remote
  │   ├── Samsung/             Model-specific Samsung remotes (from IRDB)
  │   │   ├── Samsung_BN59-01358D.ir
  │   │   └── ...
  │   └── ...                  57 Samsung, 37 LG, 26 Sony, 21 Panasonic, etc.
  ├── AC/
  │   ├── Daikin.ir            M1 universal Daikin AC remote
  │   ├── Daikin/              Model-specific Daikin remotes (from IRDB)
  │   └── ...                  80 M1 originals + 158 IRDB brands
  ├── Audio/
  │   ├── Bose.ir              M1 universal Bose remote
  │   ├── Receivers/           Audio/video receivers (from IRDB)
  │   ├── SoundBars/           Soundbar remotes (from IRDB)
  │   └── Speakers/            Speaker remotes (from IRDB)
  ├── Fan/
  │   ├── Universal_Fan.ir     M1 universal fan remote
  │   └── Dyson/, Lasko/, ...  Brand-specific fans (from IRDB)
  ├── Projector/
  │   ├── Universal_Projector.ir  M1 universal projector remote
  │   └── Epson/, BenQ/, ...   Brand-specific projectors (from IRDB)
  ├── LED/                     LED lighting remotes (from IRDB)
  └── Streaming/               Streaming device remotes (from IRDB)

Total: 1,412 IR files (116 M1 originals + 1,296 from Flipper-IRDB)

Sources
-------
- M1 original files: top-level .ir files per category (tested on hardware)
- Flipper-IRDB files: brand subdirectories (CC0 public domain)
  See SOURCES.md for full attribution.

File Format
-----------
All files use the Flipper Zero .ir format (compatible with Flipper IRDB).
You can add your own .ir files or download more from the Flipper IRDB
community database.

The M1 can also learn new remotes using the IR receiver.
Learned remotes are saved to 0:/IR/Learned/
