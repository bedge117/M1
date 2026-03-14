M1 IR Remote Database
=====================

Copy the contents of this directory to the SD card at 0:/IR/

The M1 will browse these directories and .ir files from:
  Infrared > Universal Remote

Directory Structure
-------------------
  IR/
  ├── TV/
  │   ├── Samsung.ir         Full Samsung TV remote
  │   ├── LG.ir              Full LG TV remote
  │   ├── Sony.ir            Full Sony TV remote
  │   ├── Philips.ir         Full Philips TV remote
  │   ├── Panasonic.ir       Full Panasonic TV remote
  │   ├── Vizio.ir           Vizio TV remote
  │   ├── TCL.ir             TCL TV remote
  │   ├── Hisense.ir         Hisense TV remote
  │   ├── Toshiba.ir         Toshiba TV remote
  │   ├── Sharp.ir           Sharp TV remote
  │   └── Universal_Power.ir Power codes for 40+ brands (brute force)
  ├── Audio/
  │   ├── Samsung_Soundbar.ir
  │   ├── Bose.ir
  │   ├── Denon_Receiver.ir
  │   └── Universal_Power.ir Power codes for 19 audio brands
  ├── Projector/
  │   └── Universal_Projector.ir  Power codes for 16 projector brands
  └── Fan/
      └── Universal_Fan.ir   Power/speed for common fan brands

File Format
-----------
All files use the Flipper Zero .ir format (compatible with Flipper IRDB).
You can add your own .ir files or download more from the Flipper IRDB
community database.

The M1 can also learn new remotes using the IR receiver.
Learned remotes are saved to 0:/IR/Learned/
