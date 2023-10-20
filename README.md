# Main_MiSTeX Main Binary Repo

This repo serves as the home for the MiSTeX Main binaries.

This binary constitudes the main Linux executable of the MiSTeX system,
which handles, the game controllers, the menu system and uploading
of ROMs, etc.

Currently only the raspberry Pi zero is supported.

To build:

1. clone the MiSTeX-buildroot repo in the parent folder of this one
2. build it
3. come into this directory
4. `make sdk`
5. `make -f Makefile.arm -j`

Then you have the MiSTer executable, which you need to copy
to /media/fat/ on the MiSTeX Linux system
