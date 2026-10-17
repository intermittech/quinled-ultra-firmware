# The frozen record corpus

Every flash store's pages as a released firmware wrote them, so each later firmware can be checked against them: it
must read the same records back, and keep writing after them. One folder per version, never changed once frozen.

| Folder | Written by |
|---|---|
| `0.6.10/` | firmware 0.6.10, the release |
| `0.6.11/` | firmware 0.6.11, the release |

Each folder: `pages.hex` (every store's two pages, 32 bytes a line: `address hex`) and `manifest.json` (what that
firmware's own loaders read back from them, as at its next boot). The stores (the upper record layout,
`src/hal/board.h`): the settings and their extension, the calibration, the power-on scene log, the lifetime record,
the calibration history, the power-on effects (both banks). 0.6.10 and 0.6.11 wrote the same bytes.

**The check** (before a release: `tools/release/RELEASING.md`; `tools/size_gate.py` runs it too), on the new firmware's
measurement build (`MEASURE=1`: the stores out of line; the main board and the receiver, `BOARD=2`, which has no
calibration history):

```
python tools/record_corpus.py check <MEASURE=1 fw.elf> test/records/0.6.10 test/records/0.6.11
```

For each version: its pages into the emulator's flash (`tools/rv32emu.py`), the firmware's loaders must read what the
manifest says; then one more record per store, and a fresh boot reads the new ones. A format change that makes a
firmware leave an older store alone is listed in `tools/record_corpus.py` `IGNORED`, with the reason.

**Freezing a version** (at its release): build the release's source with `MEASURE=1` (the main board, the default
layout), then

```
python tools/record_corpus.py write <MEASURE=1 fw.elf> test/records/<version> <version> "<what built it>"
```

and add the folder to the table above. The writes are seeded, the same for every firmware, so two versions with the
same record formats write the same pages.
