# Garden source recovery, 2026-09-27

The committed garden Chipyard history through `041d19ea` is merged into this tree. Its KU040 dual and FP16 configs require the accompanying uncommitted KU040 harness binders and untracked FcRoCC Scala/SV/ROM resources; these are imported into their compiled paths, with SHA256 provenance in `imported-files.json`. No source checkout was modified.

The patches preserve additional uncommitted tracked changes for review; they are not automatically applied. In particular, the KU040 L2 experiment depends on separate inclusive-cache/Pynq work, and the garden FPGA-shell patch changes the physical Artix speed grade from -1 to -2 despite the documented -1 module. The active tree retains the published -1 shell and current TE0712 flash-boot fixes. Ancillary PDM microphone and research experiments are outside this recovered deployment baseline.

Prior garden performance reports are provenance, not validation of this merged RTL. Fresh elaboration, timing and hardware acceptance remain required.
