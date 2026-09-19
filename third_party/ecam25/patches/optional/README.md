# Optional nvidia-oot patches

Nothing here is applied by `scripts/build_ecam25_driver.sh`. These are the hunks that modify
NVIDIA's own modules, which the JetPack 7 build deliberately avoids.

`vendor-oot-jp6.patch` is e-con's original, unported patch against L4T 36.4.0 nvidia-oot. It is
kept so the escalation path does not depend on the vendor tarball being at hand. Three of its
five areas are dead on JetPack 7 and two are conditional; the table in `../../README.md` says
which, and which check forces each one.

To use one: extract that single hunk, rebase it onto synced 39.2.1 sources, rebuild with
`./nvbuild.sh -m`, and add a ported patch file here next to the original recording which check
forced it.
