# Camera profiles

## Storage

Named camera profiles are saved on this box, in one TOML file under `$HOME` so a deploy never clobbers venue tuning. `DEFAULT` is built in, always listed first, and means "what the camera opened with"; it is never written to the file. Nothing records which profile was active: the app starts on `DEFAULT` every time.

## Units

Image controls a camera can change while it streams are in the ZED SDK's own units, so a profile goes to `setCameraSettings` unconverted. The dashboard converts for display.

## Auto exposure

`auto_exposure` on means the SDK's AEC/AGC owns exposure and both gains, and the three values below it are whatever it last chose. Off means the applied values are the given ones.

`anti_banding` is auto-exposure flicker rejection: `OFF`, `AUTO`, `HZ_50`, `HZ_60`. `ZedOneRgbCamera` casts the enum to the SDK's `AE_ANTIBANDING` value (0 to 3 in that order), so reordering the enum sends the wrong flicker mode. `AntiBandingTest.ValuesMatchTheZedSdkAeAntibandingModes` pins it.

## Save semantics

The Runner keeps what the dashboard last asked the camera for (nullopt while on `DEFAULT`). SAVE stores that request rather than the read-back, which can trail a request by a frame.
