# Upstream Notes

This repository is informed by `agilexrobotics/questVR_ws`, especially the Quest APK, ADB/logcat reader workflow, and single/dual Piper architecture.

`questVR_ws` is treated as an architecture reference unless its license is clarified. No unlicensed text or code from it should be copied verbatim into this repository.

Reusable licensed upstreams:

- `agilexrobotics/piper_sdk`: MIT license, used through the installed `piper-sdk` package.
- `rail-berkeley/oculus_reader`: Apache-2.0 license, used as the primary Quest reader API.

The current code reimplements the host-side transport/session/control structure in English and keeps the first-working Piper endpoint-control path explicit.
