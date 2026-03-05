# ButterFlight

![](img/thumb.png)

## Summary

**ButterFlight** is a Maya plugin (.mll) that lets animators and technical directors generate realistic butterfly flight animations using the force-based model proposed by Chen et al. The approach combines simplified aerodynamic lift/drag forces with a curl-noise vortex force to produce the erratic, noisy trajectories characteristic of real butterflies, including wing-abdomen interaction and dynamic body posture adjustments. Designed for VFX artists and TDs in film, games, and virtual environments, ButterFlight replaces tedious hand-keyed animation with a physically-inspired procedural simulation controlled through MEL-scripted UI panels. Typical uses include ambient butterfly swarms, hero shots along artist-specified paths, and wind interaction effects. The tool outputs keyframed skeletal animation on rigged butterfly meshes, exportable via Alembic or FBX.

**Team Members:** Cecilia Chen and Yiding Tian

**SigGraph Paper:** "A Practical Model for Realistic Butterfly Flight Simulation," Qiang Chen, Tingsong Lu, Yang Tong, Guoliang Luo, Xiaogang Jin, and Zhigang Deng, ACM Transactions on Graphics (TOG), 2022.



## Requirements

- Autodesk Maya 2026 (installed at `C:/Program Files/Autodesk/Maya2026`)
- Visual Studio 2022 with the **Desktop Development with C++** workload
- CMake 3.25 or later

## Build

```bat
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

The compiled plugin lands at `build/bin/Release/butterFlight.mll`.

## Usage

1. In Maya 2026, open the **Plug-in Manager** and load `butterFlight.mll`.
2. In the Script Editor, run:

   ```mel
   source "path/to/src/mel/butterFlight_ui.mel";
   ```

   The ButterFlight panel will open automatically.


