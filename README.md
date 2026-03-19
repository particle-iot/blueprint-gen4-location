# Gen 4 Location Tracking

### Description

Track the location of Particle Gen 4 devices using a combination of GNSS, Wi-Fi, and cellular tower geolocation with automatic fallback when satellite visibility is poor.

**Difficulty:** Intermediate
**Estimated Time:** 30 minutes
**Hardware Needed:** Particle Gen 4 device (M404, M524, or B504e) + Muon carrier board + USB cable
**Code Link:** https://github.com/particle-iot/blueprint-gen4-location

---

### Overview

Learn how to implement robust location tracking on Particle Gen 4 devices using a non-blocking state machine that orchestrates GNSS acquisition and automatically falls back to Wi-Fi and cellular tower fusion via Particle's cloud-enhanced location services.

---

### Tools & Materials

- Particle Gen 4 device: M404 (tested), M524, or B504e (tested)
- Muon carrier board or compatible Gen 4 carrier
- USB cable
- Particle Workbench (VS Code) or Particle CLI
- Active Particle account with a claimed device

---

### Prerequisites

- Particle account and a claimed Gen 4 device
- Device OS 6.3.0 or later
- Familiarity with flashing firmware via Workbench or CLI
- Basic C++ knowledge

---

### Dependencies

Dependencies are declared in [project.properties](project.properties) and installed automatically during compilation:

- [`LocationFusionRK`](https://github.com/rickkas7/LocationFusionRK) v0.0.4, orchestrates multi-source location fusion (GNSS, Wi-Fi, cell towers)
- [`QuectelGnssRK`](https://github.com/rickkas7/QuectelGnssRK) v0.0.1, interfaces with the Quectel modem's built-in GNSS receiver

> **Note:** LocationFusionRK v0.0.4 or later is required. Earlier versions do not support the B504e device.

---

### Steps

1. **Open the project** in Particle Workbench (VS Code) by opening this folder.
2. **Select your target platform:** Open the Command Palette (`Ctrl+Shift+P`) → `Particle: Configure Project for Device` → choose `msom` and your Device OS version (6.3.0+).
3. **Flash the firmware** using `Particle: Cloud Flash` or `Particle: Local Flash` from the Command Palette.
   - CLI alternative: `particle compile msom --target 6.3.5 && particle flash <device-name> firmware.bin`
4. **Open the Serial Monitor** to watch device operation:
   ```bash
   particle serial monitor --follow
   ```
5. **Verify the cycle**, you should see the device connect to cloud, acquire a location, publish a `loc` event, and receive a `loc-enhanced` response.

---

### How It Works

The firmware uses a 7-state finite state machine that monitors the location lifecycle without blocking the main loop:

```
WAITING_FOR_CLOUD
    ↓ (cloud connects)
WAITING_FIRST_PUBLISH
    ↓ (first publish starts)
LOCATION_BUILDING      ← Collecting GNSS / Wi-Fi / cell data
    ↓ (publish succeeds)
LOCATION_PUBLISHING
    ↓ (waiting for enhanced response)
LOCATION_WAITING
    ↓ (enhanced received)
IDLE ←→ LOCATION_BUILDING  (repeats every 5 minutes)
```

| State | Description |
|-------|-------------|
| `WAITING_FOR_CLOUD` | Initial state after boot, waiting for Particle Cloud connection |
| `WAITING_FIRST_PUBLISH` | Cloud connected, waiting for the first location publish to start |
| `IDLE` | Normal operation between location publishes |
| `LOCATION_BUILDING` | Actively collecting location data (GNSS / Wi-Fi / cell) |
| `LOCATION_PUBLISHING` | Location data published, waiting for send confirmation |
| `LOCATION_WAITING` | Waiting for cloud-enhanced location response |
| `ERROR_RECOVERY` | Handling publish failures with a 60-second retry delay |

**Non-Blocking Architecture:** The state machine runs once per second using `millis()`-based timing. Between ticks, `loop()` returns immediately so system threads and background tasks continue uninterrupted. `LocationFusionRK` performs all GNSS and location fusion work in background threads, meaning the state machine only monitors status rather than waiting on I/O.

Additional code can be added to `loop()` without touching any location logic:

```cpp
void loop() {
    updateStateMachine();   // Location monitoring

    // Add your features here without touching location logic
}
```

**Cloud-First Strategy:** The device connects to Particle Cloud *before* starting any location acquisition. This is required because Wi-Fi and cell tower fusion relies on active cloud connectivity, and without it the device can only use raw GNSS.

**Location Priority & Fallback:**
1. **GNSS** (90-second timeout), most accurate, requires clear sky view
2. **Location Fusion**, if GNSS times out, `LocationFusionRK` sends Wi-Fi access point and cell tower data to Particle Cloud, which returns enhanced coordinates

This strategy ensures a location fix is always obtained, even in challenging environments:
- **Indoor locations**, no GNSS signal, but Wi-Fi access points provide positioning
- **Urban canyons**, weak GNSS from building obstruction, but dense cellular coverage compensates
- **Remote areas**, strong GNSS with clear sky, limited Wi-Fi or cellular fallback needed

> **M404 (BG95 modem) limitation:** GNSS and cellular data cannot run simultaneously on this modem. The library handles this automatically, but brief connectivity interruptions may occur during GNSS acquisition. The M524 does not have this limitation.

---

### Usage

**Particle Console:**
1. Log in at [console.particle.io](https://console.particle.io)
2. Navigate to your device → **Events** tab
3. Look for `loc` events (published by the device) and `loc-enhanced` events (returned by Particle Cloud with refined coordinates)
4. Use the **Location** tab for a map view of historical device position (requires opting into location storage)

**Expected serial output:**
```
State: WAITING_FOR_CLOUD -> WAITING_FIRST_PUBLISH
State: WAITING_FIRST_PUBLISH -> LOCATION_BUILDING
State: LOCATION_BUILDING -> LOCATION_PUBLISHING
Enhanced Position: lat=XX.XXXXXX, lon=-XXX.XXXXXX, accuracy=XX.Xm
State: LOCATION_WAITING -> IDLE
```

---

### Topics Covered

- [`LocationFusionRK`](https://github.com/rickkas7/LocationFusionRK), multi-source location orchestration
- [`QuectelGnssRK`](https://github.com/rickkas7/QuectelGnssRK), Quectel modem GNSS interface
- Non-blocking state machine pattern
- `Particle.publish()` and `loc-enhanced` cloud events
- Wi-Fi + GNSS + cellular tower location fusion

---

### Extensions

- **Adaptive publishing:** Increase publish frequency when motion is detected using an accelerometer
- **Battery-aware operation:** Reduce publish frequency when battery charge is low (`System.batteryCharge()`)
- **Geofencing:** Check coordinates in `locEnhancedCallback` and publish alerts when entering or leaving an area
- **Sleep management:** Add a second state machine to enter sleep mode when the device is stationary
- **Custom sensor data:** Chain additional handlers with `.withAddToEventHandler()` to include temperature, humidity, or other sensor readings in every `loc` event
