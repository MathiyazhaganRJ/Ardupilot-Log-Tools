# Sharkee ArduPilot Flight Log Viewer and 3D Digital Twin

A high-performance, web-based aeronautical telemetry analysis suite and 3D flight visualizer for ArduPilot DataFlash binary logs (`.bin`).

---

## Overview

The Sharkee Flight Log Viewer provides synchronized, multi-perspective aerodynamic analysis for fixed-wing and VTOL aircraft. It combines binary log parsing, a Three.js 3D trajectory and digital twin visualizer, interactive 2D geospatial satellite mapping, and authentic Garmin-style glass cockpit avionics instrumentation.

---

## Key Capabilities

### 1. High-Speed DataFlash Binary Parser
- Direct extraction of ArduPilot telemetry messages via `pymavlink` (`ATT`, `GPS`, `POS`, `BARO`, `BAT`, `ARSP`, `XKF1`, `XKF2`, `CTUN`, `RCOU`, `MODE`, `MSG`).
- Automatic flight mode segmentation (MANUAL, FBWA, AUTO, CRUISE, RTL, LOITER) and airspeed test step detection.
- Fast client-server architecture with in-memory telemetry caching for instant scrubbing across multi-hour logs.

### 2. 3D Aerodynamic Flight Visualizer (Three.js)
- Ribbon corridor visualization showing true 3D spatial trajectory, altitude profile, and orientation.
- 6-DOF aircraft attitude replay with roll, pitch, and yaw articulation.
- GLTF/GLB 3D airframe support (`Sharkee.glb`, `Endurance.glb`).
- Multiple viewpoint modes: Orbit Camera, Chase Cam, Follow Cam, Cockpit View, and Overhead Orthographic.

### 3. Interactive Geospatial Satellite Map (Leaflet)
- Real-time GPS coordinate synchronization with high-resolution imagery (ESRI World Imagery, Carto Dark, and local airfield tile cache).
- Ground track breadcrumbs, waypoint corridors, and home elevation anchoring.

### 4. Garmin GI 275 EIS (Engine Information System)
- Authentic Garmin GI 275 panel styling for electric propulsion and battery telemetry:
  - Dual Mode Switcher: **[P]** Battery Power Monitor and **[E]** Engine RPM / ESC Telemetry.
  - Power Load Percentage, Motor RPM arc, ESC and Battery Temperatures.
  - Bus Voltage, Current Draw (Amps), Electrical Power (Watts), Total Consumed mAh.
  - Aerodynamic Transport Efficiency (`mAh/km` and `Wh/km`).
  - True Flight Time calculated from pitot dynamic pressure (`Airspeed Alive`).
  - Battery pack configuration (cell count, average cell voltage, remaining capacity, estimated endurance).

### 5. Atmosphere and Environmental Telemetry Panel
- GI 275 / G3X styling with black high-contrast micro-capsule displays:
  - **ELEV / AMSL / AGL Triad**: Airfield Field Elevation MSL, Current Absolute Altitude (AMSL), and Above Ground Level height (AGL).
  - **QNH Altimeter**: Barometric sea-level standard setting with flight MIN / MAX / AVG statistics.
  - **Flight Controller Barometric Temperature**: Internal sensor readings with MIN / MAX / AVG tracking.
  - **Pitot OAT Airspeed Temperature**: Outside air temperature at the pitot tube.
  - **Station Static Pressure**: Ambient pressure in hPa.
  - **Air Density ($\rho$)**: Instantaneous atmospheric density calculated via ideal gas law $P / (R \cdot T)$ in $\text{kg/m}^3$.
  - **Wind Vector Telemetry**: Wind speed, meteorological direction, and dynamic Headwind / Crosswind (`HW / XW`) resolution relative to current aircraft heading.

---

## Directory Structure

```
Ardupilot-Log-Tools/
├── index.html              # Main flight log analysis workspace and timeline scrubber
├── sharkee_hud.html        # 3D digital twin, PFD HUD, EIS, and environmental telemetry
├── server.py               # Standalone multi-threaded Python HTTP and API server
├── parser.py               # ArduPilot binary log parser (pymavlink)
├── requirements.txt        # Python package dependencies
├── Launch_Log_Viewer.bat   # Windows launcher batch script
├── .gitignore              # Git ignore rules
├── app_icon.png            # Application iconography
├── favicon.png             # Browser favicon
├── libs/                   # Bundled JavaScript and CSS libraries
│   ├── three.min.js        # Three.js 3D graphics engine
│   ├── GLTFLoader.js       # Three.js GLTF/GLB loader
│   ├── OrbitControls.js    # Three.js camera controls
│   ├── leaflet.js          # Leaflet 2D mapping engine
│   ├── leaflet.css         # Leaflet style definition
│   └── plotly.min.js       # High-performance scientific telemetry plotting
└── models/                 # 3D airframe models
    ├── Sharkee.glb         # Sharkee airframe model
    └── Endurance.glb       # Alternate airframe model
```

---

## Installation

### Prerequisites
- Python 3.9 or newer
- Web browser (Google Chrome, Microsoft Edge, or Mozilla Firefox)

### Setup
1. Clone the repository:
   ```bash
   git clone https://github.com/MathiyazhaganRJ/Ardupilot-Log-Tools.git
   cd Ardupilot-Log-Tools
   ```

2. Install required Python packages:
   ```bash
   pip install -r requirements.txt
   ```

---

## Usage

### Starting the Server
Run the launcher script or start the server via command line:

- **Windows Batch File**: Double-click `Launch_Log_Viewer.bat`
- **Command Line**:
  ```bash
  python server.py
  ```

The server automatically starts at `http://localhost:8085` and opens the viewer in your default browser.

### Loading Flight Logs
1. Place ArduPilot `.bin` log files in `~/Desktop/Logs` or configure the directory via the `ARDUPILOT_LOGS_DIR` environment variable.
2. Select a flight log from the left sidebar or click the log browser to inspect metadata, duration, autopilot board, and firmware version.
3. Use the timeline slider, playback controls, or keyboard shortcuts to scrub through the mission.

