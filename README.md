# GroundLink

> **Serious engineering, playful presentation.**

A lightweight web-based ground control station built in **Rust** for communicating with ArduPilot vehicles over **MAVLink**.

Or, in normal-person language:

**Drone sends numbers → Rust does Rust things → pretty dashboard appears.**

GroundLink is still a **work in progress**. Right now I'm building and testing the software with its own native demo mode before moving onto real ArduPilot hardware.

Crashing imaginary drones is considerably cheaper.

---
<h2>Demo</h2>

<p align="center">
  <img
    src="https://github.com/user-attachments/assets/e9f65c13-336f-4e1d-99ff-ba67f7f975a2"
    width="850"
    alt="GroundLink application demo"
  />
</p>

<p align="center">
  <img
    src="https://github.com/user-attachments/assets/cfdeb6f5-07dd-495d-ab00-6f39341a5b56"
    width="850"
    alt="GroundLink waypoint demo"
  />
</p>

<p align="center">
  <i>GroundLink running in native demo mode — no drone was harmed in the making of these screenshots.</i>
</p>

## What does it do?

GroundLink takes vehicle telemetry and turns it into a simple browser-based ground station.

Currently it handles:

- Live telemetry
- Vehicle position and navigation
- Flight information
- Real-time WebSocket updates
- Basic waypoint missions
- Built-in simulated vehicle testing

The eventual goal is to connect the same application to a real ArduPilot flight controller through MAVLink.

---

## How does it work?

The backend is written in **Rust**.

For real vehicles, MAVLink messages are handled using `rust-mavlink`. **Axum** handles the web side of the application and WebSockets push live telemetry to the browser.

```text
ArduPilot
    ↓
  MAVLink
    ↓
   Rust
    ↓
   Axum
    ↓
 WebSocket
    ↓
 Browser
```

The frontend is deliberately simple:

- HTML
- CSS
- JavaScript

No React megaproject with 900 dependencies required. <img src="https://github.com/user-attachments/assets/425903db-d4a1-4433-9abe-166b85b9947d" width="45" alt="neco">

---

## Wait, where's the drone?

Currently?

**There isn't one.**

GroundLink has a native demo mode written directly into the Rust application.

Running:

```bash
cargo run -- --demo
```

switches GroundLink to its internal simulated telemetry source.

```text
Native Rust Demo
       ↓
   Telemetry
       ↓
Axum + WebSocket
       ↓
    Browser
```

The demo generates changing vehicle telemetry rather than requiring a flight controller just to see if the application works.

It also has separate mission demo behavior, allowing waypoint functionality to be developed without putting a real aircraft in the air.

Basically:

**Fake Drone™ until Real Drone™ is ready.**

---

## Waypoint missions

GroundLink is being built to support basic waypoint missions.

```text
HOME → WP 1 → WP 2 → WP 3 → LAND
```

In demo mode, the simulated vehicle can follow the loaded mission.

With real hardware, GroundLink sends the mission through MAVLink and **ArduPilot handles actually flying the aircraft**.

GroundLink says where to go.

ArduPilot handles the slightly more important problem of getting there without falling out of the sky.

Mission speed controls and more advanced mission functionality are still planned.

---

# Running GroundLink

## Requirements

You'll need:

- Git
- Rust / Cargo
- A modern browser

That's it for demo mode.

---

## 1. Clone it

```bash
git clone https://github.com/proto6699/GroundLink.git
cd GroundLink
```

## 2. Build it

```bash
cargo build
```

Cargo will download the required Rust dependencies.

## 3. Run the demo

```bash
cargo run -- --demo
```

GroundLink will start using its built-in simulated vehicle.

Open the local address shown in the terminal and you're good to go.

No drone required.

---

## Normal mode

Running:

```bash
cargo run
```

starts GroundLink in its normal vehicle mode rather than the internal demo.

This is the path intended for MAVLink/ArduPilot communication.

```text
Real Vehicle
     ↓
  ArduPilot
     ↓
   MAVLink
     ↓
 GroundLink
     ↓
   Browser
```

Real flight-controller and telemetry-radio testing is still part of the project's ongoing development.

---

## What's under the hood?

| | |
|---|---|
| Main language | Rust |
| Web backend | Axum |
| Vehicle protocol | MAVLink |
| MAVLink library | rust-mavlink |
| Browser communication | WebSockets |
| Frontend | HTML / CSS / JavaScript |
| Demo vehicle | Native Rust simulation |
| Target autopilot | ArduPilot |
| Build system | Cargo |

---

## How I built it

I built GroundLink in pieces rather than trying to make an entire ground station at once.

First came the telemetry model and vehicle data. Then the Rust backend, MAVLink communication, Axum/WebSockets, the browser dashboard, and finally mission functionality.

The built-in demo grew alongside it so I could test everything without depending on hardware.

```text
Telemetry
    ↓
Rust backend
    ↓
MAVLink support
    ↓
Axum + WebSockets
    ↓
Dashboard
    ↓
Waypoint missions
    ↓
Real hardware
    ↓
hopefully not a crater
```

---

## Current status

GroundLink is a **prototype under active development**.

- [x] Rust backend
- [x] Native demo vehicle
- [x] MAVLink communication
- [x] Telemetry handling
- [x] WebSocket telemetry
- [x] Browser interface
- [x] Basic mission architecture
- [x] Demo mission behavior
- [ ] Finish waypoint workflow
- [ ] Mission speed controls
- [ ] Real ArduPilot flight controller testing
- [ ] Telemetry radio testing
- [ ] Actual flight testing

Eventually, this:

```text
Native Demo → GroundLink → Browser
```

becomes this:

```text
Drone
  ↓
ArduPilot
  ↓
MAVLink
  ↓
Telemetry Link
  ↓
GroundLink
  ↓
Browser
```

Same GroundLink.

The fake drone just gets replaced by something significantly more capable of breaking a window.

---

## Why?

I wanted to understand what actually happens between:

> **"go to this waypoint"**

and

> **drone say oke

The goal isn't to replace QGroundControl.

It's to understand and build the chain myself:

```text
Human → UI → Rust → MAVLink → ArduPilot → Drone → physics :(
```

---

### Disclaimer

GroundLink is experimental software, not production flight-control software.

The built-in simulator is for development and demonstration. Real hardware testing comes later.

Don't trust my silly Rust program with your $2,000 drone just yet.

---

<p align="center">
  <b>GroundLink</b><br>
  Serious engineering, playful presentation.
</p>
