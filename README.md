# Intelligent Traffic Management System with Selective Ambulance Siren Recognition

An ESP32-based prototype that detects ambulance sirens using TinyML and automatically gives green-signal priority to the ambulance's road, reducing waiting time at junctions.

**Department of Electrical and Electronics Engineering, Shridevi Institute of Engineering and Technology, Tumakuru (VTU) | Project Phase-2**

## Problem
Traffic signals run on fixed timing and do not consider emergencies, so ambulances get delayed at junctions, which can cost critical time. This project detects an ambulance siren and controls the signals automatically.

## Objectives
- Detect ambulance sirens using a digital microphone and audio processing
- Automatically control traffic signals and give priority to the ambulance lane
- Reduce ambulance waiting time and improve emergency response

## How it works
1. The INMP441 digital microphone captures real-time sound from the surroundings.
2. The ESP32 processes the audio and identifies the ambulance siren pattern using TinyML (AI).
3. When a siren is detected, the controller overrides normal signal operation.
4. The system identifies whether the ambulance is on Road 1 or Road 2 and switches that road's signal to green.
5. A 16x2 LCD shows the live status (for example "Normal Mode" and "Ambulance Detected").
6. After the ambulance passes, normal traffic operation continues for other vehicles.

## Hardware components
| Component | Purpose |
|---|---|
| ESP32 microcontroller | Main controller: audio processing and signal control |
| INMP441 digital microphone | Captures siren sound in real time |
| 16x2 LCD display (I2C) | Shows system and signal status |
| Red / Yellow / Green LEDs | Simulate traffic signals |
| Relay module | Controls the signal LEDs |
| Resistors | Limit LED current |
| LM2596 DC-DC buck converter | Regulated 5V / 3.3V power supply |
| DS3231 RTC module | Real-time date and time (I2C) |

## Tools and technologies
- ESP32, Arduino IDE, Embedded C/C++
- TinyML for on-device siren recognition
- I2C communication (LCD, RTC)

## Results
- Successfully detects an ambulance siren using TinyML.
- Automatically identifies Road 1 or Road 2 and gives green-signal priority to the ambulance.
- Reduces ambulance waiting time and keeps normal flow for other vehicles.

## Future scope
- GPS integration to track ambulance location
- IoT-based remote monitoring of detection and signal status
- Camera module for visual emergency-vehicle detection
- Raspberry Pi integration for advanced image processing
- Improved TinyML model for higher accuracy and fewer false detections
- Multi-junction control

## Team
- Gopal S M
- Gopal Reddy
- Venugopal H L

**Guide:** Dr. Tanuja K S, Associate Professor and HOD, Dept. of EEE

## Repository contents
- `code/` ESP32 source code
- `images/` block diagram and project photos
- `demo/` demo video (if available)
