# Edge Computing Kit Specifications

This document details the hardware requirements for edge AI deployment in Physical AI courses, focusing on the NVIDIA Jetson platform for running inference and real-world robot control.

## Jetson Platform Options

### Jetson Orin Nano Super Dev Kit (8GB) - Recommended Entry
- **Price:** $249 (official MSRP, price dropped from ~$499)
- **Performance:** 40 TOPS (Tera Operations Per Second)
- **Memory:** 8GB unified RAM (shared between CPU and GPU)
- **Power:** 7W - 15W configurable TDP
- **Use case:** Student learning kits, basic perception pipelines, voice-to-action

**Pros:**
- Most affordable Orin platform
- Sufficient for Isaac ROS packages
- Built-in Wi-Fi (new "Super" kit includes module)
- Active community support

**Cons:**
- Limited RAM for complex VLA models
- Lower performance than higher-tier Jetsons
- May struggle with high-resolution multi-camera SLAM

### Jetson Orin NX (16GB)
- **Price:** $599 (16GB version)
- **Performance:** 100 TOPS
- **Memory:** 16GB unified RAM
- **Power:** 10W - 25W configurable
- **Use case:** Production deployments, multi-sensor fusion, real-time navigation

**Pros:**
- 2.5x performance vs Orin Nano
- Sufficient RAM for larger VLA models
- Better thermal management
- Production-ready

**Cons:**
- Higher cost
- Requires carrier board or dev kit

### Jetson AGX Orin (32GB/64GB)
- **Price:** $1,199 (32GB), $1,999 (64GB)
- **Performance:** 200-275 TOPS
- **Memory:** 32GB or 64GB unified RAM
- **Use case:** Research platforms, multi-robot systems, heavy compute workloads

**Only for:** Well-funded labs or specific research needs

## Required Peripherals

### Vision System: Intel RealSense D435i
- **Price:** $349
- **Features:**
  - RGB camera (1920×1080 @ 30fps)
  - Dual infrared depth cameras
  - Built-in IMU (accelerometer + gyroscope)
  - Depth range: 0.2m - 10m
- **Why this model:**
  - IMU essential for SLAM (D435 non-i lacks IMU)
  - Industry standard, excellent ROS 2 support
  - Compact form factor for robot mounting

**Alternative: Intel RealSense D455**
- Price: $529
- Advantage: Longer depth range (up to 20m), better outdoor performance
- When to choose: Mobile robots, outdoor navigation

**Budget Alternative: Generic USB Webcam + Separate IMU**
- Price: ~$50 (webcam) + $30 (IMU breakout board)
- Limitation: No depth sensing, manual calibration required
- Use case: Voice commands only, no SLAM/navigation

### Voice Interface: ReSpeaker USB Mic Array v2.0
- **Price:** $69
- **Features:**
  - 4-microphone array (far-field capture)
  - 360° voice pickup
  - Built-in audio processing (AEC, beamforming)
  - USB plug-and-play
- **Why needed:**
  - Module 4 (Voice-to-Action) requires voice input
  - Far-field performance better than laptop/phone mics
  - Directional audio helps in noisy environments

**Alternative: Standard USB Microphone**
- Price: ~$20-40
- Limitation: Poorer far-field performance, manual wake-word detection
- Use case: Quiet lab environments, close-range interaction

### IMU (If not using RealSense D435i)
- **Model:** Adafruit BNO055 9-DOF Absolute Orientation Sensor
- **Price:** $35
- **Features:**
  - 3-axis accelerometer, gyroscope, magnetometer
  - Built-in sensor fusion
  - I2C interface
- **Why needed:** Balance and orientation for locomotion control

### Storage (microSD Card)
- **Minimum:** 64GB UHS-I (U3)
- **Recommended:** 128GB UHS-I (U3) high-endurance
- **Price:** ~$20-30
- **Why:** Stores JetPack OS, Isaac ROS packages, model weights

**Critical:** Use high-endurance cards rated for continuous write (security camera grade)

### Power Supply
- **Included:** Dev kits include appropriate power adapter
- **For custom deployments:**
  - 5V/4A USB-C for Orin Nano
  - 9-19V DC barrel jack for higher-tier Jetsons
  - Consider battery packs for mobile robots (LiPo 3S/4S with voltage regulator)

### Networking
- **Wi-Fi:** Built-in on Orin Nano Super, add-on module for older kits
- **Ethernet:** Gigabit port (recommended for initial setup, SSH access)
- **Consider:** USB Wi-Fi dongle for dual-band support if needed

## Complete Edge Kit Configurations

### Student Learning Kit (~$700)
- Jetson Orin Nano Super Dev Kit (8GB): $249
- Intel RealSense D435i: $349
- ReSpeaker USB Mic Array: $69
- 128GB microSD (high-endurance): $30
- **Total:** ~$697

**Use case:** Individual student deployment, Modules 3-4, sim-to-real basics

### Production Edge Kit (~$1,200)
- Jetson Orin NX Dev Kit (16GB): $599
- Intel RealSense D455: $529
- ReSpeaker Mic Array: $69
- Storage + accessories: $50
- **Total:** ~$1,247

**Use case:** Real robot deployments, research projects, multi-sensor systems

### Budget Minimal Kit (~$350)
- Jetson Orin Nano Super (8GB): $249
- Generic USB webcam: $50
- Adafruit BNO055 IMU: $35
- Standard USB mic: $20
- Storage: $20
- **Total:** ~$374

**Use case:** Voice commands only, basic ROS 2 learning, no SLAM

## Software Stack for Edge Devices

### JetPack SDK (Included)
- Ubuntu 20.04 LTS (for older Jetsons) or 22.04 LTS (Orin series)
- CUDA, cuDNN, TensorRT (hardware-accelerated AI inference)
- Multimedia APIs (camera, video encode/decode)

### Isaac ROS Packages
- Visual SLAM (vSLAM)
- Object detection (YOLO, DOPE)
- Depth perception (stereo, depth estimation)
- Sensor drivers (RealSense, lidar)

### Voice Processing
- OpenAI Whisper (speech-to-text)
- Text-to-Speech (TTS) engine
- Wake word detection (Porcupine, Snowboy)

## Setup Verification

### 1. Jetson Initial Boot
```bash
# Check JetPack version
sudo apt-cache show nvidia-jetpack

# Verify GPU
jtop  # Interactive system monitor (install with: sudo pip3 install jetson-stats)
```

### 2. RealSense Camera Test
```bash
# Install RealSense SDK
sudo apt install ros-humble-realsense2-camera

# Test depth stream
ros2 run realsense2_camera realsense2_camera_node
ros2 topic echo /camera/depth/image_rect_raw
```

### 3. Microphone Test
```bash
# List audio devices
arecord -l

# Record 5-second test
arecord -D hw:1,0 -f cd -d 5 test.wav

# Play back
aplay test.wav
```

### 4. GPU Performance
```bash
# Run TensorRT benchmark
/usr/src/tensorrt/bin/trtexec --onnx=model.onnx --fp16

# Check power mode
sudo nvpmodel -q
```

## Common Edge Deployment Issues

### Issue: Camera not detected
- **Solution:** Check USB 3.0 connection, verify RealSense firmware, try different USB port

### Issue: Out of memory (OOM) errors
- **Solution:** Reduce model size, enable swap space, use quantized models (INT8)

### Issue: Slow inference times
- **Solution:** Verify TensorRT optimization, check power mode (max performance), reduce input resolution

### Issue: Wi-Fi connectivity unstable
- **Solution:** Use external antenna, switch to 5GHz band, prefer Ethernet for stationary setups

### Issue: Voice commands not recognized
- **Solution:** Adjust microphone sensitivity, reduce background noise, retrain wake word model

## Role Differentiation: Workstation vs Edge

| Task | Workstation | Edge Device (Jetson) |
|------|-------------|----------------------|
| Training VLA models | ✅ Primary | ❌ Too slow |
| Running Isaac Sim | ✅ Required | ❌ Not supported |
| Inference (real-time) | ⚠️ Overkill | ✅ Optimized |
| Real robot control | ❌ Too much latency | ✅ Low latency |
| Sensor processing | ⚠️ Can do | ✅ Hardware accelerated |
| Development & debugging | ✅ Preferred | ⚠️ Limited by resources |

**Key principle:** Train on workstation, deploy to edge device. Use Jetson to understand resource constraints vs powerful workstations.

## Power and Thermal Considerations

### Power Modes (Jetson)
- **MAXN:** Maximum performance, highest power draw
- **Mode 15W:** Balanced performance/power
- **Mode 10W:** Low power, reduced performance

**Configure with:**
```bash
sudo nvpmodel -m 0  # MAXN mode
```

### Thermal Management
- Passive cooling (heatsink): Adequate for dev kits at moderate loads
- Active cooling (fan): Required for sustained high-performance workloads
- Monitor temps with `jtop` or `tegrastats`

### Battery-Powered Deployments
- Orin Nano: ~10-15W typical → 2-3 hours on 50Wh battery
- Include voltage regulator (5V/4A minimum)
- Consider power budget for motors, sensors

## Integration with Robot Hardware

### Mounting Considerations
- Jetson: Compact carrier boards fit inside robot chassis
- RealSense: Front-facing, clear FOV, vibration isolation
- Mic array: Top-mounted for 360° pickup, minimize motor noise

### Cable Management
- USB 3.0: Max 3m for RealSense (use quality cables)
- Power: Secure connections, strain relief
- Ethernet: For development, switch to Wi-Fi in production

### Example Physical Integration
```
Robot Chassis
├── Jetson Orin Nano (inside, protected)
├── RealSense D435i (front bumper, forward-facing)
├── ReSpeaker Mic (top plate, unobstructed)
└── Battery pack (bottom, low CG)
```