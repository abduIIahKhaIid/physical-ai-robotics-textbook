# Workstation Specifications Reference

This document provides detailed specifications for simulation workstations used in Physical AI & Humanoid Robotics courses.

## Component Requirements

### GPU (Critical Bottleneck)

**Minimum Tier:**
- NVIDIA RTX 4060 Ti (16GB VRAM)
- Price: ~$500-600
- Use case: Basic Isaac Sim scenes, single robot simulations
- Limitations: Struggles with complex environments, limited concurrent model loading

**Recommended Tier:**
- NVIDIA RTX 4070 Ti (12GB VRAM)
- Price: ~$800-1000
- Use case: Standard course work, moderate complexity scenes
- Why: Balances cost and performance, handles most VLA workflows

**Ideal Tier:**
- NVIDIA RTX 3090 (24GB VRAM) or RTX 4090 (24GB VRAM)
- Price: ~$1500-2000
- Use case: Complex sim-to-real training, multiple concurrent models
- Why: High VRAM enables loading USD assets + VLA models simultaneously without swapping

**Critical Notes:**
- RTX series REQUIRED (ray tracing cores needed for Isaac Sim)
- Non-RTX cards (GTX series) CANNOT run Isaac Sim
- VRAM is the primary bottleneck, not raw compute
- Apple M-series chips NOT compatible with Isaac Sim

### CPU

**Minimum:**
- Intel i5-13400 or AMD Ryzen 5 7600
- 6-core/12-thread minimum
- Price: ~$200-250

**Recommended:**
- Intel i7-13700K or AMD Ryzen 7 7700X
- 8+ cores/16+ threads
- Price: ~$350-400

**Ideal:**
- Intel i9-13900K or AMD Ryzen 9 7900X
- 16+ cores/32+ threads
- Price: ~$500-600

**Why it matters:**
Physics calculations (rigid body dynamics) in Gazebo/Isaac are CPU-intensive. Multi-threading helps with parallel simulation steps.

### RAM

**Minimum:**
- 32 GB DDR4/DDR5
- Price: ~$100-150
- Warning: May crash during complex scene rendering

**Recommended:**
- 64 GB DDR5
- Price: ~$200-250
- Why: Prevents crashes, enables smooth multitasking (IDE + browser + simulation)

**Ideal:**
- 128 GB DDR5
- Price: ~$400-500
- Use case: Running multiple simulations simultaneously, large-scale experiments

### Storage

**System Drive (Required):**
- 1 TB NVMe SSD (Gen 4)
- Price: ~$80-120
- Why: Fast OS boot, quick asset loading in Isaac Sim

**Data Drive (Recommended):**
- 2-4 TB NVMe SSD or SATA SSD
- Price: ~$150-300
- Why: Store datasets, simulation recordings, model checkpoints

**Budget Alternative:**
- 512 GB NVMe + 2 TB HDD
- Price: ~$100 total
- Limitation: Slower dataset access times

### Operating System

**Required:**
- Ubuntu 22.04 LTS (Jammy Jellyfish)
- Free (open source)

**Why Ubuntu 22.04:**
- Native ROS 2 Humble/Iron support
- Best driver support for NVIDIA GPUs
- Isaac Sim officially supported
- Industry standard for robotics development

**Windows Alternative:**
- Windows 11 Pro
- Limitations:
  - ROS 2 support via WSL2 (added complexity)
  - Isaac Sim Windows version has known issues
  - Native Linux is strongly preferred

**macOS:**
- NOT RECOMMENDED
- Cannot run Isaac Sim (no NVIDIA GPU support)
- Limited ROS 2 support
- Consider dual-boot Ubuntu or dedicated Linux machine

### Power Supply

**Minimum:**
- 750W 80+ Gold
- Price: ~$100-120

**Recommended:**
- 850W 80+ Gold
- Price: ~$120-150
- Why: Headroom for GPU power spikes, future upgrades

**For RTX 4090 systems:**
- 1000W 80+ Gold
- Price: ~$180-200

### Motherboard

**Key features needed:**
- PCIe 4.0 x16 slot (for GPU)
- Support for chosen CPU socket
- 4x RAM slots (for 64GB+ configurations)
- Multiple M.2 slots (for NVMe storage)

**Price range:** $150-300 depending on features

### Case & Cooling

**Case:**
- Mid-tower ATX with good airflow
- Price: ~$80-150

**CPU Cooler:**
- Air: Noctua NH-D15 (~$100) or equivalent
- Liquid: 240mm AIO (~$120-150)
- Why: High-performance CPUs generate significant heat

**GPU Cooling:**
- Most GPUs come with adequate cooling
- Consider case fans for improved airflow (~$20-40)

## Complete System Configurations

### Budget System (~$2,500)
- CPU: AMD Ryzen 7 7700X
- GPU: NVIDIA RTX 4060 Ti 16GB
- RAM: 32GB DDR5
- Storage: 1TB NVMe + 2TB HDD
- PSU: 750W Gold
- OS: Ubuntu 22.04 LTS

**Use case:** Individual students, basic course work, single robot simulations

### Standard System (~$3,500)
- CPU: Intel i7-13700K
- GPU: NVIDIA RTX 4070 Ti 12GB
- RAM: 64GB DDR5
- Storage: 1TB NVMe + 2TB SSD
- PSU: 850W Gold
- OS: Ubuntu 22.04 LTS

**Use case:** Recommended for course, handles most workflows smoothly

### Premium System (~$5,000+)
- CPU: Intel i9-13900K or AMD Ryzen 9 7950X
- GPU: NVIDIA RTX 4090 24GB
- RAM: 128GB DDR5
- Storage: 2TB NVMe Gen4 + 4TB NVMe
- PSU: 1000W Platinum
- OS: Ubuntu 22.04 LTS

**Use case:** Research labs, complex sim-to-real projects, instructor workstations

## Peripheral Requirements

### Monitor
- Minimum: 1920x1080, 24"
- Recommended: 2560x1440, 27" or dual 1080p
- Ideal: 3840x2160 (4K), 32" for detailed robot visualization

### Input Devices
- Keyboard & Mouse: Any comfortable setup
- 3D Mouse (optional): SpaceMouse Compact (~$150) for easier 3D navigation in Isaac Sim

### Network
- Ethernet adapter (1 Gbps minimum)
- Why: Stable connection for cloud resources, dataset downloads, remote development

## Verification Steps

After assembly, verify hardware with:

1. **GPU Check:**
   ```bash
   nvidia-smi
   # Should show RTX GPU model and VRAM
   ```

2. **CUDA Verification:**
   ```bash
   nvcc --version
   # Should show CUDA 11.8 or 12.x
   ```

3. **Isaac Sim Test:**
   - Launch Isaac Sim
   - Load sample humanoid robot
   - Verify smooth rendering (30+ FPS)

4. **ROS 2 Check:**
   ```bash
   ros2 run demo_nodes_cpp talker
   # Should publish messages without errors
   ```

## Common Issues & Solutions

**Issue:** Isaac Sim crashes on launch
- **Solution:** Check NVIDIA driver version (535+ required), verify VRAM availability

**Issue:** Simulation runs slowly (<15 FPS)
- **Solution:** Reduce scene complexity, lower resolution, check CPU/GPU utilization

**Issue:** Out of memory errors
- **Solution:** Close unnecessary applications, increase swap space, upgrade RAM

**Issue:** ROS 2 packages fail to build
- **Solution:** Verify Ubuntu version, install missing dependencies, check compiler version