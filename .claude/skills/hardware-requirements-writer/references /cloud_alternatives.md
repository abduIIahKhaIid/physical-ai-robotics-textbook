# Cloud Alternatives for Physical AI Education

This document details cloud-based alternatives to on-premise hardware for running Isaac Sim, Gazebo, and ROS 2 workloads.

## When to Use Cloud Infrastructure

**Choose cloud when:**
- Students have weak local hardware (non-RTX laptops, Macs)
- Program is short-term (single quarter/semester)
- Budget favors OpEx over CapEx
- Need to scale quickly (cohort size uncertain)
- Testing before major hardware investment

**Avoid cloud when:**
- Long-term program (>1 year) - on-premise becomes cheaper
- Need to control physical robots in real-time (latency issues)
- Tight budget with large student base (recurring costs add up)
- Unreliable internet connectivity

## Cloud Provider Options

### AWS (Amazon Web Services)

#### Recommended Instance: g5.2xlarge
**Specifications:**
- GPU: NVIDIA A10G (24GB VRAM)
- vCPU: 8 cores (AMD EPYC 7R32)
- RAM: 32 GB
- Storage: EBS volumes (charged separately)
- Network: Up to 10 Gbps

**Pricing:**
- On-Demand: ~$1.51/hour
- Spot Instance: ~$0.60/hour (savings up to 60%, but can be interrupted)
- Reserved (1 year): ~$0.90/hour

**Cost estimate (per student, per quarter):**
- 10 hours/week × 12 weeks = 120 hours
- On-Demand: 120 × $1.51 = $181.20
- Spot: 120 × $0.60 = $72.00
- Storage (100GB EBS): ~$10/month = $30/quarter
- **Total: $102-211 per student per quarter**

#### Alternative: g5.xlarge (Budget Option)
- GPU: NVIDIA A10G (24GB VRAM)
- vCPU: 4 cores
- RAM: 16 GB
- Price: ~$1.00/hour on-demand

**When to use:** Basic simulations, single robot, budget-constrained

#### Alternative: g6e.xlarge (Latest Gen)
- GPU: NVIDIA L40S (48GB VRAM)
- vCPU: 4 cores
- RAM: 16 GB
- Price: ~$1.30/hour

**When to use:** Latest GPU, better performance per dollar

#### Setup Steps (AWS)

1. **Launch Instance:**
   ```bash
   # Use AWS Deep Learning AMI (Ubuntu 22.04)
   # Select g5.2xlarge instance type
   # Configure 100-200GB root volume
   # Add security group rules (SSH, RDP if needed)
   ```

2. **Install Isaac Sim:**
   ```bash
   # Install NVIDIA Omniverse
   # Download Isaac Sim via Omniverse Launcher
   # Note: May require NVIDIA Enterprise account
   ```

3. **Install ROS 2:**
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

4. **Configure Remote Access:**
   - X11 forwarding for GUI (high latency)
   - VNC server for remote desktop
   - NoMachine for better performance

### Azure (Microsoft Azure)

#### Recommended Instance: NC6s v3
**Specifications:**
- GPU: NVIDIA V100 (16GB VRAM)
- vCPU: 6 cores (Intel Xeon E5-2690 v4)
- RAM: 112 GB
- Storage: Premium SSD

**Pricing:**
- Pay-as-you-go: ~$3.06/hour
- Spot: ~$0.90/hour
- Reserved (1 year): ~$1.80/hour

**Note:** More expensive than AWS g5 but includes significantly more RAM

#### Alternative: NCasT4_v3 (Budget)
- GPU: NVIDIA T4 (16GB VRAM)
- vCPU: 4 cores
- RAM: 28 GB
- Price: ~$0.53/hour

**Limitation:** T4 lacks RTX cores, may not support Isaac Sim well

#### Setup Steps (Azure)

1. **Create VM:**
   - Use "Data Science Virtual Machine - Ubuntu 22.04"
   - Select NCv3 or NCasT4_v3 series
   - Configure networking (SSH, RDP)

2. **Install Dependencies:**
   ```bash
   # NVIDIA drivers pre-installed in DSVM
   # Install Isaac Sim (same as AWS)
   # Install ROS 2 Humble
   ```

### Google Cloud Platform (GCP)

#### Recommended Instance: n1-standard-8 + NVIDIA T4
**Specifications:**
- GPU: NVIDIA T4 (16GB VRAM) - 1x
- vCPU: 8 cores
- RAM: 30 GB
- Storage: Persistent Disk

**Pricing:**
- On-Demand: ~$0.95/hour
- Preemptible: ~$0.29/hour
- Committed use (1 year): ~$0.60/hour

**Note:** T4 not ideal for Isaac Sim (no RTX), better for ROS 2 + Gazebo

#### Alternative: a2-highgpu-1g (Premium)
- GPU: NVIDIA A100 (40GB VRAM)
- vCPU: 12 cores
- RAM: 85 GB
- Price: ~$3.67/hour

**When to use:** Research projects, complex VLA training

### NVIDIA GPU Cloud (NGC)

#### Omniverse Cloud
**Features:**
- Isaac Sim pre-installed
- Managed Omniverse environment
- Pay-per-use streaming

**Pricing:**
- Contact NVIDIA for institutional pricing
- Typically $1-2/hour per stream

**Pros:**
- Zero setup time
- Official NVIDIA support
- Optimized for Isaac Sim

**Cons:**
- Higher cost than raw compute
- Requires internet connection for streaming
- Limited customization

## Cloud Setup Best Practices

### Image Management

**Create Base AMI/Image:**
1. Launch fresh instance
2. Install all dependencies (ROS 2, Isaac Sim, Gazebo)
3. Configure environment variables
4. Create snapshot/AMI
5. Students launch from this image

**Benefits:**
- Consistent environment
- Fast provisioning (5-10 min vs hours)
- Easier troubleshooting

### Cost Management

**Strategy 1: Stop When Not In Use**
```bash
# Stop instance (preserves state, charges only for storage)
aws ec2 stop-instances --instance-ids i-1234567890abcdef0

# Start when needed
aws ec2 start-instances --instance-ids i-1234567890abcdef0
```

**Savings:** Pay only for hours used + minimal storage costs

**Strategy 2: Use Spot Instances**
- Set max price (e.g., $1.00/hour)
- Can be interrupted with 2-minute notice
- Save simulation state frequently
- **Risk:** May lose work if interrupted mid-simulation

**Strategy 3: Schedule Auto-Shutdown**
```bash
# Cron job to shutdown at midnight
0 0 * * * sudo shutdown -h now
```

**Prevents:** Forgetting to stop instances, runaway costs

### Storage Optimization

**Use Separate Volumes:**
- Root volume (50GB): OS, software
- Data volume (100GB): Simulations, datasets, models
- Snapshot data volume regularly

**Benefits:**
- Faster instance replacement
- Preserve data across instance changes
- Share data volumes between students

### Network Configuration

**Security Groups (Firewall Rules):**
- SSH (port 22): Your IP only
- RDP/VNC (port 3389/5900): Your IP only
- ROS 2 ports: Restrict to VPC if multi-instance

**VPN for Teams:**
- Set up VPN gateway
- Students connect to VPN
- All instances in private subnet
- Better security, easier ROS 2 multi-machine

## Remote Desktop Solutions

### Option 1: X11 Forwarding (Simplest)
```bash
ssh -X ubuntu@<instance-ip>
gazebo  # GUI will forward to local machine
```

**Pros:** No extra setup, works on Linux/Mac
**Cons:** High latency, laggy for 3D graphics

### Option 2: VNC (Better Performance)
```bash
# On instance
sudo apt install tigervnc-standalone-server
vncserver :1 -geometry 1920x1080 -depth 24

# On local machine
vncviewer <instance-ip>:5901
```

**Pros:** Lower latency than X11, full desktop
**Cons:** Still not ideal for real-time simulation

### Option 3: NoMachine (Best Performance)
- Install NoMachine server on instance
- Install NoMachine client locally
- Hardware video encoding/decoding

**Pros:** Smooth 3D graphics, low latency
**Cons:** Requires setup, proprietary software

### Option 4: NICE DCV (AWS Native)
- AWS-optimized remote desktop
- Hardware-accelerated streaming
- Integrated with AWS console

**Pros:** Best for AWS, high quality
**Cons:** AWS only, learning curve

## Hybrid Cloud + Local Model

### Configuration: Cloud for Simulation, Local for Edge

**Setup:**
1. Students use cloud instances for Isaac Sim / Gazebo
2. Train models in cloud
3. Download model weights
4. Deploy to local Jetson hardware

**Workflow:**
```
Cloud (Training) → Model Weights → Local Jetson (Inference)
                 ↓
                SSH/SCP transfer
```

**Benefits:**
- Avoids latency for physical control
- Utilizes cloud for heavy compute
- Local Jetson for real-world testing

**Cost:** Cloud + edge kit (~$700 one-time + $200/quarter)

## Cloud Provider Comparison

| Feature | AWS g5.2xlarge | Azure NCv3 | GCP n1 + T4 | NGC Omniverse |
|---------|----------------|------------|-------------|---------------|
| GPU | A10G (24GB) | V100 (16GB) | T4 (16GB) | A10/A40 |
| Isaac Sim Support | ✅ Excellent | ✅ Good | ⚠️ Limited (No RTX) | ✅ Native |
| Cost (on-demand) | $1.51/hr | $3.06/hr | $0.95/hr | ~$1.50/hr |
| Spot/Preempt | $0.60/hr | $0.90/hr | $0.29/hr | ❌ N/A |
| Setup Complexity | Medium | Medium | Medium | Low |
| Best For | General use | RAM-heavy | Budget | Isaac-specific |

## Institutional Licenses and Discounts

### AWS Educate
- $100 free credits per student
- Apply as educator for classroom resources
- Access to AWS Educate Starter Account

### Azure for Students
- $100 free credit for students
- Requires .edu email verification
- Renewable annually

### GCP Education Grants
- $300 free trial (all users)
- Apply for education grants ($1000-5000 for classrooms)
- Requires faculty application

### NVIDIA Academic Programs
- NGC credits for universities
- Isaac Sim licenses (may be free for education)
- Contact NVIDIA academic team

## Monitoring and Budgeting

### Set Up Billing Alerts
```bash
# AWS CLI example
aws budgets create-budget \
  --account-id 123456789012 \
  --budget file://budget.json \
  --notifications-with-subscribers file://notifications.json
```

**Alert thresholds:**
- 50% of budget: Warning email
- 80% of budget: Urgent notification
- 100% of budget: Auto-shutdown (optional)

### Track Usage per Student
- Tag instances with student IDs
- Generate cost reports by tag
- Identify high-usage outliers

### Instance Idle Detection
```bash
# Check CPU usage, shutdown if idle >30 min
if [ $(uptime | awk '{print $10}' | cut -d',' -f1 | cut -d'.' -f1) -lt 5 ]; then
  sudo shutdown -h now
fi
```

## Limitations of Cloud Approach

### Real-Time Robot Control
- **Latency:** 50-200ms typical (AWS to user)
- **Problem:** Unacceptable for balance control, grasping
- **Solution:** Use cloud for simulation, local Jetson for physical robot

### Data Transfer Costs
- **Egress:** $0.09/GB (AWS)
- **Large datasets:** Training data, simulation recordings add up
- **Mitigation:** Keep data in cloud, download only final models

### Simulation Determinism
- **Varying performance:** Cloud instances share underlying hardware
- **Problem:** Physics simulations may behave inconsistently
- **Solution:** Use dedicated instances (more expensive)

## Example Cloud Lab Setup (30 Students)

### Architecture
- 1 shared storage server (EFS/Cloud Filestore)
- 30 student instances (g5.xlarge, stopped when not in use)
- 1 instructor instance (g5.2xlarge, always available)
- VPN gateway for secure access

### Cost Breakdown (per quarter)
| Item | Cost |
|------|------|
| Student instances (10 hrs/wk each) | 30 × $120 = $3,600 |
| Instructor instance (full-time) | $1,510 × 3 months = $4,530 |
| Shared storage (500GB) | $150/month × 3 = $450 |
| Data transfer | ~$200 |
| **Total** | **$8,780** |
| **Per student** | **$293** |

**Comparison to on-premise:**
- On-premise: $3,147/student one-time
- Cloud: $293/student per quarter
- **Break-even:** ~11 quarters (~3 years)

## Recommendations by Use Case

### Single Student, Short-Term (<3 months)
- **Recommend:** AWS g5.2xlarge spot instances
- **Cost:** ~$100-150 total
- **Alternative:** GCP preemptible for tighter budget

### Classroom (15-30 students), One Quarter
- **Recommend:** AWS Educate + spot instances
- **Cost:** ~$200-300 per student
- **Setup:** Pre-configured AMI, scheduled shutdown

### Long-Term Program (>1 year)
- **Recommend:** On-premise workstations
- **Reason:** Break-even point at ~1 year, ongoing savings
- **Hybrid:** On-premise + cloud burst for peak demand

### Research Group
- **Recommend:** Hybrid (on-premise training rig + cloud for experiments)
- **Reason:** Flexibility, cost optimization
- **Consider:** Reserved instances for predictable workloads
