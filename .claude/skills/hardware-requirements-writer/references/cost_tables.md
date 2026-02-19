# Cost Tables and Budget Tier Templates

This document provides ready-to-use cost breakdown tables for hardware procurement planning.

## Individual Student Kit Costs

### Option 1: Workstation Only (Cloud-Free)
| Component | Specification | Cost (USD) | Notes |
|-----------|--------------|------------|-------|
| CPU | Intel i7-13700K | $350 | Alternative: AMD Ryzen 7 7700X |
| GPU | NVIDIA RTX 4070 Ti 12GB | $900 | Critical: VRAM determines capability |
| Motherboard | Z790 or B650 | $200 | Must support chosen CPU |
| RAM | 64GB DDR5 | $230 | 32GB minimum, 64GB recommended |
| Storage | 1TB NVMe + 2TB SSD | $180 | Fast system drive essential |
| PSU | 850W 80+ Gold | $130 | Headroom for GPU spikes |
| Case & Cooling | Mid-tower + CPU cooler | $150 | Good airflow critical |
| Monitor | 27" 1440p | $250 | Optional if student has one |
| Peripherals | Keyboard, mouse | $50 | Basic input devices |
| OS | Ubuntu 22.04 LTS | $0 | Free, open source |
| **Subtotal** | | **$2,440** | **Without monitor/peripherals** |
| **Total** | | **$2,740** | **Complete system** |

### Option 2: Workstation + Edge Kit (Complete Physical AI)
| Component | Specification | Cost (USD) | Notes |
|-----------|--------------|------------|-------|
| Workstation | From Option 1 | $2,440 | Simulation and training |
| Jetson Orin Nano Super | 8GB Dev Kit | $249 | Edge inference |
| RealSense D435i | RGB-D + IMU | $349 | Vision and SLAM |
| ReSpeaker Mic Array | 4-mic USB array | $69 | Voice commands |
| microSD Card | 128GB high-endurance | $30 | Jetson storage |
| Cables & Accessories | USB, mounting | $30 | Misc hardware |
| **Total** | | **$3,167** | **Full sim-to-real kit** |

### Option 3: Cloud + Edge (Lower CapEx)
| Component | Specification | Cost (USD) | Notes |
|-----------|--------------|------------|-------|
| Cloud Workstation | AWS g5.2xlarge | $1.50/hr | Only when running simulations |
| Estimated Hours | 10 hrs/week × 12 weeks | $180/quarter | Spot pricing can reduce cost |
| EBS Storage | 100GB for environments | $25/quarter | Pay-per-use |
| Jetson Orin Nano Super | 8GB Dev Kit | $249 | One-time purchase |
| RealSense D435i | RGB-D + IMU | $349 | One-time purchase |
| ReSpeaker Mic Array | 4-mic USB array | $69 | One-time purchase |
| microSD & Accessories | Storage + cables | $60 | One-time purchase |
| **First Quarter** | | **$932** | **Initial hardware + cloud** |
| **Subsequent Quarters** | | **$205** | **Cloud costs only** |

## Institutional Lab Costs

### Small Lab (10 Students)

#### Configuration A: Shared Workstations + Individual Edge Kits
| Item | Quantity | Unit Cost | Total Cost | Notes |
|------|----------|-----------|------------|-------|
| High-end Workstations | 3 | $4,500 | $13,500 | Shared via scheduling |
| Budget Workstations | 4 | $2,440 | $9,760 | Basic simulation |
| Student Edge Kits | 10 | $697 | $6,970 | Individual deployment learning |
| Shared Robot (Unitree Go2) | 1 | $3,000 | $3,000 | Proxy for humanoid |
| Network Infrastructure | - | - | $500 | Switch, cables |
| **Total** | | | **$33,730** | **Setup cost** |
| **Per Student** | | | **$3,373** | **Amortized** |

#### Configuration B: Cloud + Edge Kits
| Item | Quantity | Unit Cost | Total Cost | Notes |
|------|----------|-----------|------------|-------|
| Cloud Credits (per quarter) | 10 students | $205 | $2,050 | Recurring OpEx |
| Student Edge Kits | 10 | $697 | $6,970 | One-time CapEx |
| Shared Robot | 1 | $3,000 | $3,000 | One-time CapEx |
| **First Quarter** | | | **$12,020** | **Initial** |
| **Subsequent Quarters** | | | **$2,050** | **Cloud only** |
| **Per Student (Year 1)** | | | **$1,822** | **Over 3 quarters** |

### Medium Lab (30 Students)

#### Configuration: Hybrid Model
| Item | Quantity | Unit Cost | Total Cost | Notes |
|------|----------|-----------|------------|-------|
| Premium Workstations | 5 | $5,500 | $27,500 | Research/instructor use |
| Standard Workstations | 10 | $3,500 | $35,000 | Student access |
| Student Edge Kits | 30 | $697 | $20,910 | Individual learning |
| Shared Robots (Unitree Go2) | 3 | $3,000 | $9,000 | Team projects |
| Lab Infrastructure | - | - | $2,000 | Networking, furniture |
| **Total** | | | **$94,410** | **Setup cost** |
| **Per Student** | | | **$3,147** | **Amortized** |

## Budget Tier Comparison

### Tier 1: Minimal (Cloud-Only) - $932 first quarter
**What's included:**
- AWS cloud workstation (pay-per-use)
- Basic edge kit (Jetson + camera + mic)

**What you can do:**
- Run Isaac Sim simulations
- Deploy to edge hardware
- Learn ROS 2 and Isaac ROS

**Limitations:**
- No physical robot
- Recurring cloud costs
- Latency issues for real-time control

**Best for:** Individual learners, proof-of-concept, budget-constrained

### Tier 2: Standard (Workstation + Edge) - $3,167
**What's included:**
- Mid-range workstation (RTX 4070 Ti)
- Complete edge kit
- No recurring costs

**What you can do:**
- Full simulation capabilities
- Offline development
- Edge deployment
- No physical robot

**Limitations:**
- No physical robot for final deployment

**Best for:** Serious learners, simulation-focused courses, long-term use

### Tier 3: Complete (Workstation + Edge + Robot) - $6,167+
**What's included:**
- High-end workstation
- Complete edge kit
- Physical robot (Unitree Go2 or similar)

**What you can do:**
- Full sim-to-real pipeline
- Physical robot testing
- Complete course experience

**Limitations:**
- High upfront cost
- Robot maintenance

**Best for:** Institutions, research labs, production-focused programs

## Cost Optimization Strategies

### Strategy 1: Phased Acquisition
**Year 1:** Focus on simulation (workstations only) - $2,440/student
**Year 2:** Add edge kits - +$697/student
**Year 3:** Add shared robots - +$300-600/student (amortized)

**Benefit:** Spread costs over time, validate program before full investment

### Strategy 2: Mixed Fleet
- 3 high-end workstations for instructor + advanced students
- 7 mid-range workstations for general use
- All students get edge kits
- 2 shared robots

**Benefit:** Balances capability with cost, most students don't need premium hardware

### Strategy 3: Cloud Burst Model
- On-premise workstations for daily use
- Cloud instances for peak demand (final projects, parallel experiments)
- Edge kits for all

**Benefit:** Lower baseline cost, flexibility for intensive periods

### Strategy 4: Used/Refurbished Hardware
- Previous-gen GPUs (RTX 3080/3090) often 40-50% cheaper
- Refurbished Jetson modules
- Open-box cameras

**Caution:** Verify warranty, driver support, compatibility

## Maintenance and Ongoing Costs

### Annual Costs per Workstation
| Item | Cost | Frequency | Notes |
|------|------|-----------|-------|
| Electricity | $50-100 | Ongoing | Based on usage |
| Cooling/HVAC | $30-50 | Ongoing | Lab environment |
| Component failures | $100-200 | As needed | Budget for repairs |
| Software licenses | $0 | - | All open-source stack |
| **Total** | **$180-350** | **Per year** | |

### Annual Costs per Edge Kit
| Item | Cost | Frequency | Notes |
|------|------|-----------|-------|
| microSD replacement | $30 | Every 2 years | High-endurance cards wear |
| Cable replacements | $20 | As needed | Wear and tear |
| Camera repairs | $50-100 | Rare | Usually accidental damage |
| **Total** | **$50-100** | **Per year** | |

## Procurement Checklist Template

Use this template for institutional purchasing:

```markdown
# Hardware Procurement Checklist

## Simulation Workstations
- [ ] CPU: Intel i7-13700K (or AMD equivalent) × ____ qty
- [ ] GPU: NVIDIA RTX 4070 Ti 12GB × ____ qty
- [ ] Motherboard: Compatible with chosen CPU × ____ qty
- [ ] RAM: 64GB DDR5 × ____ qty
- [ ] Storage: 1TB NVMe + 2TB SSD × ____ qty
- [ ] PSU: 850W 80+ Gold × ____ qty
- [ ] Case & Cooling × ____ qty
- [ ] Monitor: 27" 1440p × ____ qty
- [ ] Keyboard & Mouse × ____ qty

## Edge Computing Kits
- [ ] Jetson Orin Nano Super Dev Kit (8GB) × ____ qty
- [ ] Intel RealSense D435i × ____ qty
- [ ] ReSpeaker USB Mic Array v2.0 × ____ qty
- [ ] microSD Card 128GB (high-endurance) × ____ qty
- [ ] USB cables (Type-A to Type-C, 1m) × ____ qty

## Robot Hardware (Optional)
- [ ] Unitree Go2 Edu × ____ qty
  OR
- [ ] Unitree G1 Humanoid × ____ qty
  OR
- [ ] Alternative: __________ × ____ qty

## Lab Infrastructure
- [ ] Gigabit Ethernet switch (24-port) × ____ qty
- [ ] Ethernet cables (Cat6, various lengths) × ____ qty
- [ ] Power strips with surge protection × ____ qty
- [ ] Workbenches/desks × ____ qty
- [ ] Storage cabinets for hardware × ____ qty

## Software (Verify Free/Open Source)
- [ ] Ubuntu 22.04 LTS installation media
- [ ] NVIDIA drivers (latest stable)
- [ ] ROS 2 Humble
- [ ] Isaac Sim (NVIDIA download)
- [ ] Gazebo (via ROS 2)

## Total Budget Estimate: $__________
```

## Funding Sources and Grants

### Academic Institutions
- **NSF Equipment Grants:** Research instrumentation
- **Department budgets:** Annual equipment allocation
- **Industry partnerships:** Company-sponsored labs
- **Student lab fees:** Course-specific fees

### Individual Learners
- **Employer sponsorship:** Professional development funds
- **Education loans:** Student loans for equipment
- **Installment plans:** Credit/financing options
- **Used market:** eBay, Reddit hardware swap

### Research Groups
- **Grant indirect costs:** Equipment from overhead
- **Startup packages:** New faculty allocations
- **Collaboration funds:** Multi-institution projects