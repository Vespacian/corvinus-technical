# corvinus-technical

## Goals: 
- Goal 1: In a preferred simulation software with a 3D engine set-up a virtual box on a table.
- Goal 2: Create a second box.
- Goal 3: Push the first block against the 2nd to move the 2nd block to either the corner of the table.
- Goal 4: Create a barrier between the left and right side of the table at ⅓ the height of an imported 6-DOF arm, pick up a block and move it onto the other side of the barrier
- Goal 5: Incorporate collection of sensor data that can realistically be used (RGB, tactile, IMU, joint-state, etc.).
- Goal 6: Perform the next step of this challenge. The next most important step to making this training data for a VLA model. 

## Setup

**1. Create conda environment**
```powershell
conda create -n isaacenv python=3.11 -y
conda activate isaacenv
```

**2. Install CUDA-enabled PyTorch**
```powershell
pip install torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
```

**3. Install Isaac Sim 5.0 with all extensions**
```powershell
pip install "isaacsim[all,extscache]==5.0.0" --extra-index-url https://pypi.nvidia.com
```

**4. Accept EULA (run once per session)**
```powershell
$env:OMNI_KIT_ACCEPT_EULA = "YES"
```

**5. Download Franka robot assets**
```powershell
python scripts/download_franka.py
```

> Downloads the Franka Panda USD and all mesh sub-assets into `assets/` and `assets/Props/`. These files are gitignored due to size — run this once after cloning.

## How to Run

```powershell
conda activate isaacenv
$env:OMNI_KIT_ACCEPT_EULA = "YES"

python scripts/goal1.py 
python scripts/goal2.py 
python scripts/goal3.py 
python scripts/goal4.py 
python scripts/goal5.py 
```

## Notes
- may take a while to first run
- each script has their own standalone workflow so just the script itself needs to be run
- *before running goals 4 and 5*, run the `/scripts/download_franka.py` script first
    - this creates the `/assets/` folder which contains Franka Panda robot arm model data
- when running goals 4 and 5, there may be leftover processes that don't fully terminate. Added in code that handles that on each new run
