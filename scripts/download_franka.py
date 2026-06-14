# download_franka.py
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import requests
import os

BASE_URL = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Franka"

# All files needed by franka.usd
FILES = [
    "franka.usd",
    "Props/panda_link0.usd",
    "Props/panda_link1.usd",
    "Props/panda_link2.usd",
    "Props/panda_link3.usd",
    "Props/panda_link4.usd",
    "Props/panda_link5.usd",
    "Props/panda_link6.usd",
    "Props/panda_link7.usd",
    "Props/panda_hand.usd",
    "Props/panda_leftfinger.usd",
    "Props/panda_rightfinger.usd",
]

script_dir = os.path.dirname(os.path.abspath(__file__))
assets_dir = os.path.join(script_dir, "..", "assets")

for file in FILES:
    url = f"{BASE_URL}/{file}"
    local_path = os.path.join(assets_dir, file)
    os.makedirs(os.path.dirname(local_path), exist_ok=True)

    print(f"Downloading: {file}")
    try:
        r = requests.get(url, timeout=15)
        if r.status_code == 200:
            with open(local_path, "wb") as f:
                f.write(r.content)
            print(f"  Saved ({len(r.content)} bytes)")
        else:
            print(f"  FAILED — status {r.status_code}")
    except Exception as e:
        print(f"  ERROR — {e}")

print("\nDone! All files saved to assets/")
simulation_app.close()