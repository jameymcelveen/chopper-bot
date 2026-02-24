import subprocess
import os

def run(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True)

def update_ai_md(repo_root):
    # Discovery: Find all ROS 2 nodes
    nodes = []
    for root, dirs, files in os.walk(os.path.join(repo_root, "src")):
        for file in files:
            if file.endswith("_node.py"):
                nodes.append(file.replace(".py", ""))
    
    # GPU check
    docker_content = ""
    if os.path.exists("docker-compose.yml"):
        with open("docker-compose.yml", "r") as f:
            docker_content = f.read()
    gpu_status = "Enabled" if "runtime: nvidia" in docker_content else "Disabled"

    ai_content = f"""# Chopper-Bot (Public)
- **Architecture:** ROS 2 Humble / Jetson Orin Nano
- **GPU Runtime:** {gpu_status}

## 📍 Resume Here: Hardware Hookup
- **Spektrum Satellite Receiver:** Connect to J41 Header.
    - **VCC:** Pin 1 (3.3V) — **CRITICAL: DO NOT USE 5V**
    - **GND:** Pin 6 or 14.
    - **RX:** Pin 10 (UART2_RX / `/dev/ttyTHS1`).
- **NVIDIA Pinout Reference:** [Official Jetson Orin Nano Header Guide](https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide/howto.html#id1)

## Current Nodes
{chr(10).join([f"- {n}" for n in nodes])}

## Shop TODO List
- [ ] Install NVIDIA Container Toolkit on Orin Nano
- [ ] Solder/Pin-out Spektrum Satellite
- [ ] Run 'make build' on Jetson
"""
    with open("ai.md", "w") as f:
        f.write(ai_content)

def main():
    repo_root = subprocess.check_output(['git', 'rev-parse', '--show-toplevel']).decode('utf-8').strip()
    os.chdir(repo_root)
    
    update_ai_md(repo_root)
    
    msg = "feat: add hardware resume notes to ai.md for next session"
    print(f"💾 Syncing: {msg}")
    
    run("git add -A")
    result = run(f'git commit -m "{msg}"')
    
    if result.returncode == 0:
        print("🚀 Pushing to origin...")
        run("git push")
        print("✅ Mission Accomplished. Resume notes saved to ai.md.")
    else:
        print("⚠️ Nothing new to commit.")

if __name__ == "__main__":
    main()