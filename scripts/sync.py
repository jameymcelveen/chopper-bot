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
    
    # Discovery: Check for GPU runtime in docker-compose
    gpu_status = "Enabled" if "runtime: nvidia" in open("docker-compose.yml").read() else "Disabled"

    ai_content = f"""# Chopper-Bot (Public)
- **Architecture:** ROS 2 Humble / Jetson Orin Nano
- **GPU Runtime:** {gpu_status}

## Current Nodes
{chr(10).join([f"- {n}" for n in nodes])}

## Shop TODO List
- [ ] Verify Spektrum UART wiring on J41 pins 8/10
- [ ] Test 2020 Skeleton URDF in Rviz
"""
    with open("ai.md", "w") as f:
        f.write(ai_content)

def main():
    repo_root = subprocess.check_output(['git', 'rev-parse', '--show-toplevel']).decode('utf-8').strip()
    os.chdir(repo_root)
    
    # 1. Update project documentation and metadata
    update_ai_md(repo_root)
    
    # 2. Define the commit message
    # In a real sync, we could pass this as an argument
    msg = "feat: consolidate automation into python and update ai.md node list"
    
    print(f"💾 Syncing and committing: {msg}")
    
    # 3. Git flow
    run("git add -A")
    result = run(f'git commit -m "{msg}"')
    
    if result.returncode == 0:
        print("🚀 Pushing to origin...")
        run("git push")
        print("✅ Mission Accomplished.")
    else:
        print("⚠️ Nothing new to commit.")

if __name__ == "__main__":
    main()