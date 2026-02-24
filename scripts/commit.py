import subprocess
import os
import sys

def run_cmd(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True)

def main():
    repo_root = subprocess.check_output(['git', 'rev-parse', '--show-toplevel']).decode('utf-8').strip()
    os.chdir(repo_root)
    
    msg_path = os.path.join(repo_root, ".gemini_msg")
    
    # 1. Get the message
    if os.path.exists(msg_path):
        with open(msg_path, 'r') as f:
            msg = f.read().strip()
    else:
        msg = "chore: automated sync"

    print(f"💾 Staging and committing: {msg}")
    
    # 2. Git flow
    run_cmd("git add -A")
    result = run_cmd(f'git commit -m "{msg}"')
    
    if result.returncode == 0:
        print("🚀 Pushing to origin...")
        run_cmd("git push")
        if os.path.exists(msg_path):
            os.remove(msg_path)
        print("✅ Done!")
    else:
        print("⚠️ Nothing to commit or commit failed.")
        print(result.stderr)

if __name__ == "__main__":
    main()
