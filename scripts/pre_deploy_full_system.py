#!/usr/bin/env python3
"""
PlatformIO Pre-deployment Script for Full System Upload
"""

import subprocess
import sys
import os

# Get the project directory - PlatformIO compatible
try:
    # Try to use __file__ if available
    project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
except NameError:
    # Fallback for PlatformIO environment
    project_dir = os.getcwd()

def run_full_deployment():
    """Run the full system deployment script"""
    script_path = os.path.join(project_dir, 'scripts', 'deploy_full_system.py')
    
    print("=" * 60)
    print(">>> LAUNCHING FULL HUB SYSTEM DEPLOYMENT")
    print("=" * 60)
    
    try:
        result = subprocess.run([sys.executable, script_path], 
                              cwd=project_dir,
                              check=False)
        
        if result.returncode == 0:
            print("[OK] Full system deployment completed successfully!")
        else:
            print("[FAIL] Full system deployment failed!")
            sys.exit(1)
            
    except Exception as e:
        print(f"[ERROR] Error running full deployment: {e}")
        sys.exit(1)

# This script is called by PlatformIO before upload
if __name__ == "__main__":
    run_full_deployment()