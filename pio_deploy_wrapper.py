#!/usr/bin/env python3
"""
PlatformIO Extra Script Wrapper for Complete System Deployment
This allows triggering the deployment from VS Code PlatformIO
"""

Import("env")
import subprocess
import sys
import os

def deploy_complete_system(*args, **kwargs):
    """Wrapper function to run the deployment script"""
    print("\n" + "="*60)
    print("🚀 TRIGGERING COMPLETE SYSTEM DEPLOYMENT")
    print("="*60)
    
    # Get current working directory
    project_dir = env.get("PROJECT_DIR")
    script_path = os.path.join(project_dir, "deploy_complete_system.py")
    
    if not os.path.exists(script_path):
        print(f"❌ Deployment script not found: {script_path}")
        return
    
    try:
        # Run the deployment script
        result = subprocess.run([sys.executable, script_path], 
                              cwd=project_dir, 
                              timeout=600)  # 10 minute timeout
        
        if result.returncode == 0:
            print("\n🎉 DEPLOYMENT COMPLETED SUCCESSFULLY!")
        else:
            print(f"\n⚠️  Deployment completed with warnings (return code: {result.returncode})")
            
    except subprocess.TimeoutExpired:
        print("⏰ Deployment timed out after 10 minutes")
    except Exception as e:
        print(f"❌ Deployment failed: {e}")

# Add the custom target
env.AddCustomTarget(
    name="deploy_all",
    dependencies=None,
    actions=[deploy_complete_system],
    title="Deploy Complete HUB System",
    description="Deploy Master + All 5 Slots automatically"
)

# Also run on post action if this is the deploy_all_systems environment
if env.get("PIOENV") == "deploy_all_systems":
    env.AddPostAction("buildprog", deploy_complete_system)
elif env.get("PIOENV") == "quick_deploy":
    env.AddPreAction("buildprog", deploy_complete_system)