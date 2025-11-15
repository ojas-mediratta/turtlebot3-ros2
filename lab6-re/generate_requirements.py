import os
import subprocess
import sys

# Step 1: Get Python version
print("Getting Python version...")
try:
    # --- MODIFIED: Use sys.executable ---
    # This ensures we use the python from the active venv
    python_executable = sys.executable 
    
    python_version_result = subprocess.run(
        [python_executable, '--version'], 
        capture_output=True, 
        text=True, 
        check=True
    )
    python_version = python_version_result.stdout.strip()
except Exception as e:
    print(f"Warning: Could not get Python version. {e}")
    python_version = "Python (version unknown)"

print(f"Using {python_version}")
print(f"Interpreter path: {python_executable}") # Added for clarity

# Step 2: Generate requirements.txt using pipreqs
print("Running pipreqs to generate requirements.txt...")
print("This will scan all .py files in the current directory and subdirectories.")
try:
    # We run pipreqs on the current directory '.'
    # and use '--force' to overwrite any existing requirements.txt
    
    # --- MODIFIED: Use sys.executable ---
    subprocess.run(
        [
            python_executable, '-m', 'pipreqs', 
            '.',                # Scan the current directory
            '--encoding=utf-8', 
            '--force'           # Overwrite requirements.txt if it exists
        ],
        check=True,
        # We don't capture stdout/stderr, so we can see pipreqs' output
    )
    print("Successfully generated requirements.txt")
except FileNotFoundError:
    print("\n--- ERROR ---")
    print("Error: 'pipreqs' command not found.")
    print("Please make sure 'pipreqs' is installed in your (lab6_env) environment:")
    print("  pip install pipreqs")
    print("-------------")
    sys.exit(1)
except subprocess.CalledProcessError as e:
    print("\n--- ERROR ---")
    print(f"Error: pipreqs failed with exit status {e.returncode}.")
    print("This can happen if it fails to parse a file or finds no imports.")
    print("Please ensure all dependencies are installed (e.g., pip install scikit-learn numpy opencv-python ...)")
    if e.output:
        print(f"Full output:\n{e.output.decode()}")
    print("-------------")
    sys.exit(1)

# Step 3: Add Python version as a comment to the top of requirements.txt
requirements_file = 'requirements.txt'
if os.path.exists(requirements_file):
    print(f"Prepending Python version to {requirements_file}...")
    try:
        with open(requirements_file, 'r') as file:
            content = file.readlines()
        
        # Remove old version comment if it exists
        content = [line for line in content if not line.strip().startswith('# Python')]
        
        content.insert(0, f'# {python_version}\n')
        
        with open(requirements_file, 'w') as file:
            file.writelines(content)
        print("Done.")
    except Exception as e:
        print(f"Error: Failed to write to {requirements_file}. {e}")
else:
    print(f"Error: {requirements_file} was not created by pipreqs.")