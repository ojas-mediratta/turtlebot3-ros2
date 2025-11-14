import os
import subprocess
import tempfile
import shutil


# Step 1: Generate requirements.txt using pipreqs
with tempfile.TemporaryDirectory() as tmpdir:
    # Copy only root-level Python/Jupyter files to temp directory
    for f in os.listdir('.'):
        if os.path.isfile(f) and f.split('.')[-1] in ['py', 'ipynb']:
            shutil.copy2(f, tmpdir)
    
    # Generate requirements.txt in temp directory
    subprocess.run(
        ['python3', '-m', 'pipreqs.pipreqs', '--encoding=utf-8', tmpdir],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT
    )
    
    # Move requirements.txt to project root if created
    temp_req = os.path.join(tmpdir, 'requirements.txt')
    if os.path.exists(temp_req):
        shutil.move(temp_req, 'requirements.txt')
        print("Generated requirements.txt from root-level files")

# Step 2: Get Python version
python_version = subprocess.check_output(['python3', '--version']).decode().strip()

# Step 3: Add Python version as a comment to the top of requirements.txt
requirements_file = 'requirements.txt'
if os.path.exists(requirements_file):
    with open(requirements_file, 'r') as file:
        content = file.readlines()
    content.insert(0, f'# {python_version}\n')
    with open(requirements_file, 'w') as file:
        file.writelines(content)
else:
    print(f"{requirements_file} not found.")
