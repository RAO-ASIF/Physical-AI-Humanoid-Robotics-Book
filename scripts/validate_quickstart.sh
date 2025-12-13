#!/bin/bash
"""
Quickstart Validation Script
This script validates that the quickstart guide works as expected
"""

echo "Starting quickstart validation..."
echo "This script will verify that all components of the Physical AI & Humanoid Robotics book are working properly."

# Check if ROS 2 is installed and accessible
echo "Checking ROS 2 installation..."
if command -v ros2 &> /dev/null; then
    echo "✓ ROS 2 is installed"
    ros2 --version
else
    echo "✗ ROS 2 is not installed or not in PATH"
    exit 1
fi

# Check Python version
echo "Checking Python version..."
python_version=$(python3 --version 2>&1 | grep -oP '\d+\.\d+' | head -1)
if (( $(echo "$python_version >= 3.10" | bc -l) )); then
    echo "✓ Python version is $python_version (meets requirement >= 3.10)"
else
    echo "✗ Python version is $python_version (requirement is >= 3.10)"
    exit 1
fi

# Check if required Python packages are installed
echo "Checking Python packages..."
required_packages=("rclpy" "numpy" "opencv-python" "openai")
for package in "${required_packages[@]}"; do
    if python3 -c "import $package" &> /dev/null; then
        echo "✓ $package is installed"
    else
        echo "✗ $package is not installed"
        # Try to install it
        pip3 install "$package" || echo "Could not install $package, please install manually"
    fi
done

# Check if workspace directory exists
echo "Checking ROS 2 workspace..."
if [ -d "$HOME/ros2_ws" ]; then
    echo "✓ ROS 2 workspace exists at $HOME/ros2_ws"
else
    echo "Creating ROS 2 workspace..."
    mkdir -p "$HOME/ros2_ws/src"
    echo "✓ Created ROS 2 workspace"
fi

# Try to build the workspace
echo "Building ROS 2 workspace..."
cd "$HOME/ros2_ws"
source /opt/ros/humble/setup.bash 2>/dev/null || echo "Could not source ROS 2 (this may be expected in validation)"
colcon build --packages-select robot_basics 2>/dev/null || echo "robot_basics package not found, this is expected if not created yet"

# Check if Docusaurus files exist
echo "Checking Docusaurus setup..."
if [ -f "package.json" ]; then
    echo "✓ package.json exists"
else
    echo "✗ package.json not found - this should exist in the project root"
fi

if [ -f "docusaurus.config.ts" ]; then
    echo "✓ docusaurus.config.ts exists"
else
    echo "✗ docusaurus.config.ts not found - this should exist in the project root"
fi

# Check if documentation files exist
echo "Checking documentation structure..."
docs_to_check=(
    "docs/intro.md"
    "docs/ros2/index.md"
    "docs/simulation/index.md"
    "docs/ai-robot-brain/index.md"
    "docs/vla/index.md"
    "docs/capstone/index.md"
)

for doc in "${docs_to_check[@]}"; do
    if [ -f "$doc" ]; then
        echo "✓ $doc exists"
    else
        echo "✗ $doc not found"
    fi
done

# Check if source code exists
echo "Checking source code structure..."
code_dirs=(
    "src/ros2-examples"
    "src/simulation-scenes"
    "src/ai-pipelines"
    "src/capstone-project"
)

for dir in "${code_dirs[@]}"; do
    if [ -d "$dir" ]; then
        echo "✓ $dir exists"
        # Count files in the directory
        file_count=$(find "$dir" -type f | wc -l)
        echo "  - Contains $file_count files"
    else
        echo "✗ $dir not found"
    fi
done

# Test basic ROS 2 functionality
echo "Testing basic ROS 2 functionality..."
if ros2 topic list &> /dev/null; then
    echo "✓ ROS 2 communication is working"
else
    echo "⚠ ROS 2 daemon may not be running - this is expected if no ROS 2 nodes are active"
fi

# Summary
echo ""
echo "Quickstart validation completed!"
echo "Please review the above checks to ensure all components are properly set up."
echo ""
echo "To complete the quickstart validation:"
echo "1. Ensure all checks marked with ✓ are successful"
echo "2. Address any issues marked with ✗ or ⚠"
echo "3. Run the simple publisher-subscriber example to verify ROS 2 communication"
echo "4. Start the Docusaurus server with 'npm start' to view the book"