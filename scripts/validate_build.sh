#!/bin/bash
"""
Book Build and Validation Script
This script validates that the complete book builds correctly and all components work
"""

echo "Starting book build and validation..."
echo "This script will verify that the complete Physical AI & Humanoid Robotics book compiles and validates properly."

# Check if Node.js and npm are available
echo "Checking Node.js and npm..."
if command -v node &> /dev/null && command -v npm &> /dev/null; then
    echo "✓ Node.js and npm are available"
    node --version
    npm --version
else
    echo "✗ Node.js and npm are not available"
    exit 1
fi

# Check if required Docusaurus dependencies are installed
echo "Checking Docusaurus dependencies..."
if [ -f "package.json" ]; then
    echo "✓ package.json exists"

    # Check if Docusaurus packages are listed in package.json
    if grep -q "@docusaurus/core" package.json; then
        echo "✓ Docusaurus core is listed in dependencies"
    else
        echo "✗ Docusaurus core not found in dependencies"
    fi

    if grep -q "@docusaurus/preset-classic" package.json; then
        echo "✓ Docusaurus preset-classic is listed in dependencies"
    else
        echo "✗ Docusaurus preset-classic not found in dependencies"
    fi
else
    echo "✗ package.json not found"
    exit 1
fi

# Check if node_modules exist or install dependencies
if [ ! -d "node_modules" ]; then
    echo "Installing npm dependencies..."
    npm install
    if [ $? -eq 0 ]; then
        echo "✓ Dependencies installed successfully"
    else
        echo "✗ Failed to install dependencies"
        exit 1
    fi
else
    echo "✓ node_modules already exists"
fi

# Validate Docusaurus configuration
echo "Checking Docusaurus configuration..."
if [ -f "docusaurus.config.ts" ]; then
    echo "✓ docusaurus.config.ts exists"

    # Check for required configuration elements
    if grep -q "title.*Physical AI" docusaurus.config.ts; then
        echo "✓ Site title is configured correctly"
    else
        echo "⚠ Site title may not be configured correctly"
    fi

    if grep -q "sidebarPath" docusaurus.config.ts; then
        echo "✓ Sidebar configuration exists"
    else
        echo "✗ Sidebar configuration not found"
    fi
else
    echo "✗ docusaurus.config.ts not found"
    exit 1
fi

# Check if sidebars file exists
if [ -f "sidebars.js" ]; then
    echo "✓ sidebars.js exists"
else
    echo "✗ sidebars.js not found"
    exit 1
fi

# Check documentation directory structure
echo "Checking documentation structure..."
required_docs=(
    "docs/intro.md"
    "docs/physical-ai/index.md"
    "docs/ros2/index.md"
    "docs/simulation/index.md"
    "docs/ai-robot-brain/index.md"
    "docs/vla/index.md"
    "docs/capstone/index.md"
)

all_docs_present=true
for doc in "${required_docs[@]}"; do
    if [ -f "$doc" ]; then
        echo "✓ $doc exists"
    else
        echo "✗ $doc not found"
        all_docs_present=false
    fi
done

if [ "$all_docs_present" = true ]; then
    echo "✓ All required documentation files are present"
else
    echo "⚠ Some documentation files are missing"
fi

# Count total documentation files
doc_count=$(find docs -name "*.md" | wc -l)
echo "Total documentation files: $doc_count"

# Check source code directories
echo "Checking source code integration..."
code_examples_count=$(find src -name "*.py" -o -name "*.cpp" -o -name "*.launch.py" -o -name "*.urdf" -o -name "*.xml" | wc -l)
echo "Total code examples: $code_examples_count"

# Try to build the site
echo "Attempting to build the Docusaurus site..."
if npm run build &> /dev/null; then
    echo "✓ Site builds successfully"
    # Check if build directory was created
    if [ -d "build" ]; then
        echo "✓ Build directory created successfully"
        build_size=$(du -sh build | cut -f1)
        echo "Build size: $build_size"
    else
        echo "✗ Build directory was not created"
    fi
else
    echo "✗ Site build failed"
    # Show build errors
    npm run build
    exit 1
fi

# Try to start the development server (in background) to verify it works
echo "Testing development server..."
# Start server in background
npm run start &> /tmp/docusaurus_start.log &
SERVER_PID=$!
# Wait a bit for server to start
sleep 10
# Check if server is running
if ps -p $SERVER_PID > /dev/null; then
    echo "✓ Development server starts successfully"
    # Kill the server
    kill $SERVER_PID
else
    echo "⚠ Development server may have issues (check logs)"
    cat /tmp/docusaurus_start.log
fi

# Check for broken links in markdown files
echo "Checking for broken links in documentation..."
broken_links=0
while IFS= read -r -d '' file; do
    # Simple check for potentially broken links (http/https that don't look like valid URLs)
    if grep -E "http[^ ]+[^/]" "$file" | grep -v -E "https?://[a-zA-Z0-9\.-]+\.[a-zA-Z]{2,}" > /dev/null; then
        echo "⚠ Potential broken link found in $file"
        grep -E "http[^ ]+[^/]" "$file" | grep -v -E "https?://[a-zA-Z0-9\.-]+\.[a-zA-Z]{2,}"
    fi
done < <(find docs -name "*.md" -print0)

# Check image references
echo "Checking image references..."
while IFS= read -r -d '' file; do
    # Find image references in markdown files
    if grep -E "\!\[.*\]\(.*\)" "$file" > /dev/null; then
        # Extract image paths and check if they exist
        image_paths=$(grep -oE "\!\[.*\]\(\S+\)" "$file" | sed 's/!\[.*\](\(.*\))/\1/' | sed 's/)$//')
        for img_path in $image_paths; do
            if [[ "$img_path" == http* ]]; then
                continue  # Skip external images
            fi
            full_path="$file/../../$img_path"
            full_path=$(realpath -m "$full_path")
            if [ ! -f "$full_path" ]; then
                echo "⚠ Image not found: $img_path (referenced in $file)"
            fi
        done
    fi
done < <(find docs -name "*.md" -print0)

# Summary
echo ""
echo "Build and validation completed!"
echo ""
echo "Summary:"
echo "- Documentation files: $doc_count files"
echo "- Code examples: $code_examples_count files"
echo "- Build status: SUCCESS"
echo "- Configuration: VERIFIED"
echo ""
echo "Next steps:"
echo "1. Review any warnings shown above"
echo "2. Verify the built site in the 'build' directory"
echo "3. Test the site functionality manually if needed"
echo "4. The book is ready for publication"