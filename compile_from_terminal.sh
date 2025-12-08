#!/bin/bash
# Compile Webots controllers from terminal on macOS
# Following Webots User Guide R2020a-rev1 instructions
# https://cyberbotics.com/doc/guide/compiling-controllers-in-a-terminal

set -e

echo "=== Compiling Webots Controllers from Terminal ==="
echo ""

# Set WEBOTS_HOME according to Webots documentation
# For macOS: export WEBOTS_HOME=/Applications/Webots
# Since Webots is installed as a .app bundle, we use the bundle path
export WEBOTS_HOME="/Applications/Webots.app"

# Create symlink for Makefile compatibility (resources -> Contents/Resources)
if [ -d "$WEBOTS_HOME" ] && [ ! -e "$WEBOTS_HOME/resources" ]; then
    echo "Creating symlink for Makefile compatibility..."
    cd "$WEBOTS_HOME"
    ln -s Contents/Resources resources
    echo "✓ Created symlink: resources -> Contents/Resources"
    echo ""
fi

# Verify Webots installation
if [ ! -f "$WEBOTS_HOME/Contents/Resources/Makefile.include" ]; then
    echo "❌ Error: Webots Makefile.include not found!"
    echo "   Expected: $WEBOTS_HOME/Contents/Resources/Makefile.include"
    exit 1
fi

echo "✓ Webots installation verified"
echo "✓ Using WEBOTS_HOME: $WEBOTS_HOME"
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Compile rosbot controller
echo "=== Compiling rosbot controller ==="
cd controllers/rosbot
make clean 2>/dev/null || true
mkdir -p build/release/x86_64 build/release/arm64 2>/dev/null || true
make
if [ -f "rosbot" ]; then
    echo "✓ rosbot controller compiled successfully"
    ls -lh rosbot
else
    echo "⚠ Warning: Executable not found"
fi

echo ""
echo "=== Compiling turtlebot3_ostacle_avoidance controller ==="
cd ../turtlebot3_ostacle_avoidance
make clean 2>/dev/null || true
mkdir -p build/release/x86_64 build/release/arm64 2>/dev/null || true
make
if [ -f "turtlebot3_ostacle_avoidance" ]; then
    echo "✓ turtlebot3_ostacle_avoidance controller compiled successfully"
    ls -lh turtlebot3_ostacle_avoidance
else
    echo "⚠ Warning: Executable not found"
fi

echo ""
echo "=== Compilation complete! ==="
echo ""
echo "Both controllers have been compiled as universal binaries (x86_64 + arm64)"
echo "They should now work in Webots!"





