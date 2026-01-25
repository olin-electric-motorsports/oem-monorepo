#!/bin/bash

# Define the root of the repo
REPO_ROOT=$(git rev-parse --show-toplevel)
HOOKS_DIR="$REPO_ROOT/.git/hooks"

echo "Installing Git Hooks (KiCad + Git LFS)..."

install_git_lfs(){
    if ! command -v git-lfs >/dev/null 2>&1; then
        
        if [[ "$OSTYPE" == "linux-gnu"* ]]; then
            # Debian/Ubuntu
            if command -v apt-get >/dev/null 2>&1; then
                sudo apt-get update && sudo apt-get install -y git-lfs
            # Fedora/RHEL
            elif command -v dnf >/dev/null 2>&1; then
                sudo dnf install -y git-lfs
            # Arch Linux
            elif command -v pacman >/dev/null 2>&1; then
                sudo pacman -S --noconfirm git-lfs
            else
                echo "Error: Package manager not found."
                exit 1
            fi
        elif [[ "$OSTYPE" == "darwin"* ]]; then
            # macOS
            if command -v brew >/dev/null 2>&1; then
                brew install git-lfs
            else
                echo "Homebrew not found. Please install git-lfs manually."
                exit 1
            fi
        else
            echo "Unsupported OS type: $OSTYPE"
            exit 1
        fi
        
        # Initialize LFS after install
        git lfs install
        echo "Git LFS installed."
    else
        echo "why you got Git LFS already???"
    fi
}

# Run the function immediately
install_git_lfs

# This hook needs to:
# A. Implode the KiCad symbols
# B. Fetch LFS files (so 3D models appear)
cat > "$HOOKS_DIR/post-checkout" <<EOF
#!/bin/bash
# Implode symbols 
bazel run //tools/symbols:convert -- symbols2library --source "$REPO_ROOT/parts/schematic/oem" --out "$REPO_ROOT/parts/schematic/oem.kicad_sym"

# Fetch large files
if command -v git-lfs >/dev/null 2>&1; then
    git lfs post-checkout "\$@"
fi
EOF
chmod +x "$HOOKS_DIR/post-checkout"


# Same logic as post-checkout
cat > "$HOOKS_DIR/post-merge" <<EOF
#!/bin/bash
# Implode symbols
bazel run //tools/symbols:convert -- symbols2library --source "$REPO_ROOT/parts/schematic/oem" --out "$REPO_ROOT/parts/schematic/oem.kicad_sym"

# Fetch large files
if command -v git-lfs >/dev/null 2>&1; then
    git lfs post-merge "\$@"
fi
EOF
chmod +x "$HOOKS_DIR/post-merge"


# This explodes the library so we track small files
cat > "$HOOKS_DIR/pre-commit" <<EOF
#!/bin/bash
# Explode symbols
bazel run //tools/symbols:convert -- library2symbols --source "$REPO_ROOT/parts/schematic/oem.kicad_sym" --out "$REPO_ROOT/parts/schematic/oem"

# Add the exploded 
git add "$REPO_ROOT/parts/schematic/oem"
EOF
chmod +x "$HOOKS_DIR/pre-commit"


# Ensures the single file is rebuilt locally so you can keep working
cat > "$HOOKS_DIR/post-commit" <<EOF
#!/bin/bash
# Implode symbols
bazel run //tools/symbols:convert -- symbols2library --source "$REPO_ROOT/parts/schematic/oem" --out "$REPO_ROOT/parts/schematic/oem.kicad_sym"
EOF
chmod +x "$HOOKS_DIR/post-commit"


# LFS needs this to upload large files
cat > "$HOOKS_DIR/pre-push" <<EOF
#!/bin/bash
if command -v git-lfs >/dev/null 2>&1; then
    git lfs pre-push "\$@"
fi
EOF
chmod +x "$HOOKS_DIR/pre-push"


echo "Hooks installed."