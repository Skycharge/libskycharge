#!/bin/bash

# =============================================================================
# Docker Build Script for libskycharge ARM/BeagleBone Cross-Compilation
# =============================================================================
# This script provides a clean Docker-based build environment for compiling
# libskycharge for ARM/BeagleBone devices while preserving the original 
# build system and Makefile.
#
# Features:
# - ARM cross-compilation using Docker
# - Automatic Makefile patching (temporary, non-destructive)
# - Support for both binary and debian package builds
# - Original versioning system preserved (debian/changelog)
# - Complete cleanup capabilities
# =============================================================================

set -e

# Directory paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/builds"

# =============================================================================
# MAKEFILE PATCH SYSTEM
# =============================================================================
# This patch fixes a Make implicit rule issue where Make tries to create
# standalone executables from parser object files (*-cmd.tab.o, *-cmd.lex.o).
# These are library components, not complete programs, so linking them alone
# fails with "undefined reference to main" and missing parser symbols.

apply_makefile_patch() {
    echo "Applying temporary Makefile patch for Docker builds..."
    
    # Check if patch is already applied
    if grep -q "prevents Make's implicit rule" Makefile 2>/dev/null; then
        echo "  ALREADY APPLIED: Patch already applied"
        return 0
    fi
    
    # Apply the patch by appending to Makefile
    cat >> Makefile << 'EOF'

# =============================================================================
# DOCKER BUILD PATCH (Temporary - Auto-removed after build)
# =============================================================================
# This patch prevents Make's implicit rules from trying to create standalone
# executables from parser object files. Parser components (*-cmd.tab.o and 
# *-cmd.lex.o) are meant to be linked INTO main programs, not used alone.
#
# Without this patch, Make tries: gcc skyuartd-cmd.tab.o -o skyuartd-cmd.tab
# This fails because parser objects don't have main() and missing symbols.

%-cmd.tab: %-cmd.tab.o
	@# This rule prevents Make's implicit rule from trying to link this object into an executable

%-cmd.lex: %-cmd.lex.o
	@# This rule prevents Make's implicit rule from trying to link this object into an executable

EOF
    
    echo "  SUCCESS: Makefile patch applied successfully"
}

remove_makefile_patch() {
    echo "Removing temporary Makefile patch..."
    
    # Check if patch exists
    if ! grep -q "DOCKER BUILD PATCH" Makefile 2>/dev/null; then
        echo "  INFO: No patch found to remove"
        return 0
    fi
    
    # Find the patch start line and remove everything from there
    PATCH_LINE=$(grep -n "DOCKER BUILD PATCH" Makefile | cut -d: -f1 | head -1)
    if [ -n "$PATCH_LINE" ]; then
        # Keep everything before the patch
        head -n $((PATCH_LINE - 2)) Makefile > Makefile.tmp
        mv Makefile.tmp Makefile
        echo "  SUCCESS: Makefile patch removed successfully"
    else
        echo "  WARNING: Could not find patch marker, using git restore"
        git checkout -- Makefile 2>/dev/null || echo "  ERROR: Git restore failed"
    fi
}

# =============================================================================
# CLEANUP FUNCTIONS
# =============================================================================

clean_artifacts() {
    echo "Cleaning build artifacts..."
    
    # Clean build files using Make with patch applied
    echo "Cleaning compilation artifacts..."
    docker-compose run --rm build-arm bash -c "
        # Apply patch to prevent linking errors during clean
        $(declare -f apply_makefile_patch); apply_makefile_patch
        make distclean || true
        # Remove patch after clean
        $(declare -f remove_makefile_patch); remove_makefile_patch
    " 2>/dev/null || true
    
    # Remove Docker build output directory
    echo "Removing builds directory..."
    rm -rf "$BUILD_DIR"
    
    # Clean debian packaging artifacts (NEVER touch changelog!)
    echo "Cleaning debian packaging artifacts..."
    docker-compose run --rm build-arm bash -c "
        rm -f debian/files debian/*.debhelper.log debian/*.debhelper debian/*.substvars
        rm -f ../*.deb ../*.changes ../*.buildinfo ../*.dsc ../*.tar.gz ../*.tar.xz
    " 2>/dev/null || true
    
    echo "Cleanup completed"
}

# =============================================================================
# COMMAND LINE ARGUMENT PARSING
# =============================================================================

show_help() {
    cat << EOF
libskycharge Docker Build System

USAGE:
    $0 [OPTIONS]

OPTIONS:
    --debian, --deb         Build debian packages instead of binaries
    --version VERSION       Set custom version (temporarily modifies changelog)
    --clean                 Clean all build artifacts before building
    --help, -h             Show this help message

EXAMPLES:
    $0                              # Build ARM binaries
    $0 --debian                     # Build debian packages  
    $0 --debian --version 2.2.3     # Build packages with custom version
    $0 --clean --debian             # Clean and build packages

OUTPUT:
    Binaries:        ../builds/binaries/
    Debian packages: ../builds/debian/

NOTES:
    - Uses original build system (preserves Makefile and debian/changelog)
    - Custom versions only affect the build (original changelog restored)
    - All builds target ARM/BeagleBone (linux/arm/v7, armhf architecture)
EOF
}

# Default values
BUILD_TYPE="binaries"
PACKAGE_VERSION=""
CLEAN_FIRST=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --debian|--deb)     BUILD_TYPE="debian"; shift ;;
        --version)          PACKAGE_VERSION="$2"; shift 2 ;;
        --clean)            CLEAN_FIRST=true; shift ;;
        --help|-h)          show_help; exit 0 ;;
        *)                  echo "ERROR: Unknown option: $1"; echo "Use --help for usage"; exit 1 ;;
    esac
done

# =============================================================================
# MAIN BUILD PROCESS
# =============================================================================

echo "Building libskycharge for ARM/BeagleBone"
echo "   Project: $PROJECT_DIR"
echo "   Output:  $BUILD_DIR"
echo "   Type:    $BUILD_TYPE"
echo ""

# Clean if requested
if [[ "$CLEAN_FIRST" == "true" ]]; then
    clean_artifacts
    echo ""
fi

# Ensure build output directory exists
mkdir -p "$BUILD_DIR"

# Change to docker directory for docker-compose
cd "$SCRIPT_DIR"

# =============================================================================
# BUILD FUNCTIONS
# =============================================================================

build_debian_packages() {
    docker-compose run --rm build-arm bash -c "
        echo 'Starting debian package build'
        echo 'Working directory: \$(pwd)'
        echo 'Current version:'
        head -1 debian/changelog
        echo ''
        
        # Apply Makefile patch to prevent linking errors
        $(declare -f apply_makefile_patch); apply_makefile_patch
        
        # Handle custom version if specified
        if [[ -n '$PACKAGE_VERSION' ]]; then
            echo 'Setting custom version: $PACKAGE_VERSION'
            cp debian/changelog debian/changelog.backup
            sed -i '1s/([^)]*)/(${PACKAGE_VERSION})/' debian/changelog
            echo 'SUCCESS: Version updated:'
            head -1 debian/changelog
            echo ''
        fi
        
        # Clean and build
        echo 'Cleaning previous builds...'
        make distclean || true
        
        echo 'Generating version.h from changelog...'
        make version.h
        
        echo 'Building debian packages with parallel compilation...'
        DEB_BUILD_OPTIONS=\"parallel=\$(nproc)\" dpkg-buildpackage -us -uc -b --host-arch=armhf
        
        # Restore original changelog if modified
        if [[ -n '$PACKAGE_VERSION' ]]; then
            echo 'Restoring original changelog...'
            mv debian/changelog.backup debian/changelog
        fi
        
        # Copy output to builds directory
        echo 'Copying packages to builds directory...'
        mkdir -p /builds/debian
        cp ../*.deb /builds/debian/ 2>/dev/null || echo 'WARNING: No .deb files found'
        cp ../*.changes /builds/debian/ 2>/dev/null || echo 'WARNING: No .changes files found'
        cp ../*.buildinfo /builds/debian/ 2>/dev/null || echo 'WARNING: No .buildinfo files found'
        
        echo 'Built packages:'
        ls -la /builds/debian/ | grep -E '\.(deb|changes|buildinfo)$' || echo 'No packages found'
        
        # Clean up Makefile patch
        $(declare -f remove_makefile_patch); remove_makefile_patch
        
        echo 'Debian package build completed!'
    "
    
    echo ""
    echo "Debian packages ready!"
    echo "   Location: $BUILD_DIR/debian/"
    echo "   Copy to BB:  scp ../builds/debian/*.deb root@skydevice.local:updates"
}

build_binaries() {
    docker-compose run --rm build-arm bash -c "
        echo 'Starting binary build'
        echo 'Working directory: \$(pwd)'
        echo 'Current version:'
        head -1 debian/changelog
        echo ''
        
        # Apply Makefile patch to prevent linking errors
        $(declare -f apply_makefile_patch); apply_makefile_patch
        
        # Clean and build
        echo 'Cleaning previous builds...'
        make distclean || true
        
        echo 'Generating version.h from changelog...'
        make version.h
        
        echo 'Building binaries with parallel compilation...'
        make -j\\\$(nproc)
        
        # Copy output to builds directory
        echo 'Copying binaries to builds directory...'
        mkdir -p /builds/binaries
        
        # Copy main binaries
        for binary in skyuartd skycharged skybmsd skypsu skyhttpd skycharge-cli; do
            if [ -f \"\$binary\" ]; then
                cp \"\$binary\" /builds/binaries/
                echo \"  COPIED: \$binary\"
            else
                echo \"  WARNING: \$binary not found\"
            fi
        done
        
        # Copy configuration files
        echo 'Copying configuration files...'
        cp skycharge.conf /builds/binaries/ 2>/dev/null && echo '  COPIED: skycharge.conf' || echo '  WARNING: skycharge.conf not found'
        cp skyuart-config /builds/binaries/ 2>/dev/null && echo '  COPIED: skyuart-config' || echo '  WARNING: skyuart-config not found'
        
        # Show version information
        echo 'Version information:'
        grep SKY_VERSION version.h | head -2
        
        echo 'Built files:'
        ls -la /builds/binaries/
        
        # Clean up Makefile patch
        $(declare -f remove_makefile_patch); remove_makefile_patch
        
        echo 'Binary build completed!'
    "
    
    echo ""
    echo "Binaries ready!"
    echo "   Location: $BUILD_DIR/binaries/"
    echo "   Deploy:   scp $BUILD_DIR/binaries/* root@skydevice.local:/usr/local/bin/"
}


# =============================================================================
# MAIN EXECUTION
# =============================================================================

if [[ "$BUILD_TYPE" == "debian" ]]; then
    echo "Building debian packages..."
    build_debian_packages
else
    echo "Building binaries..."
    build_binaries
fi