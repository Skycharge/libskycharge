#!/bin/bash

# Build script for ARM/BeagleBone compilation using Docker
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/builds"

# Function to clean build artifacts
clean_artifacts() {
    echo "Cleaning build artifacts..."
    
    # Clean using original Makefile
    docker-compose run --rm build-arm make distclean || true
    
    # Clean Docker builds directory
    rm -rf "$BUILD_DIR"
    
    # Clean debian packaging artifacts in Docker container (NEVER touch changelog!)
    docker-compose run --rm build-arm bash -c "
        rm -f debian/files debian/*.debhelper.log debian/*.debhelper debian/*.substvars
        rm -f ../*.deb ../*.changes ../*.buildinfo ../*.dsc ../*.tar.gz ../*.tar.xz
    " || true
    
    echo "Cleanup completed."
}

# Parse command line arguments
BUILD_TYPE="binaries"
PACKAGE_VERSION=""
CLEAN_FIRST=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --debian|--deb)
            BUILD_TYPE="debian"
            shift
            ;;
        --version)
            PACKAGE_VERSION="$2"
            shift 2
            ;;
        --clean)
            CLEAN_FIRST=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [--debian|--deb] [--version VERSION] [--clean]"
            echo ""
            echo "Options:"
            echo "  --debian, --deb    Build debian packages instead of just binaries"
            echo "  --version VERSION  Set custom version for debian package"
            echo "  --clean            Clean build artifacts before building"
            echo "  --help, -h         Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                           # Build binaries only"
            echo "  $0 --debian                  # Build debian packages"
            echo "  $0 --debian --version 2.2.3  # Build debian packages with custom version"
            echo "  $0 --clean --debian          # Clean and build debian packages"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "Building libskycharge for ARM/BeagleBone..."
echo "Project directory: $PROJECT_DIR"
echo "Build directory: $BUILD_DIR"
echo "Build type: $BUILD_TYPE"

# Clean if requested
if [[ "$CLEAN_FIRST" == "true" ]]; then
    clean_artifacts
fi

# Create builds directory if it doesn't exist
mkdir -p "$BUILD_DIR"

# Build the Docker image and run compilation
cd "$SCRIPT_DIR"

if [[ "$BUILD_TYPE" == "debian" ]]; then
    echo "Starting Docker debian package build..."
    docker-compose run --rm build-arm bash -c "
        echo 'Starting debian package build...'
        echo 'Current directory: \$(pwd)'
        echo 'Current version from changelog:'
        head -1 debian/changelog
        
        # Update version if specified (backup original first)
        if [[ -n '$PACKAGE_VERSION' ]]; then
            echo 'Backing up original changelog...'
            cp debian/changelog debian/changelog.backup
            echo 'Updating package version to $PACKAGE_VERSION...'
            sed -i '1s/([^)]*)/(${PACKAGE_VERSION})/' debian/changelog
            echo 'New version:'
            head -1 debian/changelog
        fi
        
        # Clean previous builds
        echo 'Cleaning previous builds...'
        make distclean || true
        
        # Generate version.h from changelog (as per original build system)
        echo 'Generating version.h...'
        make version.h
        
        # Build debian packages using original debian/rules
        echo 'Building debian packages...'
        dpkg-buildpackage -us -uc -b --host-arch=armhf
        
        # Restore original changelog if we modified it
        if [[ -n '$PACKAGE_VERSION' ]]; then
            echo 'Restoring original changelog...'
            mv debian/changelog.backup debian/changelog
        fi
        
        # Copy packages to builds directory
        echo 'Copying debian packages to builds directory...'
        mkdir -p /builds/debian
        cp ../*.deb /builds/debian/ 2>/dev/null || echo 'No .deb files found'
        cp ../*.changes /builds/debian/ 2>/dev/null || echo 'No .changes files found'
        cp ../*.buildinfo /builds/debian/ 2>/dev/null || echo 'No .buildinfo files found'
        
        # Show what was built
        echo 'Built debian packages:'
        ls -la /builds/debian/
        
        echo 'Debian package build completed successfully!'
    "
    
    echo ""
    echo "Debian package build completed! Packages are available in: $BUILD_DIR/debian/"
    echo "Install on BeagleBone with: dpkg -i *.deb"
    
else
    echo "Starting Docker binary build..."
    docker-compose run --rm build-arm bash -c "
        echo 'Starting binary build...'
        echo 'Current directory: \$(pwd)'
        echo 'Current version from changelog:'
        head -1 debian/changelog
        
        # Clean previous builds
        echo 'Cleaning previous builds...'
        make distclean || true
        
        # Generate version.h (as per original build system)
        echo 'Generating version.h...'
        make version.h
        
        # Build the project using original Makefile
        echo 'Building project...'
        make -j\$(nproc)
        
        # Copy binaries to builds directory
        echo 'Copying binaries to builds directory...'
        mkdir -p /builds/binaries
        cp skyuartd skycharged skybmsd skypsu skyhttpd skycharge-cli /builds/binaries/ 2>/dev/null || echo 'Some binaries may not have been built'
        
        # Copy configuration files
        echo 'Copying configuration files...'
        cp skycharge.conf /builds/binaries/ 2>/dev/null || true
        cp skyuart-config /builds/binaries/ 2>/dev/null || true
        
        # Show version info
        echo 'Version information:'
        cat version.h | grep SKY_VERSION
        
        # Show what was built
        echo 'Built binaries:'
        ls -la /builds/binaries/
        
        echo 'Binary build completed successfully!'
    "
    
    echo ""
    echo "Binary build completed! Files are available in: $BUILD_DIR/binaries/"
    echo "You can now copy these binaries to your BeagleBone device."
fi