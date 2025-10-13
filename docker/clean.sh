#!/bin/bash

# =============================================================================
# Docker Environment Cleanup Script for libskycharge
# =============================================================================
# This script provides comprehensive cleanup of Docker build artifacts,
# containers, and volumes. Use this when you want to completely reset
# the Docker build environment.
#
# Usage: ./clean.sh [--deep]
# 
# Options:
#   --deep    Also clean Docker volumes, containers, and images
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "Cleaning libskycharge Docker environment"
echo "   Project: $PROJECT_DIR"
echo ""

cd "$PROJECT_DIR"

# Use the build script's clean function
echo "Running build artifact cleanup..."
if [ -f "docker/build.sh" ]; then
    # Extract and run the clean function from build.sh
    bash docker/build.sh --clean >/dev/null 2>&1 || echo "Build script cleanup had issues (this is normal)"
else
    echo "build.sh not found, performing manual cleanup..."
    
    # Fallback manual cleanup
    docker-compose -f docker/docker-compose.yml run --rm build-arm make distclean 2>/dev/null || true
    rm -rf builds/
    docker-compose -f docker/docker-compose.yml run --rm build-arm bash -c "
        rm -f debian/files debian/*.debhelper.log debian/*.debhelper debian/*.substvars
        rm -f ../*.deb ../*.changes ../*.buildinfo ../*.dsc ../*.tar.gz ../*.tar.xz
    " 2>/dev/null || true
fi

# Deep clean option
if [[ "$1" == "--deep" ]]; then
    echo "Performing deep Docker cleanup..."
    
    echo "  - Stopping and removing containers..."
    docker-compose -f docker/docker-compose.yml down -v --remove-orphans 2>/dev/null || true
    
    echo "  - Removing Docker build cache..."
    docker system prune -f 2>/dev/null || true
    
    echo "  - Removing unused Docker images..."
    docker image prune -f 2>/dev/null || true
    
    echo "  ✓ Deep cleanup completed"
else
    echo ""
    echo "For complete Docker cleanup (containers, volumes, images):"
    echo "   ./clean.sh --deep"
fi

echo ""
echo "Cleanup completed successfully!"
echo "Ready for fresh builds with: ./build.sh"