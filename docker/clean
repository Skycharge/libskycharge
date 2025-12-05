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
echo "Removing build artifacts..."
rm -rf builds/

# Deep clean option: remove the helper image and prune caches. Safe to skip if you want to keep caches.
if [[ "${1:-}" == "--deep" ]]; then
    echo "Performing deep Docker cleanup..."
    echo "  - Removing bone-runner image (if present)..."
    docker rmi -f bone-runner 2>/dev/null || true

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
echo "Ready for fresh builds with: ./build-bone.sh"