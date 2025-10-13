#!/bin/bash

# Cleanup script for libskycharge Docker builds
# This script cleans all build artifacts without modifying the original Makefile

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "Cleaning libskycharge build artifacts..."
echo "Project directory: $PROJECT_DIR"

cd "$PROJECT_DIR"

# Clean using original Makefile in Docker container
echo "Cleaning with original Makefile..."
docker-compose -f docker/docker-compose.yml run --rm build-arm make distclean || true

# Clean Docker builds directory
echo "Removing builds directory..."
rm -rf builds/

# Clean debian packaging artifacts in Docker container
echo "Cleaning debian packaging artifacts..."
docker-compose -f docker/docker-compose.yml run --rm build-arm bash -c "
    rm -f debian/files debian/*.debhelper.log debian/*.debhelper debian/*.substvars
    rm -f ../*.deb ../*.changes ../*.buildinfo ../*.dsc ../*.tar.gz ../*.tar.xz
" || true

# Clean Docker volumes and containers (optional)
read -p "Also clean Docker volumes and containers? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Cleaning Docker volumes and containers..."
    docker-compose -f docker/docker-compose.yml down -v --remove-orphans || true
    docker system prune -f || true
fi

echo "Cleanup completed successfully!"