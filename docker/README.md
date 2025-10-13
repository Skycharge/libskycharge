# Docker Build Environment for libskycharge

**ARM cross-compilation for BeagleBone devices**

This directory provides a Docker-based build system for cross-compiling libskycharge to ARM architecture while preserving the original build system and Makefile.

## Files

| File | Purpose |
|------|---------|
| `build.sh` | Main build script |
| `clean.sh` | Docker environment script |
| `docker-compose.yml` | Docker service configuration |
| `Dockerfile.arm` | ARM cross-compilation environment |
| `README.md` | This documentation |

## Quick Start

## Usage

### Quick Binary Build

Run the automated build script for binaries only:

```bash
cd docker
./build.sh
```

This will:
1. Build the Docker image with all dependencies
2. Use the original Makefile to compile for ARM/BeagleBone
3. Generate `version.h` from `debian/changelog` (as per original build system)
4. Copy binaries to `../builds/binaries/` directory

### Quick Debian Package Build

Build debian packages for easy installation:

```bash
cd docker
./build.sh --debian
```

Or with custom version (temporarily modifies `debian/changelog` during build):

```bash
cd docker
./build.sh --debian --version 2.2.3-custom
```

**Note**: Custom version only affects the build - original `debian/changelog` is restored after build.

This will:
1. Build the Docker image with all dependencies
2. Use the original `debian/rules` and `dpkg-buildpackage` system
3. Generate version from `debian/changelog` (preserves original versioning)
4. Create debian packages (.deb files) for ARM/BeagleBone
5. Copy packages to `../builds/debian/` directory

### Manual Build

If you prefer manual control:

#### Binary Build
```bash
cd docker

# Start the build container
docker-compose run --rm build-arm

# Inside the container:
make clean
make -j$(nproc)

# Copy binaries
mkdir -p /builds/binaries
cp skyuartd skycharged skybmsd skypsu skyhttpd skycharge-cli /builds/binaries/
```

#### Debian Package Build
```bash
cd docker

# Start the build container
docker-compose run --rm build-arm

# Inside the container:
make clean
debian/rules clean
dpkg-buildpackage -us -uc -b --host-arch=armhf

# Copy packages
mkdir -p /builds/debian
cp ../*.deb /builds/debian/
```

### Install on BeagleBone

#### Using Debian Packages (Recommended)

```bash
# Copy and install packages
scp ../builds/debian/*.deb root@beaglebone-ip:/tmp/
ssh root@beaglebone-ip "cd /tmp && dpkg -i *.deb"
```

#### Using Binaries

```bash
# Copy binaries manually
scp ../builds/binaries/skyuartd root@beaglebone-ip:/usr/local/bin/
scp ../builds/binaries/skycharged root@beaglebone-ip:/usr/local/bin/
scp ../builds/binaries/skycharge.conf root@beaglebone-ip:/etc/
# ... copy other files as needed
# install
scp ../builds/debian/*.deb root@skydevice.local:updates
```

## Build Output

### Binaries (./build.sh)
Compiled binaries will be available in `../builds/binaries/`:
- `skyuartd` - UART daemon (with your modifications)
- `skycharged` - Main charging daemon
- `skybmsd` - BMS daemon
- `skypsu` - PSU utility
- `skyhttpd` - HTTP daemon
- `skycharge-cli` - Command line interface
- `skycharge.conf` - Configuration file
- `skyuart-config` - UART configuration script

### Debian Packages (./build.sh --debian)
Debian packages will be available in `../builds/debian/`:
- `skycharge-conf_*.deb` - Configuration package
- `skycharged_*.deb` - Main charging daemon package
- `skybmsd_*.deb` - BMS daemon package (part of skycharged)
- `skyhttpd_*.deb` - HTTP daemon package
- `skyuartd_*.deb` - UART daemon package
- `skycharge-cli_*.deb` - Command line interface package
- `skybroker_*.deb` - Broker package (x86-64 only)

This Docker setup uses the original libskycharge build system:

### Versioning System
- **Source**: Version comes from first line of `debian/changelog`
- **Current**: `skycharge (2.2.2) stretch; urgency=low`
- **Processing**: Makefile extracts version and generates `version.h`
- **Generated**: `#define SKY_VERSION_STR "2.2.2"`

### Build Process
1. **Makefile**: Original Makefile handles all compilation
2. **version.h**: Auto-generated from changelog (not manually edited)
3. **Dependencies**: All original build dependencies preserved
4. **Debian**: Uses original `debian/rules` and packaging system

## Dependencies

The Docker image includes all necessary dependencies:
- ARM cross-compilation tools
- libserialport, libzmq, libczmq
- GPS, Avahi, microhttpd libraries
- Build tools (make, gcc, bison, flex)
- Debian packaging tools (debhelper, dpkg-dev)

## Platform

Builds are configured for:
- Platform: `linux/arm/v7`
- Architecture: `armhf`
- Target: BeagleBone

## Package Management on BeagleBone

### Installing Packages
```bash
# Install all packages
dpkg -i *.deb

# Install specific package
dpkg -i skyuartd_*.deb

# Fix dependencies if needed
apt-get install -f
```

### Managing Services
```bash
# Check service status
systemctl status skyuartd
systemctl status skycharged

# Start/stop services
systemctl start skyuartd
systemctl stop skyuartd

# Enable/disable auto-start
systemctl enable skyuartd
systemctl disable skyuartd

# View logs
journalctl -u skyuartd -f
```

### Updating Packages
```bash
# Remove old version
dpkg -r skyuartd

# Install new version
dpkg -i skyuartd_*.deb
```

## Cleanup

### Clean All Build Artifacts
```bash
# Clean compilation files (binaries, objects, generated files)
make clean

# Clean debian packaging artifacts  
make debian-clean

# Clean everything (includes docopt-gen)
make distclean
```

### Clean Docker Artifacts
```bash
cd docker

# Remove build volumes
docker-compose down -v

# Remove Docker images
docker-compose down --rmi all

# Clean up builds directory
rm -rf ../builds/
```

## Troubleshooting

### Build Issues
- Ensure Docker has enough resources (memory/disk)
- Check that all dependencies are available in debian repos
- Use `docker-compose run --rm build-arm bash` to debug interactively
- Try cleaning first: `make distclean` then rebuild

### Deployment Issues
- Verify target BeagleBone architecture matches (`uname -a`)
- Check dependencies: `dpkg -I package.deb` shows requirements
- Use `dpkg -l` to list installed packages

## Troubleshooting

### Build Fails
```bash
# Clean rebuild
./build.sh --clean

# Check Docker status
docker-compose ps
```

### Permission Issues
```bash
# Fix ownership
sudo chown -R $USER:$USER ../builds/
```

### Docker Issues
```bash
# Deep clean and rebuild
./clean.sh --deep
./build.sh
```

## Notes

- Original System Preserved: All changes are temporary during build
- Cross-Platform: Builds on macOS, Linux, Windows (with WSL)
- BeagleBone Ready: Outputs are compatible with BeagleBone Black ARM architecture
