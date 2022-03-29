Skycharge library and set of services and tools
===============================================

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

This official Skycharge repository includes the library and a set of
daemons and command line tools for interaction with the Skycharge
hardware.

At the current moment the library is a collection of C sources and
headers that are linked directly with Skycharge applications. The
library itself is not represented as a shared object, which can be
changed in the future.

The main Skycharge tools and services are: skycharge-cli, skycharged,
skyhttpd, skyuartd.

### skycharge-cli

Command line tool which interacts with the Skycharge hardware either
locally through the Skycharge library, either remotely, sending
requests over TCP/IP to the 'skycharged' service.

### skycharged

Service which runs on the BeagleBone Board, responds to remote
requests from command line tool and interacts with the Skycharge
hardware through the Skycharge library.

### skyhttpd

HTTP service, which runs on the BeagleBone Board and acts as a HTTP
gateway to the Skycharge hardware.

### skyuartd

UART service, which runs on the BeagleBone Board and acts as a UART
gateway to the Skycharge hardware.

## SDK documentation

https://support.skycharge.de/docs/sdk

## Building from sources (Debian Linux)

You'll need to install several dependencies:

```bash
sudo apt update
sudo apt install libelf-dev uuid-dev libserialport-dev libgps-dev libzmq3-dev
sudo apt install libczmq-dev libavahi-client-dev libmicrohttpd-dev libmongoc-dev
```

From here it is simple:

```bash
# To build everything
make

# Build just the CLI
make skycharge-cli
```

The executables will be available in the root of the repository.
