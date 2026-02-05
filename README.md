<p align="center">
<img src="https://signalhound.com/sigdownloads/Other/SH-SOAPY.jpg" width="75%" />
</p>

# A [SoapySDR](https://github.com/pothosware/SoapySDR/wiki) driver for the [Signal Hound SM200 series 20 GHz Real-Time Spectrum Analyzers](https://signalhound.com/products/sm200b-20-ghz-real-time-spectrum-analyzer/) and [Signal Hound SM435 43.5 GHz Real-Time Spectrum Analyzers](https://signalhound.com/products/sm435b-43-5-ghz-real-time-spectrum-analyzer/)
## System Requirements

- 64-bit Linux operating system
    - Tested on DragonOS Noble 6.14.0-27-generic
- Native USB 3.0 support

## Dependancies

1. Libusb 1.0 
    ~~~
    $ sudo apt-get update
    $ sudo apt-get install libusb-1.0-0
    ~~~
    Note: libusb requires root by default to change this add the following to etc/udev/rules.d/sh.rules
    or download [Spike](https://signalhound.com/spike) and the setup will be done for you
    ~~~
    SUBSYSTEM=="usb", ATTR{idVendor}=="2817", MODE="0666", GROUP="plugdev"
    ~~~
2. SM API
- grab shared object file from [Signal Hound SDK](https://signalhound.com/software/signal-hound-software-development-kit-sdk/) 
    ~~~
    $ cd device_apis/sm_series/lib/linux/Ubuntu 18.04
    $ sudo cp libsm_api.* /usr/local/lib
    $ sudo ldconfig -v -n /usr/local/lib
    $ sudo ln -sf /usr/local/lib/libsm_api.so.5 /usr/local/lib/libsm_api.so
    ~~~
3. [SoapySDR Dev Tools](https://github.com/pothosware/PothosCore/wiki/BuildGuide#ubuntu).
    $ sudo apt-get install libsoapysdr-dev

4. CMake
    $ sudo apt-get install cmake
5. G++
    $ sudo apt-get install g++
## Installation

1. Clone this repository.
2. Run the following commands from the cloned repo:
    ~~~
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ sudo make install
    $ sudo ldconfig
    ~~~
Note: MATLAB runtime LD_LIBRARY_PATH environment variable may cause conflicts with cmake

Note: if a SM200B or SM435B device is plugged in, `SoapySDRUtil --find` will display its serial number.

## Usage

- `#include <SoapySDR/Device.hpp>` and use the functions in [Device.hpp](https://github.com/pothosware/SoapySDR/blob/master/include/SoapySDR/Device.hpp) to interface with the SM family devices.

- When trying to connect to an SFP+ based device that does not use the default password you will need to pass in parameters for the hostAddr, deviceAddr, an port such as shown below with the Soapy

    SoapySDRUtil --find="hostAddr=192.168.2.2,deviceAddr=192.168.2.11,port=51665"

- Use with [other platforms](https://github.com/pothosware/SoapySDR/wiki#platforms) that are compatible with SoapySDR such as [GNU Radio Companion](https://www.gnuradio.org/), [SDRangel](https://www.sdrangel.org/), and many others.
