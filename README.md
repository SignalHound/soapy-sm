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
2. SM API
- grab shared object file from [Signal Hound SDK](https://signalhound.com/software/signal-hound-software-development-kit-sdk/) 
    ~~~
    $ cd device_apis/sm_series/lib/linux/Ubuntu 18.04
    $ sudo cp libsm_api.* /usr/local/lib
    $ sudo ldconfig -v -n /usr/local/lib
    $ sudo ln -sf /usr/local/lib/libsm_api.so.5 /usr/local/lib/libsm_api.so
    ~~~
3. [SoapySDR](https://github.com/pothosware/PothosCore/wiki/Ubuntu).

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

Note: if a SM200B or SM435B device is plugged in, `SoapySDRUtil --find` will display its serial number.

## Usage

- `#include <SoapySDR/Device.hpp>` and use the functions in [Device.hpp](https://github.com/pothosware/SoapySDR/blob/master/include/SoapySDR/Device.hpp) to interface with the SM200B or SM435B.

- Use with [other platforms](https://github.com/pothosware/SoapySDR/wiki#platforms) that are compatible with SoapySDR such as [GNU Radio Companion](https://www.gnuradio.org/), [SDRangel](https://www.sdrangel.org/), and many others.