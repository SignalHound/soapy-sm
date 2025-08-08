<p align="center">
<img src="https://signalhound.com/sigdownloads/Other/SH-SOAPY.jpg" width="75%" />
</p>

## A [SoapySDR](https://github.com/pothosware/SoapySDR/wiki) driver for the [Signal Hound SM200 series 20 GHz Real-Time Spectrum Analyzers](https://signalhound.com/products/sm200b-20-ghz-real-time-spectrum-analyzer/) and [Signal Hound SM435 43.5 GHz Real-Time Spectrum Analyzers](https://signalhound.com/products/sm435b-43-5-ghz-real-time-spectrum-analyzer/)
### Requirements

- 64-bit Linux operating system
    - Tested on DragonOS Noble 6.11.0-29-generic
- Native USB 3.0 support

### Prerequisites

1. [Install SoapySDR](https://github.com/pothosware/PothosCore/wiki/Ubuntu).
    - Note: Python bindings are not needed for this driver.
    - Check installation with `SoapySDRUtil --info`.
2. [Install the Signal Hound SDK](https://signalhound.com/software/signal-hound-software-development-kit-sdk/).
    - Follow directions in _device_apis/bb_series/linux/README.txt_.

### Installation

1. Clone this repository.
2. Run the following commands from the cloned repo directory of the repository:

```
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
$ sudo ldconfig
```
Now if a SM200 or SM435 device is plugged in, `SoapySDRUtil --find` will display its serial number.

### Usage

- `#include <SoapySDR/Device.hpp>` and use the functions in [Device.hpp](https://github.com/pothosware/SoapySDR/blob/master/include/SoapySDR/Device.hpp) to interface with the SM200 or SM435.

- Use with [other platforms](https://github.com/pothosware/SoapySDR/wiki#platforms) that are compatible with SoapySDR such as [GNURadio](https://www.gnuradio.org/), [CubicSDR](https://cubicsdr.com/), and many others.
