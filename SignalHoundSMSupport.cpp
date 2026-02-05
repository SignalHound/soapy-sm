// SoapySDR Include
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>

// Util
#include <cstdint>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>

// Signal Hound API
#include "sm_api.h"

// Defines
#define SM_DEFAULT_REF -20

static SmDeviceType SMStringToType(std::string typeString)
{
    SmDeviceType type = smDeviceTypeSM200A;
    if (typeString == "SM200A") {
        type = smDeviceTypeSM200A;    
    } else if (typeString == "SM200B") {
        type = smDeviceTypeSM200B;
    } else if (typeString == "SM200C") {
        type = smDeviceTypeSM200C;
    } else if (typeString == "SM435B") {
        type = smDeviceTypeSM435B;
    } else if (typeString == "SM435C") {
        type = smDeviceTypeSM435C;
    }
    return type;
}

static std::string SMTypeToString(SmDeviceType type)
{
    std::string typeString = "";

    if (type == smDeviceTypeSM200A) {
        typeString = "SM200A";    
    } else if (type == smDeviceTypeSM200B) {
        typeString = "SM200B";
    } else if (type == smDeviceTypeSM200C) {
        typeString = "SM200C";
    } else if (type == smDeviceTypeSM435B) {
        typeString = "SM435B";
    } else if (type == smDeviceTypeSM435C) {
        typeString = "SM435C";
    }
    return typeString;
}

/***********************************************************************
 * Device interface
 **********************************************************************/
class SignalHoundSM : public SoapySDR::Device {
private:
    // Variables used
    int deviceHandle, serial, decimation, numDevices;
    int serials[SM_MAX_DEVICES];
    SmDeviceType types[SM_MAX_DEVICES];
    SmDeviceType type;
    double sampleRate, centerFrequency, bandwidth, maxBW, attenLevel;
    SmStatus status;
    const double maxSmSfpSampleRate = 200e6;
    const double maxSmUsbSampleRate = 50e6;
    const float sampleToBwRatio = 0.8;
    std::string hostAddr;
    std::string deviceAddr;
    int port;
public:
    
    /*******************************************************************
     * Constructor and Destructor
     ******************************************************************/

    SignalHoundSM(const SoapySDR::Kwargs &args) 
    {
        // Defaults
        SmStatus status = smNoError;
        deviceHandle = -1;
        serial = 0;
        decimation = 1;
        numDevices = SM_MAX_DEVICES;
        type = smDeviceTypeSM200A;
        centerFrequency = 100e6;
        attenLevel = -1;
        hostAddr = SM200_ADDR_ANY;
        deviceAddr = SM200_DEFAULT_ADDR;
        port = SM200_DEFAULT_PORT;

        if(args.count("hostAddr") &&
           args.count("deviceAddr") &&
           args.count("port")) {
            hostAddr = args.at("hostAddr");
            deviceAddr = args.at("deviceAddr");
            port = std::stoi(args.at("port"));

            if((status = smOpenNetworkedDevice(&deviceHandle, hostAddr.c_str(), deviceAddr.c_str(), port)) != smNoError) {
                throw std::runtime_error("Unable to open IP SM device with HostAddr "
                                                + hostAddr
                                                + " DeviceAddr "
                                                + deviceAddr 
                                                + " Port "
                                                + args.at("port"));
            }
        } else if(args.count("serial")) {
            serial = std::stoull(args.at("serial"), nullptr, 10);

            // Open device 
            if((status = smOpenDeviceBySerial(&deviceHandle, serial)) != smNoError) {
                throw std::runtime_error("Unable to open USB SM device with S/N "
                                                + std::to_string(serial));
            }
        } else {
            // Try to open default IP based SM device first
            status = smOpenNetworkedDevice(&deviceHandle, hostAddr.c_str(), deviceAddr.c_str(), port);

            // If no IP device is found try to open a USB based device
            if(deviceHandle == -1) {
                status = smOpenDevice(&deviceHandle);
            }
        }

        if(deviceHandle == -1) {
            throw std::runtime_error("Unable to open SM device");
        }

        if(deviceHandle != -1) {
            status = smGetDeviceInfo(deviceHandle, &type, &serial);
            if(type == smDeviceTypeSM200A || type == smDeviceTypeSM200B || type == smDeviceTypeSM435B)
            {
                sampleRate = maxSmUsbSampleRate / decimation;
            } else {
                sampleRate = maxSmSfpSampleRate / decimation;
            }
            bandwidth = sampleRate * sampleToBwRatio;
            maxBW = bandwidth;
        }
    }

    ~SignalHoundSM(void)
    {
        smAbort(deviceHandle);
        smCloseDevice(deviceHandle);
    }

    /*******************************************************************
     * Identification API
     ******************************************************************/

    std::string getDriverKey(void) const
    {
        return "Signal Hound SM Series";
    }

    std::string getHardwareKey(void) const
    {
        std::string hardwareKey = "";
        if (type == smDeviceTypeSM200A) {
            hardwareKey = "Signal Hound SM200A";    
        } else if (type == smDeviceTypeSM200B) {
            hardwareKey = "Signal Hound SM200B";
        } else if (type == smDeviceTypeSM200C) {
            hardwareKey = "Signal Hound SM200C";
        } else if (type == smDeviceTypeSM435B) {
            hardwareKey = "Signal Hound SM435B";
        } else if (type == smDeviceTypeSM435C) {
            hardwareKey = "Signal Hound SM435C";
        } 
        return hardwareKey;
    }

    SoapySDR::Kwargs getHardwareInfo(void) const
    {
        // Get firmware info
        int firmmaj = 0;
        int firmmin = 0;
        int firmrev = 0;
        smGetFirmwareVersion(deviceHandle, &firmmaj, &firmmin, &firmrev);

        // Get diagnostics
        float temp, volt, curr;
        smGetDeviceDiagnostics(deviceHandle, &volt, &curr, &temp);

        // Place into in Kwargs dictionary
        SoapySDR::Kwargs args;

        args["serial"] = std::to_string(serial);
        args["api_version"] = smGetAPIVersion();
        args["firmware:major"] = std::to_string(firmmaj);
        args["firmware:minor"] = std::to_string(firmmin);
        args["firmware:revision"] = std::to_string(firmrev);
        args["temperature"] = std::to_string(temp);
        args["voltage"] = std::to_string(volt);
        args["current"] = std::to_string(curr);

        return args;
    }

    /*******************************************************************
     * Channels API
     ******************************************************************/

    size_t getNumChannels(const int direction) const
    {
        return (direction == SOAPY_SDR_RX) ? 1 : 0;
    }

    SoapySDR::Kwargs getChannelInfo(const int direction, const size_t channel) const
    {
        SoapySDR::Kwargs args;
        if (direction != SOAPY_SDR_RX or channel != 1) {
            return args;
        }
        if (type < 3) {
            args["channel"] = std::to_string(channel);
            args["dBm_MAX"] = "+20 dBm";
            args["impedance"] = "50 ohm";
            args["rf_range"] = "100kHz to 20GHz";
            args["max_sensitivty"] = "-20 dBm";
            args["instantaneous_bandwidth"] = "40 MHz";
        } else {
            args["channel"] = std::to_string(channel);
            args["dBm_MAX"] = "+20 dBm";
            args["impedance"] = "50 ohm";
            args["rf_range"] = "100kHz to 43.5GHz";
            args["max_sensitivty"] = "-20 dBm";
            args["instantaneous_bandwidth"] = "40 MHz";
        }
        return args;
    }

    /*******************************************************************
     * Stream API
     ******************************************************************/

    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const
    {
        std::vector<std::string> formats;
        formats.push_back(SOAPY_SDR_CF32);
        formats.push_back(SOAPY_SDR_CS16);
        return formats;
    }

    std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const
    {
        fullScale = 1.0;
        return SOAPY_SDR_CF32;
    }

    // SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const {}

    SoapySDR::Stream* setupStream(const int direction,
                                  const std::string &format,
                                  const std::vector<size_t> &channels = std::vector<size_t>(),
                                  const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) 
    {
        if(deviceHandle == -1)
        {
            if(type == smDeviceTypeSM435C || type == smDeviceTypeSM200C) {
                status = smOpenNetworkedDevice(&deviceHandle, hostAddr.c_str(), deviceAddr.c_str(), port);
            } else {
                status = smOpenDeviceBySerial(&deviceHandle, serial);
            }
        }
        // Check channel config
        if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0)) {
            throw std::runtime_error("setupStream invalid channel selection");
        }

        // Check format
        if (format == SOAPY_SDR_CF32) {
            SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32");
            smSetIQDataType(deviceHandle, smDataType32fc);
        } else if (format == SOAPY_SDR_CS16) {
            SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16");
            smSetIQDataType(deviceHandle, smDataType16sc);
        } else {
            throw std::runtime_error("setupStream: Invalid format '" + format
                            + "' -- Only CF32 and CS16 are supported by SignalHoundSM module.");
        }

        // Configure device defaults
        status = smSetIQCenterFreq(deviceHandle, centerFrequency);
        status = smSetIQBaseSampleRate(deviceHandle, smIQStreamSampleRateNative);
        status = smSetIQSampleRate(deviceHandle, decimation);
        status = smSetIQBandwidth(deviceHandle, smTrue, bandwidth);
        status = smSetAttenuator(deviceHandle, attenLevel);

        return (SoapySDR::Stream*) this;
    }

    void closeStream(SoapySDR::Stream *stream) {
        smCloseDevice(deviceHandle);
        deviceHandle = -1;
    }

    // size_t getStreamMTU(SoapySDR::Stream *stream) const{}

    int activateStream(SoapySDR::Stream* stream,
                       const int flags = 0,
                       const long long timeNs = 0,
                       const size_t numElems = 0) 
    {
        if (flags != 0) {
            return SOAPY_SDR_NOT_SUPPORTED;
        }

        status = smConfigure(deviceHandle, smModeIQStreaming);

        // Query the receiver IQ stream characteristics
        // Should match what we set earlier
        double actualSampleRate, actualBandwidth;
        smGetIQParameters(deviceHandle, &actualSampleRate, &actualBandwidth);
        SoapySDR_logf(SOAPY_SDR_INFO, "Actual Sample Rate = %lf Actual Bandwidth = %lf", actualSampleRate, actualBandwidth);
        return 0;
    }

    int deactivateStream(SoapySDR::Stream *stream, const int flags = 0, const long long timeNs = 0)
    {
        status = smAbort(deviceHandle);
        return status;
    }

    int readStream(SoapySDR::Stream *stream,
                   void * const *buffs,
                   const size_t numElems,
                   int &flags,
                   long long &timeNs,
                   const long timeoutUs = 100000)
    {
        // Start clock
        const auto start = std::chrono::high_resolution_clock::now();

        // Grab IQ data

        int sampleLoss = 0;
        int samplesRemaining = 0;
        int64_t nsSinceEpoch = 0;
        status = smGetIQ(deviceHandle, *buffs, numElems, 0, 0, &nsSinceEpoch, smFalse, &sampleLoss, &samplesRemaining);
        if (status > smNoError) {
            SoapySDR_logf(SOAPY_SDR_WARNING, "GetIQ: %s", smGetErrorString(status));
        }

        if (status < smNoError) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "GetIQ: %s", smGetErrorString(status));
            return SOAPY_SDR_STREAM_ERROR;
        }

        // Return time
        flags = 0;
        auto stop = std::chrono::high_resolution_clock::now();
        timeNs = std::chrono::duration_cast<std::chrono::nanoseconds>(stop-start).count();

        return numElems;
    }

    /*******************************************************************
     * Antenna API
     ******************************************************************/

    std::vector<std::string> listAntennas(const int direction, const size_t channel) const
    {
        std::vector<std::string> antennas;
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "listGains: invalid direction/channel");
            return antennas; 
        }
        antennas.push_back("RX");
        return antennas;
    }

    void setAntenna(const int direction, const size_t channel, const std::string &name)
    {
        return;
    }

    std::string getAntenna(const int direction, const size_t channel) const
    {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "getAntenna: invalid direction/channel");
            return "None"; 
        }
        return "RX";
    }

    /*******************************************************************
     * Gain API
     ******************************************************************/

    std::vector<std::string> listGains(const int direction, const size_t channel) const
    {
        std::vector<std::string> results;
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "listGains: invalid direction/channel");
            return results; 
        }
        results.push_back("ATT");
        return results;
    }

    bool hasGainMode(const int direction, const size_t channel) const
    {
        return false;
    }

    // void setGain(const int direction, const size_t channel, const double value) {}

    void setGain(const int direction, const size_t channel, const std::string &name, const double value)
    {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "setGain: invalid direction/channel");
            return; 
        }
        if (name == "ATT") {
            attenLevel = value;
        } else {
            // SoapySDR program should not get here
            throw std::runtime_error(std::string("Unknown GAIN ")+name);
            return;
        }
    }

    double getGain(const int direction, const size_t channel, const std::string &name) const
    {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "getGain: invalid direction/channel");
            return SOAPY_SDR_NOT_SUPPORTED; 
        }
        if (name == "ATT") {
            return attenLevel;
        } else {
            throw std::runtime_error(std::string("Unsupported GAIN ")+name);
        }
        return 0.0;
    }

    SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string &name) const
    {
        if (name == "ATT") {
            return SoapySDR::Range(-30, 0);
        } else {
            throw std::runtime_error(std::string("Unsupported gain: ") + name);
        }
        return SoapySDR::Range(0,0);
    }

    /*******************************************************************
     * Frequency API
     ******************************************************************/

    void setFrequency(const int direction,
                      const size_t channel,
                      const double frequency,
                      const SoapySDR::Kwargs &args)
    {
        setFrequency(direction, channel, "RF", frequency, args);
    }

    void setFrequency(const int direction, 
                      const size_t channel, 
                      const std::string &name, 
                      const double frequency, 
                      const SoapySDR::Kwargs &args)
    {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "setFrequency: invalid direction/channel");
            return;
        }
        if(name != "RF") {
            SoapySDR_logf(SOAPY_SDR_ERROR, "setFrequency: invalid name");
            return;
        }
        centerFrequency = frequency;
        return;
    }

    double getFrequency(const int direction, const size_t channel) const
    {
        return getFrequency(direction, channel, "RF");
    }

    double getFrequency(const int direction, const size_t channel, const std::string &name) const
    {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "getFrequency: invalid direction/channel");
            return SOAPY_SDR_NOT_SUPPORTED;
        }
        return centerFrequency;
    }

    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const
    {
        std::vector<std::string> names;
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "listFrequency: invalid direction/channel");
            return names;
        }
        names.push_back("RF");
        return names;
    }

    SoapySDR::RangeList getFrequencyRange(const int direction,
                                          const size_t channel) const 
    {
        return getFrequencyRange(direction, channel, "RF");
    }

    SoapySDR::RangeList getFrequencyRange(const int direction,
                                          const size_t channel,
                                          const std::string &name) const
    {
        SoapySDR::RangeList results;
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "getFrequencyRange: invalid direction/channel");
            return results;
        }
        if(name == "RF") {
            if (type < 3) {
                results.push_back(SoapySDR::Range(SM200_MIN_FREQ, SM200_MAX_FREQ));  
            } else {
                results.push_back(SoapySDR::Range(SM435_MIN_FREQ, SM435_MAX_FREQ));
            }
        }
        return results;
    }

    /*******************************************************************
     * Sample Rate API
     ******************************************************************/


    void setSampleRate(const int direction, const size_t channel, const double rate) {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "setSampleRate: invalid direction/channel");
            return;
        }

        if (type == smDeviceTypeSM200A ||type == smDeviceTypeSM200B || type == smDeviceTypeSM435B) {
            sampleRate = maxSmUsbSampleRate;
        } else if (type == smDeviceTypeSM200C || type == smDeviceTypeSM435C) {
            sampleRate = maxSmSfpSampleRate;
        } 

        decimation = 1;

        while(sampleRate > rate) {
            sampleRate /= 2;
            decimation *= 2;
        }

        bandwidth = sampleRate * sampleToBwRatio;
        maxBW = bandwidth;
    }

    double getSampleRate(const int direction, const size_t channel) const
    {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "getSampleRate: invalid direction/channel");
            return SOAPY_SDR_NOT_SUPPORTED;
        }
        return sampleRate;
    }

    SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const
    {
        SoapySDR::RangeList results;
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "getSampleRateRange: invalid direction/channel");
            return results;
        }

        double tempSampleRate = 0.0;
        if (type == smDeviceTypeSM200A ||type == smDeviceTypeSM200B || type == smDeviceTypeSM435B) {
            tempSampleRate = maxSmUsbSampleRate;
        } else if (type == smDeviceTypeSM200C || type == smDeviceTypeSM435C) {
            tempSampleRate = maxSmSfpSampleRate;
        }

        for(int tempDecimation = 1; tempDecimation <= 4096; tempDecimation *= 2) {
            results.push_back(SoapySDR::Range(tempSampleRate,tempSampleRate));
            tempSampleRate /= 2;
        }
        return results;
    }

    // Deprecated dont use
    std::vector<double> listSampleRates(const int direction, const size_t channel) const
    {
        std::vector<double> results;
        double tempSampleRate = 0.0;
        if (type == smDeviceTypeSM200A ||type == smDeviceTypeSM200B || type == smDeviceTypeSM435B) {
            tempSampleRate = maxSmUsbSampleRate;
        } else if (type == smDeviceTypeSM200C || type == smDeviceTypeSM435C) {
            tempSampleRate = maxSmSfpSampleRate;
        }

        for(int tempDecimation = 1; tempDecimation >= 4096; tempDecimation *= 2) {
            results.push_back(tempSampleRate);
            tempSampleRate /= 2;
        }
        return results;
    }     

    /*******************************************************************
     * Bandwidth API
     ******************************************************************/

    void setBandwidth(const int direction, const size_t channel, const double bw)
    {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "setBandwidth: invalid direction/channel");
            return;
        }
        if (bw > maxBW) {
            bandwidth = maxBW;
            SoapySDR_logf(SOAPY_SDR_INFO, "setBandwidth: %lf clamped to %lf.", bw, maxBW);  
        } else {
            bandwidth = bw;
        }
    }

    double getBandwidth(const int direction, const size_t channel) const
    {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "getBandwidth: invalid direction/channel");
            return SOAPY_SDR_NOT_SUPPORTED;
        }
        return bandwidth;
    }

    SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const
    {
        SoapySDR::RangeList results;
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "getBandwidthRange: invalid direction/channel");
            return results;
        }
        results.push_back(SoapySDR::Range(0,maxBW));
        return results;
    }

    // Deprecated dont use
    std::vector<double> listBandwidths(const int direction, const size_t channel) const
    {
        std::vector<double> results;
        results.insert(results.begin(), maxBW);
        return results;
    }

    /*******************************************************************
     * Native Access API
     ******************************************************************/

    void* getNativeDeviceHandle(void) const
    {
        return (void*) &deviceHandle;
    }
};

/***********************************************************************
 * Find available devices
 **********************************************************************/

SoapySDR::KwargsList findSignalHoundSM(const SoapySDR::Kwargs &args)
{
    int handle = -1;
    SmStatus status = smNoError;
    int serialNumber = 0;
    SmDeviceType type;   
    std::string hostAddr = SM200_ADDR_ANY;
    std::string deviceAddr = SM200_DEFAULT_ADDR;
    int port = SM200_DEFAULT_PORT;

    SoapySDR::KwargsList devices;

    if(args.count("hostAddr") &&
       args.count("deviceAddr") &&
       args.count("port")) {
        hostAddr = args.at("hostAddr");
        deviceAddr = args.at("deviceAddr");
        port = std::stoi(args.at("port"));
        
    }
    // Look for Networked SM
    status = smOpenNetworkedDevice(&handle, hostAddr.c_str(), deviceAddr.c_str(), port);
    if(status == smNoError) {
        smGetDeviceInfo(handle, &type, &serialNumber);
        smCloseDevice(handle);

        SoapySDR::Kwargs deviceInfo;

        deviceInfo["hostAddr"] = hostAddr;
        deviceInfo["deviceAddr"] = deviceAddr;
        deviceInfo["port"] = std::to_string(port);
        if (type == smDeviceTypeSM200C) {
            deviceInfo["type"] = SMTypeToString(type);
            deviceInfo["label"] = "SM200C [" + std::to_string(serialNumber) + "]";
        } else if (type == smDeviceTypeSM435C) {
            deviceInfo["type"] = SMTypeToString(type);
            deviceInfo["label"] = "SM435C [" + std::to_string(serialNumber) + "]";
        } 
        deviceInfo["serial"] = std::to_string(serialNumber);

        devices.push_back(deviceInfo);
    }

    // Look for USB SM
    int serials[SM_MAX_DEVICES]; 
    SmDeviceType types[SM_MAX_DEVICES];
    int count = SM_MAX_DEVICES;
    status = smGetDeviceList2(serials, types, &count);
    if(status != smNoError) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "Error: %s\n", smGetErrorString(status));
    }

    for(int i = 0; i < count; i++) {
        SoapySDR::Kwargs deviceInfo;

        if (types[i] == smDeviceTypeSM200A) {
            deviceInfo["type"] = SMTypeToString(types[i]);
            deviceInfo["label"] = "SM200A [" + std::to_string(serials[i]) + "]";    
        } else if (types[i] == smDeviceTypeSM200B) {
            deviceInfo["type"] = SMTypeToString(types[i]);
            deviceInfo["label"] = "SM200B [" + std::to_string(serials[i]) + "]";
        } else if (types[i] == smDeviceTypeSM435B) {
            deviceInfo["type"] = SMTypeToString(types[i]);
            deviceInfo["label"] = "SM435B [" + std::to_string(serials[i]) + "]";
        }
        deviceInfo["serial"] = std::to_string(serials[i]);
        devices.push_back(deviceInfo);
    }

    // Filter Device List Based on args
    for(int i = 0; i < static_cast<int>(devices.size()); i++)
    {
            for (auto it = devices[i].begin(); it != devices[i].end(); it++)
    {
        std::cout << it->first    // string (key)
                << ':'
                << it->second   // string's value 
                << std::endl;
    }
        // Remove all devices that don't contain IP specific device info
        if(args.count("hostAddr") &&
           args.count("deviceAddr") &&
           args.count("port")) {
            if(!devices[i].count("hostAddr")) {
                devices.erase(devices.begin() + i);
            }
        } else if(args.count("type")) {
            if(devices[i].at("type") != args.at("type")) {
                devices.erase(devices.begin() + i);
            }
        } else if(args.count("serial")) {
            if(devices[i].at("serial") != args.at("serial")) {
                devices.erase(devices.begin() + i);
            }
        }
    }
    return devices;
}

/***********************************************************************
 * Make device instance
 **********************************************************************/
SoapySDR::Device* makeSignalHoundSM(const SoapySDR::Kwargs &args)
{
    return new SignalHoundSM(args);
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerSignalHoundSM("SignalHoundSM", &findSignalHoundSM, &makeSignalHoundSM, SOAPY_SDR_ABI_VERSION);
