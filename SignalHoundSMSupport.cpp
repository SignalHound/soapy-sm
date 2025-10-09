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

/***********************************************************************
 * Device interface
 **********************************************************************/
class SignalHoundSM : public SoapySDR::Device {
private:
    // Variables used
    mutable std::mutex devMutex;
    bool serialSpecified;
    int deviceId, serial, decimation, numDevices, type; 
    int serials[SM_MAX_DEVICES];
    SmDeviceType types[SM_MAX_DEVICES];
    double sampleRate, centerFrequency, bandwidth, attenLevel;
    SmStatus status;

    // Decimation to max bandwidth with filters
    const std::map<int, double> smBandwidth = {{4096, 9.375e3},
                                               {2048, 18.75e3},
                                               {1024, 37.5e3},
                                               {512, 75e3},
                                               {256, 150e3},
                                               {128, 300e3},
                                               {64, 600e3},
                                               {32, 1.2e6},
                                               {16, 2.4e6},
                                               {8, 4.8e6},
                                               {4, 9.6e6},
                                               {2, 19.2e6},
                                               {1, 41.5e6}};

    // Decimation to samplerate
    const std::map<int, double> smSamplerate = {{4096, 15e3},
                                                {2048, 30e3},
                                                {1024, 60e3},
                                                {512, 120e3},
                                                {256, 240e3},
                                                {128, 480e3},
                                                {64, 960e3},
                                                {32, 1.92e6},
                                                {16, 3.84e6},
                                                {8, 7.68e6},
                                                {4, 15.36e6},
                                                {2, 30.72e6},
                                                {1, 61.44e6}};

public:
    
    /*******************************************************************
     * Constructor and Destructor
     ******************************************************************/

    SignalHoundSM(const SoapySDR::Kwargs &args) 
    {
        // Defaults
        serialSpecified = false;
        deviceId = -1;
        serial = 0;
        decimation = 1;
        numDevices = SM_MAX_DEVICES;
        type = 0;
        sampleRate = 61.44e6;
        centerFrequency = 100e6;
        bandwidth = 41.5e6;
        attenLevel = -1;

        // Read provided serial
        if (args.count("serial") != 0) {
            try {
                serial = std::stoull(args.at("serial"), nullptr, 10);
            } catch (const std::invalid_argument &) {
                throw std::runtime_error("serial is not a number");
            } catch (const std::out_of_range &) {
                throw std::runtime_error("serial value of out range");
            }
            serialSpecified = true;

        // Read provided device id
        } else if (args.count("device_id") != 0) {
            try {
                deviceId = std::stoi(args.at("device_id"));
            } catch (const std::invalid_argument &) {
                throw std::runtime_error("device_id is not a number");
            } catch (const std::out_of_range &) {
                throw std::runtime_error("device_id of out range");
            }
        }

        // Retrieve device list
        status = smGetDeviceList2(serials, types, &numDevices);
        if (status != smNoError) {
            throw std::runtime_error("Failed to retrieve list of SM devices");
        }

        if (numDevices < 1) {
            throw std::runtime_error("No SM devices found");
        }

        // Find serial in device list
        if (serialSpecified) {
            for (int i = 0; i < numDevices; i++) {
                if (serials[i] == serial) {
                    deviceId = i;
                    break;
                }
            }
            if (deviceId < 0) {
                throw std::runtime_error("SM device with S/N " 
                                            + std::to_string(serial) 
                                            + " not found");
            }
        } else {
            if (deviceId < 0) {
                deviceId = 0; // Default
            } else if (deviceId >= numDevices) {
                throw std::runtime_error("SM device_id out of range [0 .. " 
                                            + std::to_string(numDevices-1) 
                                            + "].");
            }
            serial = serials[deviceId];
        }

        // Open device 
        if ((status = smOpenDeviceBySerial(&deviceId, serial)) != smNoError) {
            throw std::runtime_error("Unable to open SM device " 
                                            + std::to_string(deviceId) 
                                            + " with S/N " 
                                            + std::to_string(serial));
        }
        type = types[deviceId];

        // Configure device defaults
        smSetIQCenterFreq(deviceId, centerFrequency);
        smSetIQSampleRate(deviceId, decimation);
        smSetIQBandwidth(deviceId, smTrue, bandwidth);
        smSetAttenuator(deviceId, attenLevel);
    }

    ~SignalHoundSM(void)
    {
        smAbort(deviceId);
        smCloseDevice(deviceId);
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
        const std::lock_guard<std::mutex> lock(devMutex);
        if (type == smDeviceTypeSM200A) {
            return "Signal Hound SM200A";    
        } else if (type == smDeviceTypeSM200B) {
            return "Signal Hound SM200B";
        } else if (type == smDeviceTypeSM200C) {
            return "Signal Hound SM200C";
        } else if (type == smDeviceTypeSM435B) {
            return "Signal Hound SM435B";
        } else if (type == smDeviceTypeSM435C) {
            return "Signal Hound SM435C";
        } else {
            return "Signal Hound SM"; 
        }    
    }

    SoapySDR::Kwargs getHardwareInfo(void) const
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        // Get firmware info
        int firmmaj = 0;
        int firmmin = 0;
        int firmrev = 0;
        smGetFirmwareVersion(deviceId, &firmmaj, &firmmin, &firmrev);

        // Get diagnostics
        float temp, volt, curr;
        smGetDeviceDiagnostics(deviceId, &volt, &curr, &temp);

        // Place into in Kwargs dictionary
        SoapySDR::Kwargs args;

        args["device_id"] = std::to_string(deviceId);
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
        const std::lock_guard<std::mutex> lock(devMutex);
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
        const std::lock_guard<std::mutex> lock(devMutex);
        // Check channel config
        if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0)) {
            throw std::runtime_error("setupStream invalid channel selection");
        }

        // Check format
        if (format == SOAPY_SDR_CF32) {
            SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32");
            smSetIQDataType(deviceId, smDataType32fc);
        } else if (format == SOAPY_SDR_CS16) {
            SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16");
            smSetIQDataType(deviceId, smDataType16sc);
        } else {
            throw std::runtime_error("setupStream: Invalid format '" + format
                            + "' -- Only CF32 and CS16 are supported by SignalHoundSM module.");
        }
        return (SoapySDR::Stream*) this;
    }

    void closeStream(SoapySDR::Stream *stream) {
        const std::lock_guard<std::mutex> lock(devMutex);
        smAbort(deviceId);
    }

    // size_t getStreamMTU(SoapySDR::Stream *stream) const{}

    int activateStream(SoapySDR::Stream* stream,
                       const int flags = 0,
                       const long long timeNs = 0,
                       const size_t numElems = 0) 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        if (flags != 0) {
            return SOAPY_SDR_NOT_SUPPORTED;
        }

        updateConfig();
        return 0;
    }

    int deactivateStream(SoapySDR::Stream *stream, const int flags = 0, const long long timeNs = 0)
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        status = smAbort(deviceId);
        return status;
    }

    int readStream(SoapySDR::Stream *stream,
                   void * const *buffs,
                   const size_t numElems,
                   int &flags,
                   long long &timeNs,
                   const long timeoutUs = 100000)
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        // Start clock
        const auto start = std::chrono::high_resolution_clock::now();

        // Grab IQ data
        status = smGetIQ(deviceId, *buffs, numElems, 0, 0, 0, smFalse, 0, 0);
        if (status > smNoError) {
            SoapySDR_logf(SOAPY_SDR_WARNING, "GetIQ: %s", smGetErrorString(status));
            std::this_thread::sleep_for(std::chrono::microseconds(timeoutUs));
        }
        if (status < smNoError) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "GetIQ: %s", smGetErrorString(status));
            updateConfig(true);
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
        const std::lock_guard<std::mutex> lock(devMutex);
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
        updateConfig();
    }

    double getGain(const int direction, const size_t channel, const std::string &name) const
    {
        const std::lock_guard<std::mutex> lock(devMutex);
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
        const std::lock_guard<std::mutex> lock(devMutex);
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "setFrequency: invalid direction/channel");
            return;
        }
        if(name != "RF") {
            SoapySDR_logf(SOAPY_SDR_ERROR, "setFrequency: invalid name");
            return;
        }
        centerFrequency = frequency;
        updateConfig();
        return;
    }

    double getFrequency(const int direction, const size_t channel) const
    {
        return getFrequency(direction, channel, "RF");
    }

    double getFrequency(const int direction, const size_t channel, const std::string &name) const
    {
        const std::lock_guard<std::mutex> lock(devMutex);
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
        const std::lock_guard<std::mutex> lock(devMutex);
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "setSampleRate: invalid direction/channel");
            return;
        }
        for (auto &sr: smSamplerate) {
            if (sr.second > rate) {
                continue;
            } else {
                sampleRate = sr.second;
                if (sampleRate != rate) {
                    SoapySDR_logf(SOAPY_SDR_WARNING, "setSampleRate: %lf clamped to nearest valid sample rate %lf ", rate, sr.second);
                }
                decimation = sr.first;
                if (bandwidth > smBandwidth.at(decimation)) {
                    SoapySDR_logf(SOAPY_SDR_WARNING, "setSampleRate: bandwidth %lf clamped to %lf due to sample rate limits", bandwidth, smBandwidth.at(decimation));
                    bandwidth = smBandwidth.at(decimation);    
                }
                break;
            }
        }
        updateConfig(true);
    }

    double getSampleRate(const int direction, const size_t channel) const
    {
        const std::lock_guard<std::mutex> lock(devMutex);
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
        for(auto &sr: smSamplerate) {
            results.push_back(SoapySDR::Range(sr.second,sr.second));
        }

        return results;
    }

    // Deprecated dont use
    std::vector<double> listSampleRates(const int direction, const size_t channel) const
    {
        SoapySDR_log(SOAPY_SDR_WARNING, "listSampleRates: This function is deprecrated.");
        std::vector<double> results;
        for(auto &sr: smSamplerate) {
            results.insert(results.begin(),sr.second);
        }
        return results;
    }     

    /*******************************************************************
     * Bandwidth API
     ******************************************************************/

    void setBandwidth(const int direction, const size_t channel, const double bw)
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "setBandwidth: invalid direction/channel");
            return;
        }
        if (bw > smBandwidth.at(decimation)) {
            bandwidth = smBandwidth.at(decimation);
            SoapySDR_logf(SOAPY_SDR_WARNING, "setBandwidth: %lf clamped to %lf due to sample rate limits", bw, smBandwidth.at(decimation));  
        } else {
            bandwidth = bw;
        }
        updateConfig();
    }

    double getBandwidth(const int direction, const size_t channel) const
    {
        const std::lock_guard<std::mutex> lock(devMutex);
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
        results.push_back(SoapySDR::Range(0,smBandwidth.at(decimation)));
        return results;
    }

    // Deprecated dont use
    std::vector<double> listBandwidths(const int direction, const size_t channel) const
    {
        SoapySDR_log(SOAPY_SDR_WARNING, "listBandwidths: This function is deprecrated.");
        std::vector<double> results;
        results.insert(results.begin(), smBandwidth.at(decimation));
        return results;
    }

    /*******************************************************************
     * Native Access API
     ******************************************************************/

    void* getNativeDeviceHandle(void) const
    {
        return (void*) &deviceId;
    }

    /*******************************************************************
     * Util API
     ******************************************************************/

    void updateConfig(bool srChange=false)
    {
        if (srChange) {
            // Abort current proccess
            smAbort(deviceId);
            SoapySDR_logf(SOAPY_SDR_INFO, "SM: changing samplerate...");
            // Give time for abort to complete
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        // Configure device
        smSetIQBandwidth(deviceId, smTrue, bandwidth);
        smSetIQCenterFreq(deviceId, centerFrequency);
        smSetAttenuator(deviceId, ((int)attenLevel)/-5);

        // Activate configuration
        smConfigure(deviceId, smModeIQStreaming);
    }

};

/***********************************************************************
 * Find available devices
 **********************************************************************/
SoapySDR::KwargsList findSignalHoundSM(const SoapySDR::Kwargs &args)
{
    int serials[SM_MAX_DEVICES]; 
    SmDeviceType types[SM_MAX_DEVICES];
    int count = SM_MAX_DEVICES;
    SmStatus status = smGetDeviceList2(serials, types, &count);
    if(status != smNoError) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "Error: %s\n", smGetErrorString(status));
    }

    SoapySDR::KwargsList devices;

    for(int i = 0; i < count; i++) {
        SoapySDR::Kwargs deviceInfo;

        deviceInfo["device_id"] = std::to_string(i);
        if (types[i] == smDeviceTypeSM200A) {
            deviceInfo["label"] = "SM200A [" + std::to_string(serials[i]) + "]";    
        } else if (types[i] == smDeviceTypeSM200B) {
            deviceInfo["label"] = "SM200B [" + std::to_string(serials[i]) + "]";
        } else if (types[i] == smDeviceTypeSM200C) {
            deviceInfo["label"] = "SM200C [" + std::to_string(serials[i]) + "]";
        } else if (types[i] == smDeviceTypeSM435B) {
            deviceInfo["label"] = "SM435B [" + std::to_string(serials[i]) + "]";
        } else if (types[i] == smDeviceTypeSM435C) {
            deviceInfo["label"] = "SM435C [" + std::to_string(serials[i]) + "]";
        } else {
            deviceInfo["label"] = "SM [" + std::to_string(serials[i]) + "]"; 
        } 
        deviceInfo["serial"] = std::to_string(serials[i]);

        devices.push_back(deviceInfo);
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