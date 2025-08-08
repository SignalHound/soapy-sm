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
    bool streamActive, serialSpecified, autoAtten, readMut;
    int deviceId, serial, decimation, numDevices, type; 
    int serials[SM_MAX_DEVICES];
    SmDeviceType types[SM_MAX_DEVICES];
    double sampleRate, centerFrequency, bandwidth, refLevel, attenLevel;
    SmStatus status;

    // Decimation to max bandwidth with filters
    const std::map<int, double> smBandwidth = {
        {4096, 9.375e3},
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
        {1, 41.5e6}
    };

    // Decimation to samplerate
    const std::map<int, double> smSamplerate = {
        {4096, 15e3},
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
        {1, 61.44e6}
    };

public:
    
    /*******************************************************************
     * Constructor and Destructor
     ******************************************************************/

    SignalHoundSM(const SoapySDR::Kwargs &args) {
        // Defaults
        streamActive = false;
        serialSpecified = false;
        autoAtten = false;
        readMut = false;
        deviceId = -1;
        serial = 0;
        decimation = 1;
        numDevices = SM_MAX_DEVICES;
        type = 0;
        sampleRate = 61.44e6;
        centerFrequency = 100e6;
        bandwidth = 41.5e6;
        refLevel = -20;
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
        smSetRefLevel(deviceId, refLevel);
        smSetAttenuator(deviceId, attenLevel);
    }

    ~SignalHoundSM(void) {
        smAbort(deviceId);
        smCloseDevice(deviceId);
    }

    /*******************************************************************
     * Identification API
     ******************************************************************/

    std::string getDriverKey(void) const {
        return "Signal Hound SM Series";
    }

    std::string getHardwareKey(void) const{
        if (type == smDeviceTypeSM200A){
            return "Signal Hound SM200A";    
        } else if (type == smDeviceTypeSM200B){
            return "Signal Hound SM200B";
        } else if (type == smDeviceTypeSM200C){
            return "Signal Hound SM200C";
        } else if (type == smDeviceTypeSM435B){
            return "Signal Hound SM435B";
        } else if (type == smDeviceTypeSM435C){
            return "Signal Hound SM435C";
        } else {
            return "Signal Hound SM"; 
        }    
    }

    SoapySDR::Kwargs getHardwareInfo(void) const {
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

    // void setFrontendMapping(const int direction, const std::string &mapping) {}

    // std::string getFrontendMapping(const int direction) const {}

    size_t getNumChannels(const int direction) const {
        return (direction == SOAPY_SDR_RX) ? 1 : 0;
    }

    SoapySDR::Kwargs getChannelInfo(const int direction, const size_t channel) const {
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

    // bool getFullDuplex(const int direction, const size_t channel) const {}

    /*******************************************************************
     * Stream API
     ******************************************************************/

    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const {
        std::vector<std::string> formats;
        formats.push_back(SOAPY_SDR_CF32);
        formats.push_back(SOAPY_SDR_CS16);
        return formats;
    }

    std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {
        fullScale = 1.0;
        return SOAPY_SDR_CF32;
    }

    // SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const {}

    SoapySDR::Stream* setupStream(
            const int direction,
            const std::string &format,
            const std::vector<size_t> &channels = std::vector<size_t>(),
            const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) {
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
        smAbort(deviceId);
        streamActive = false;
    }

    // size_t getStreamMTU(SoapySDR::Stream *stream) const{}

    int activateStream(
            SoapySDR::Stream* stream,
            const int flags = 0,
            const long long timeNs = 0,
            const size_t numElems = 0) {
        if (flags != 0) {
            return SOAPY_SDR_NOT_SUPPORTED;
        }
        if (autoAtten) {
            smSetRefLevel(deviceId, SM_DEFAULT_REF);
            smSetAttenuator(deviceId, SM_AUTO_ATTEN);
        } else {
            smSetRefLevel(deviceId, refLevel);
            smSetAttenuator(deviceId, attenLevel);
        }
        smSetIQSampleRate(deviceId, decimation);
        updateConfig();
        streamActive = true;
        return 0;

    }

    int deactivateStream(SoapySDR::Stream *stream, const int flags = 0, const long long timeNs = 0) {
        status = smAbort(deviceId);
        streamActive = false;
        return status;
    }

    int readStream(
            SoapySDR::Stream *stream,
            void * const *buffs,
            const size_t numElems,
            int &flags,
            long long &timeNs,
            const long timeoutUs = 100000) {
        if (readMut) {
            std::this_thread::sleep_for(std::chrono::microseconds(timeoutUs));
            return 0;
        }
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

    // int writeStream(SoapySDR::Stream* stream, const void* const* buffs, const size_t numElems, int &flags, const long long timeNs = 0, const long timeoutUs = 100000) {}

    // int readStreamStatus(SoapySDR::Stream* stream, size_t &chanMask, int &flags, long long &timeNs, const long timeoutUs = 100000) {}

    /*******************************************************************
     * Direct buffer access API
     ******************************************************************/

    // size_t getNumDirectAccessBuffers(SoapySDR::Stream* stream) {}

    // int getDirectAccessBufferAddrs(SoapySDR::Stream* stream, const size_t handle,  void** buff) {}

    // int acquireReadBuffer(SoapySDR::Stream *stream, size_t &handle, const void **buffs, int &flags, long long &timeNs, const long timeoutUs = 100000) {}

    // void releaseReadBuffer(SoapySDR::Stream* stream, const size_t handle) {}

    // int acquireWriteBuffer(SoapySDR::Stream *stream, size_t &handle, void **buffs, const long timeoutUs = 100000) {}

    // void releaseWriteBuffer(SoapySDR::Stream *stream, const size_t handle, const size_t numElems, int &flags, const long long timeNs = 0) {}

    /*******************************************************************
     * Antenna API
     ******************************************************************/

    std::vector<std::string> listAntennas(const int direction, const size_t channel) const {
        std::vector<std::string> antennas;
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "listGains: invalid direction/channel");
            return antennas; 
        }
        antennas.push_back("RX");
        return antennas;
    }

    void setAntenna(const int direction, const size_t channel, const std::string &name) {
        return;
    }

    std::string getAntenna(const int direction, const size_t channel) const {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "getAntenna: invalid direction/channel");
            return "None"; 
        }
        return "RX";
    }

    /*******************************************************************
     * Frontend corrections API
     ******************************************************************/

    // bool hasDCOffsetMode(const int direction, const size_t channel) const {}

    // void setDCOffsetMode(const int direction, const size_t channel) {}

    // bool getDCOffsetMode(const int direction, const size_t channel) const {}

    // bool hasDCOffset(const int direction, const size_t channel) const {}

    // void setDCOffset(const int direction, const size_t channel, const std::complex<double> &offset) {}

    // std::complex<double> getDCOffset(const int direction, const size_t channel) {}

    // bool hasIQBalance(const int direction, const size_t channel) const {}

    // void setIQBalance(const int direction, const size_t channel, const std::complex<double> &balance) {}

    // std::complex<double> getIQBalance(const int direction, const size_t channel) {}

    // bool hasIQBalanceMode(const int direction, const size_t channel) const {}

    // void setIQBalanceMode(const int direction, const size_t channel, const bool automatic) {}

    //bool getIQBalanceMode(const int direction, const size_t channel) const {}

    // bool hasFrequencyCorrection(const int direction, const size_t channel) const {}

    // void setFrequencyCorrection(const int direction, const size_t channel, const double value) {}

    // double getFrequencyCorrection(const int direction, const size_t channel) const {}

    /*******************************************************************
     * Gain API
     ******************************************************************/

    std::vector<std::string> listGains(const int direction, const size_t channel) const {
        std::vector<std::string> results;
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "listGains: invalid direction/channel");
            return results; 
        }
        results.push_back("ATT");
        results.push_back("REF");
        return results;
    }

    bool hasGainMode(const int direction, const size_t channel) const {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "hasGainMode: invalid direction/channel");
            return false; 
        }
        return true;
    }

    void setGainMode(const int direction, const size_t channel, const bool automatic) {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "setGainMode: invalid direction/channel");
            return; 
        }
        if (automatic) {
            autoAtten = true;
        } else {
            autoAtten = false;
        }
        updateConfig();
    }

    bool getGainMode(const int direction, const size_t channel) const {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "getGainMode: invalid direction/channel");
            return false; 
        }
        return autoAtten;
    }

    // void setGain(const int direction, const size_t channel, const double value) {}

    void setGain(const int direction, const size_t channel, const std::string &name, const double value) {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "setGain: invalid direction/channel");
            return; 
        }
        if(name == "REF") {
            refLevel = value;
        } else if (name == "ATT") {
            attenLevel = value;
        } else {
            // SoapySDR program should not get here
            throw std::runtime_error(std::string("Unknown GAIN ")+name);
            return;
        }
        updateConfig();
    }

    // double getGain(const int direction, const size_t channel) const {}

    double getGain(const int direction, const size_t channel, const std::string &name) const {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "getGain: invalid direction/channel");
            return SOAPY_SDR_NOT_SUPPORTED; 
        }
        if (name=="REF") {
            if (autoAtten) {
                return SM_DEFAULT_REF;
            } else return refLevel;
        } else if (name == "ATT") {
            if (autoAtten) {
                return SM_AUTO_ATTEN;
            } else return attenLevel;
        } else throw std::runtime_error(std::string("Unsupported GAIN ")+name);
        return 0.0;
    }

    // SoapySDR::Range getGainRange(const int direction, const size_t channel) const {}

    SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string &name) const {
        if (name == "REF") {
            return SoapySDR::Range(-160, 20);
        } else if (name == "ATT") {
            return SoapySDR::Range(0, 6);
        } else throw std::runtime_error(std::string("Unsupported gain: ") + name);
        return SoapySDR::Range(0,0);
    }

    /*******************************************************************
     * Frequency API
     ******************************************************************/

    void setFrequency(
            const int direction, 
            const size_t channel, 
            const double frequency, 
            const SoapySDR::Kwargs &args) {
        setFrequency(direction, channel, "RF", frequency, args);
    }

    void setFrequency(
            const int direction, 
            const size_t channel, 
            const std::string &name, 
            const double frequency, 
            const SoapySDR::Kwargs &args) {
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

    double getFrequency(const int direction, const size_t channel) const {
        return getFrequency(direction, channel, "RF");
    }

    double getFrequency(const int direction, const size_t channel, const std::string &name) const {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "getFrequency: invalid direction/channel");
            return SOAPY_SDR_NOT_SUPPORTED;
        }
        return centerFrequency;
    }

    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const {
        std::vector<std::string> names;
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "listFrequency: invalid direction/channel");
            return names;
        }
        names.push_back("RF");
        return names;
    }

    SoapySDR::RangeList getFrequencyRange(
            const int direction,
            const size_t channel) const {
        return getFrequencyRange(direction, channel, "RF");
    }

    SoapySDR::RangeList getFrequencyRange(
            const int direction,
            const size_t channel,
            const std::string &name) const {
        SoapySDR::RangeList results;
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "getFrequencyRange: invalid direction/channel");
            return results;
        }
        if(name == "RF") {
            if (type < 3){
                results.push_back(SoapySDR::Range(SM200_MIN_FREQ, SM200_MAX_FREQ));  
            } else {
                results.push_back(SoapySDR::Range(SM435_MIN_FREQ, SM435_MAX_FREQ));
            }
        }

        return results;
    }

    // SoapySDR::ArgInfoList getFrequencyArgsInfo(const int direction, const size_t channel) const {}

    /*******************************************************************
     * Sample Rate API
     ******************************************************************/


    void setSampleRate(const int direction, const size_t channel, const double rate) {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "setSampleRate: invalid direction/channel");
            return;
        }
        readMut = true;
        for (auto &sr: smSamplerate){
            if (sr.second > rate){
                continue;
            } else {
                sampleRate = sr.second;
                if (sampleRate != rate) SoapySDR_logf(SOAPY_SDR_WARNING, "setSampleRate: %lf clamped to nearest valid sample rate %lf ", rate, sr.second);
                decimation = sr.first;
                if (bandwidth > smBandwidth.at(decimation)){
                    SoapySDR_logf(SOAPY_SDR_WARNING, "setSampleRate: bandwidth %lf clamped to %lf due to sample rate limits", bandwidth, smBandwidth.at(decimation));
                    bandwidth = smBandwidth.at(decimation);    
                }
                break;
            }
        }
        updateConfig(true);
    }

    double getSampleRate(const int direction, const size_t channel) const {
        if (direction != SOAPY_SDR_RX and channel != 1) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "getSampleRate: invalid direction/channel");
            return SOAPY_SDR_NOT_SUPPORTED;
        }
        return sampleRate;
    }

    SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const {
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
    std::vector<double> listSampleRates(const int direction, const size_t channel) const {
        SoapySDR_log(SOAPY_SDR_WARNING, "listSampleRates: This function is deprecrated.");
        std::vector<double> results;
        for(auto &sr: smSamplerate)results.insert(results.begin(),sr.second);
        return results;
    }     

    /*******************************************************************
     * Bandwidth API
     ******************************************************************/

    void setBandwidth(const int direction, const size_t channel, const double bw) {
        if (direction != SOAPY_SDR_RX and channel != 1){
            SoapySDR_logf(SOAPY_SDR_ERROR, "setBandwidth: invalid direction/channel");
            return;
        }
        // Block if samplerate is changing
        while(readMut);
        if (bw > smBandwidth.at(decimation)) {
            bandwidth = smBandwidth.at(decimation);
            SoapySDR_logf(SOAPY_SDR_WARNING, "setBandwidth: %lf clamped to %lf due to sample rate limits", bw, smBandwidth.at(decimation));  
        } else {
            bandwidth = bw;
        }
        updateConfig();
    }

    double getBandwidth(const int direction, const size_t channel) const {
        if (direction != SOAPY_SDR_RX and channel != 1){
            SoapySDR_logf(SOAPY_SDR_ERROR, "getBandwidth: invalid direction/channel");
            return SOAPY_SDR_NOT_SUPPORTED;
        }
        return bandwidth;
    }

    SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const {
        SoapySDR::RangeList results;
        if (direction != SOAPY_SDR_RX and channel != 1){
            SoapySDR_logf(SOAPY_SDR_ERROR, "getBandwidthRange: invalid direction/channel");
            return results;
        }
        results.push_back(SoapySDR::Range(0,smBandwidth.at(decimation)));
        return results;
    }

    // Deprecated dont use
    std::vector<double> listBandwidths(const int direction, const size_t channel) const {
        SoapySDR_log(SOAPY_SDR_WARNING, "listBandwidths: This function is deprecrated.");
        std::vector<double> results;
        results.insert(results.begin(), smBandwidth.at(decimation));
        return results;
    }

    /*******************************************************************
     * Clocking API
     ******************************************************************/

    // void setMasterClockRate(const double rate) {}

    // double getMasterClockRate(void) const {}

    // SoapySDR::RangeList getMasterClockRates(void) const {}

    // void setReferenceClockRate(const double rate) {}

    // double getReferenceCFlockRate(void) const {}

    // SoapySDR::RangeList getReferenceClockRates(void) const {}

    // std::vector<std::string> listClockSources(void) const {}

    // void setClockSource(const std::string &source) {}

    // std::string getClockSource(void) const {}

    /*******************************************************************
     * Time API
     ******************************************************************/

    // std::vector<std::string> listTimeSources(void) const {}

    // void setTimeSource(const std::string &source) {}

    // std::string getTimeSource(void) const {}

    // bool hasHardwareTime(const std::string &what = "") const {}

    // long long getHardwareTime(const std::string &what = "") const {}

    // void setHardwareTime(const long long timeNs, const std::string &what = "") {}

    /*******************************************************************
     * Sensor API
     ******************************************************************/

    // std::vector<std::string> listSensors(void) const {}

    // SoapySDR::ArgInfo getSensorInfo(const std::string &key) const {}

    // std::string readSensor(const std::string &key) const {}

    // std::vector<std::string> listSensors(const int direction, const size_t channel) const {}

    // SoapySDR::ArgInfo getSensorInfo(const int direction, const size_t channel, const std::string &key) const {}

    // std::string readSensor(const int direction, const size_t channel, const std::string &key) const {}

    /*******************************************************************
     * Register API
     ******************************************************************/

    // std::vector<std::string> listRegisterInterfaces(void) const {}

    // void writeRegister(const std::string &name, const unsigned addr, const unsigned value) {}

    // unsigned readRegister(const std::string &name, const unsigned addr) const {}

    // void writeRegister(const unsigned addr, const unsigned value) {}

    // unsigned readRegister(const unsigned addr) const {}

    // void writeRegisters(const std::string &name, const unsigned addr, const std::vector<unsigned> &value) {}

    // std::vector<unsigned> readRegisters(const std::string &name, const unsigned addr, const size_t length) const {}

    /*******************************************************************
     * Settings API
     ******************************************************************/

    // SoapySDR::ArgInfoList getSettingInfo(void) const {}

    // SoapySDR::ArgInfo getSettingInfo(const std::string &key) const {}

    // void writeSetting(const std::string &key, const std::string &value) {}

    // std::string readSetting(const std::string &key) const {}

    /*******************************************************************
     * I2C API
     ******************************************************************/

    // void writeI2C(const int addr, const std::string &data) {}

    // std::string readI2C(const int addr, const size_t numBytes) {}

    /*******************************************************************
     * SPI API
     ******************************************************************/

    // unsigned transactSPI(const int addr, const unsigned data, const size_t numBits) {}

    /*******************************************************************
     * UART API
     ******************************************************************/

    // std::vector<std::string> listUARTs(void) const {}

    // void writeUART(const std::string &which, const std::string &data) {}

    // std::string readUART(const std::string &which, const long timeoutUs = 100000) const {}

    /*******************************************************************
     * Native Access API
     ******************************************************************/

    void* getNativeDeviceHandle(void) const {
        return (void*) &deviceId;
    }

    /*******************************************************************
     * Util API
     ******************************************************************/

    void updateConfig(bool srChange=false) {
        readMut = true;
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
        if (autoAtten) {
            smSetRefLevel(deviceId, SM_DEFAULT_REF);
            smSetAttenuator(deviceId, SM_AUTO_ATTEN);
        } else {
            smSetRefLevel(deviceId, refLevel);
            smSetAttenuator(deviceId, attenLevel);
        }

        // Activate configuration
        smConfigure(deviceId, smModeIQStreaming);
        readMut = false;
    }

};

/***********************************************************************
 * Find available devices
 **********************************************************************/
SoapySDR::KwargsList findSignalHoundSM(const SoapySDR::Kwargs &args) {
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
        if (types[i] == smDeviceTypeSM200A){
            deviceInfo["label"] = "SM200A [" + std::to_string(serials[i]) + "]";    
        } else if (types[i] == smDeviceTypeSM200B){
            deviceInfo["label"] = "SM200B [" + std::to_string(serials[i]) + "]";
        } else if (types[i] == smDeviceTypeSM200C){
            deviceInfo["label"] = "SM200C [" + std::to_string(serials[i]) + "]";
        } else if (types[i] == smDeviceTypeSM435B){
            deviceInfo["label"] = "SM435B [" + std::to_string(serials[i]) + "]";
        } else if (types[i] == smDeviceTypeSM435C){
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
SoapySDR::Device* makeSignalHoundSM(const SoapySDR::Kwargs &args) {
    return new SignalHoundSM(args);
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerSignalHoundSM("SignalHoundSM", &findSignalHoundSM, &makeSignalHoundSM, SOAPY_SDR_ABI_VERSION);