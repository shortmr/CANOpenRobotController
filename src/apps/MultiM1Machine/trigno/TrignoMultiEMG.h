#ifndef TRIGNO_EMG_H
#define TRIGNO_EMG_H

#include <string>
#include <vector>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/async.h>
#include <sstream>
#include <iostream>
#include <iomanip>

class TrignoMultiEMG {
public:
    TrignoMultiEMG(int nSensor);
    ~TrignoMultiEMG();

    struct EMGData {
        double start_time;
        std::string emg_pos;
        int emg_id;
        std::vector<double> emg;
        double meanAbsEmg;
    };

    // Public member function to add a new data instance
    void addEMGSensor(double start_time, const std::string& emg_pos, int emg_id, const std::vector<double>& emg);

    // Public member function to get all data instances
    std::vector<EMGData> getMultiEMGData() const;

    // Public member function to delete all stored instances
    void clearMultiEMGInstances();

    // Public member function to log emg data collection
    void log(double corcTime);

    std::vector<double> getDownSampledMultiEMG() const; // be careful retrun coppy not reference

private:
    // Private member variable to store data instances
    std::vector<EMGData> multiEMGData_;

    bool alreadyLogged_;

    std::shared_ptr<spdlog::logger> logger_; // spdlog logger declaration

    int nSensor_;
};

#endif 