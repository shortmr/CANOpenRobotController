#ifndef TRIGNO_IMU_H
#define TRIGNO_IMU_H

#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/async.h>
#include <sstream>
#include <iostream>
#include <iomanip>

class TrignoMultiIMU {
public:
    TrignoMultiIMU(int nSensor);
    ~TrignoMultiIMU();

    struct IMUData {
        double start_time;
        std::string imu_pos;
        int imu_id;
        Eigen::Quaterniond q0; 
        std::vector<Eigen::Quaterniond> q;
        std::vector<Eigen::Matrix3d> R_AB;
        std::vector<double> sagittalAngle;
    };

    // Public member function to add a new data instance
    void addIMUSensor(double start_time, 
                    const std::string& imu_pos,
                    int imu_id, 
                    Eigen::Quaterniond& q0,
                    std::vector<Eigen::Quaterniond>& q);

    // Public member function to get all data instances
    std::vector<IMUData> getMultiIMUData() const;

    // Public member function to delete all stored instances
    void clearMultiIMUInstances();

    // Public member function to log emg data collection
    void log(double corcTime);

    // returns the last received saggital plane angle for every imu
    std::vector<double> getSagittalAngleMultiImus () const; // be careful returning a coppy not reference

private:
    // Private member variable to store data instances
    std::vector<IMUData> multiIMUData_;
    bool alreadyLogged_;
    std::shared_ptr<spdlog::logger> logger_; // spdlog logger declaration
    int nSensor_;

    static constexpr int QUAT_SIZE = 4;
    std::vector<char> qLabelList{'x', 'y', 'z', 'w'};

    // direction 0 fro front placed sensors, 1 for back placed sensors
    void updateSagittalAngles(const std::vector<Eigen::Matrix3d>& R_AB,
                              std::vector<double>& sagittalAngle, bool direction);

};

#endif 