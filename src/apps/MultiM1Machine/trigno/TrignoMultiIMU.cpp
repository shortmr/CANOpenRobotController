#include "TrignoMultiIMU.h"

TrignoMultiIMU::TrignoMultiIMU(int nSensor) : nSensor_(nSensor) {
    alreadyLogged_ = true;
    // Initialize spdlog's asynchronous logger
    if (!spdlog::thread_pool()) { // Check if the thread pool has already been initialized
        spdlog::init_thread_pool(8192, 1); // Queue size and number of threads
    }

    std::stringstream logFileName;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    logFileName << "imu/"<<std::put_time(&tm, "imu_%d-%m-%Y_%H-%M-%S") << ".csv";

    logger_ = spdlog::basic_logger_mt<spdlog::async_factory>("imu_logger", logFileName.str(), true);
    logger_->set_pattern("%v"); // Only log the message

    std::stringstream titleStream;
    titleStream << "imu_time," << "corc_time,";
    for(int sensor = 1; sensor <= nSensor_; sensor++){
        for(const char& qLabel : qLabelList){
            titleStream << "imu_"<<sensor<<qLabel;
            if(sensor != nSensor_ || qLabel != qLabelList.back()) titleStream<<","; // add coma if not the last item
        }
    }

    logger_->info(titleStream.str());
}

// Destructor to cleanup the logger
TrignoMultiIMU::~TrignoMultiIMU() {
    spdlog::drop_all(); // Cleanup spdlog resources
}

// Public member function to add a new data instance
void TrignoMultiIMU::addIMUSensor(double start_time, const std::string& imu_pos, int imu_id, Eigen::Quaterniond& q0,
                                  std::vector<Eigen::Quaterniond>& q) {
    IMUData instance;
    instance.start_time = start_time;
    instance.imu_pos = imu_pos;
    instance.imu_id = imu_id;
    instance.q0 = q0;
    instance.q = q;

    // 0 is wrt imus frame(unknown), A is horizontal calibration orient, B is current orient
    for(int time = 0; time < q.size(); time++) { // loop each time instant and calculate R_AB
        Eigen::Matrix3d R_0A = instance.q0.toRotationMatrix();
        Eigen::Matrix3d R_0B = instance.q[time].toRotationMatrix();
        instance.R_AB.push_back(R_0A.transpose()*R_0B);
    }

    bool direction;
    if(instance.imu_pos.find("front") != std::string::npos){
        direction = 0;
    } else if(instance.imu_pos.find("back") != std::string::npos){
        direction = 1;
    } else{
        spdlog::warn("imu pos for sensor {} doesn't include front, back or trunk. Assuming front");
        direction = 0;
    }

    instance.sagittalAngle = std::vector<double>(instance.R_AB.size(), 0);
    updateSagittalAngles(instance.R_AB, instance.sagittalAngle, direction);

    multiIMUData_.push_back(instance);
    alreadyLogged_ = false;
}

// Public member function to get all data instances
std::vector<TrignoMultiIMU::IMUData> TrignoMultiIMU::getMultiIMUData() const {
    return multiIMUData_;
}

// Public member function to delete all stored instances
void TrignoMultiIMU::clearMultiIMUInstances() {
    multiIMUData_.clear();
}

// Newly added method to check for new data and log accordingly
void TrignoMultiIMU::log(double corcTime) {
    if (alreadyLogged_) return;

    std::stringstream logStream;

    if(multiIMUData_.size() != nSensor_){
        spdlog::warn("{} emg reading was expected. Received {}.", nSensor_, multiIMUData_.size());
    }

    int size = multiIMUData_[0].q.size(); // all sensor data has the same size

    for(int time = 0; time < size; time++) { // loop each time instant
        if(time == 0) {
            logStream << multiIMUData_[0].start_time<< "," <<corcTime<<",";
        }
        else logStream << ",,"; // skipping imu time, corc time, and q0_x, q0_y, q0_z, q0_w
        for (int sensor = 0; sensor < multiIMUData_.size(); sensor++) { // loop each sensor
            Eigen::Quaterniond q_ab = Eigen::Quaterniond(multiIMUData_[sensor].R_AB[time]);
            logStream<<q_ab.x()<<","<<q_ab.y()<<","<<q_ab.z()<<","<<q_ab.w()<<",";
        }
        logger_->info(logStream.str());
        logStream.str(""); // Clear the stringstream for the next instance
    }
    alreadyLogged_ = true; // Mark the current data as logged

}

void TrignoMultiIMU::updateSagittalAngles(const std::vector<Eigen::Matrix3d>& R_AB,
                                          std::vector<double>& sagittalAngle, bool direction) {

    sagittalAngle = std::vector<double>(R_AB.size(), 0); // initialize as zero

    for(int time = 0; time<R_AB.size(); time++){
        double angle;
        Eigen::Matrix3d R = R_AB[time];

        angle = std::asin(R(2,2));
        angle += M_PI_2;

        if(direction == 1){
            angle = -angle + M_PI;
        }

        sagittalAngle[time] = angle;
    }
}

std::vector<double> TrignoMultiIMU::getSagittalAngleMultiImus() const {

    std::vector<double> sagAngles(nSensor_, 0);

    int sensor = 0;
    for(const auto& imuData : multiIMUData_){
        sagAngles[sensor++] = (imuData.sagittalAngle.back());
    }

    return sagAngles;
}