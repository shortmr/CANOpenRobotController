#include "TrignoMultiEMG.h"
TrignoMultiEMG::TrignoMultiEMG(int nSensor) : nSensor_(nSensor) {
    alreadyLogged_ = true;

    // Initialize spdlog's asynchronous logger
    if (!spdlog::thread_pool()) { // Check if the thread pool has already been initialized
        spdlog::init_thread_pool(8192, 1); // Queue size and number of threads
    }
    std::stringstream logFileName;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    logFileName << "emg/"<<std::put_time(&tm, "emg_%d-%m-%Y_%H-%M-%S") << ".csv";

    logger_ = spdlog::basic_logger_mt<spdlog::async_factory>("emg_logger", logFileName.str(), true);
    logger_->set_pattern("%v"); // Only log the message

    std::stringstream titleStream;
    titleStream << "emg_time," << "corc_time,";
    for(int n = 1; n <= nSensor_; n++){
        std::cout<<n<<std::endl;
        titleStream << "emg_"<<n;
        if(n != nSensor_) titleStream<<","; // add coma if not the last item
    }

    logger_->info(titleStream.str());
}

// Destructor to cleanup the logger
TrignoMultiEMG::~TrignoMultiEMG() {
    spdlog::drop_all(); // Cleanup spdlog resources
}

// Public member function to add a new data instance
void TrignoMultiEMG::addEMGSensor(double start_time, const std::string& emg_pos,
                                  int emg_id, const std::vector<double>& emg) {
    EMGData instance;
    instance.start_time = start_time;
    instance.emg_pos = emg_pos;
    instance.emg_id = emg_id;
    instance.emg = emg;

//    double sum = 0;
//    int count = 0;
//    for(const double& emgVal : emg){
//        sum += std::abs(emgVal);
//        count++;
//    }
    //instance.meanAbsEmg = sum/(float)count;

//    instance.meanAbsEmg = emg.back();

    multiEMGData_.push_back(instance);
    alreadyLogged_ = false;
}

// Public member function to get all data instances
std::vector<TrignoMultiEMG::EMGData> TrignoMultiEMG::getMultiEMGData() const {
    return multiEMGData_;
}

// Public member function to delete all stored instances
void TrignoMultiEMG::clearMultiEMGInstances() {
    multiEMGData_.clear();
}

// Newly added method to check for new data and log accordingly
void TrignoMultiEMG::log(double corcTime) {
    if (alreadyLogged_) return;

    std::stringstream logStream;

    if(multiEMGData_.size() != nSensor_){
        spdlog::warn("{} emg reading was expected. Received {}.", nSensor_, multiEMGData_.size());
    }

    int size = multiEMGData_[0].emg.size(); // all sensor data has the same size

    for(int time = 0; time < size; time++) { // loop each time instant
        if(time == 0) logStream << multiEMGData_[0].start_time<< "," <<corcTime<<",";
        else logStream << ",,";
        for (int sensor = 0; sensor < multiEMGData_.size(); sensor++) { // loop each sensor
            logStream<<multiEMGData_[sensor].emg[time]<<",";
        }
        logger_->info(logStream.str());
        logStream.str(""); // Clear the stringstream for the next instance
    }
    alreadyLogged_ = true; // Mark the current data as logged

}

std::vector<double> TrignoMultiEMG::getDownSampledMultiEMG() const {
    std::vector<double> meanAbsMultiEMG(nSensor_, 0); // initialize as zero val

    int sensor = 0;
    for(const auto& emgData : multiEMGData_){
        meanAbsMultiEMG[sensor++] = emgData.emg.back(); // just get the last data point
    }
    return meanAbsMultiEMG;
}