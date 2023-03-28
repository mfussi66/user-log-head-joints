#include "Reader.h"

Reader::Reader(double period, const Property & conf, const std::string & filename, 
        const std::vector<std::string> & interfaces, const int n_axes, 
        const std::vector<int>& axes_indices, const std::vector<std::string>& axes_names, const std::string& port_name)
: PeriodicThread(period) {
    n_axes_ = n_axes;
    thread_period_ = period;
    conf_ = conf;
    filename_ = filename;
    interfaces_ = interfaces;
    axes_to_log_.resize(n_axes_);
    axes_to_log_ = axes_indices;
    axes_names_ = axes_names;
    port_name_ = port_name;

}

bool Reader::threadInit() {

    if (!driver.open(conf_)) {
    yError() << "Failed to connect to" << conf_.find("remote").asString();
    return false;
    }

    if (!(driver.view(iEnc))) {
    yError() << "Failed to open encoder interface";
    driver.close();
    return false;
    }

    if(true){
    joint_encoders.resize(1, std::vector<double>(n_axes_));
    line_.resize(n_axes_);
    yInfo() << "naxes" << n_axes_;
    } else {
    yError() << "Could not get the number of joint_encoders!";
    return false;
    }

    if (!(driver.view(iPidCtrl))) {
    yError() << "Failed to open PID control interface";
    driver.close();
    return false;
    }

    if (!(driver.view(iPosCtrl))) {
    yError() << "Failed to open position control interface";
    driver.close();
    return false;
    }

    if (!(driver.view(iMEnc))) {
    yError() << "Failed to open motor encoder interface";
    driver.close();
    return false;
    }

    if (true) {
    motor_encoders.resize(1, std::vector<double>(n_axes_));
    pids_reference.resize(1, std::vector<double>(n_axes_));
    pids_error.resize(1, std::vector<double>(n_axes_));
    pids_out.resize(1, std::vector<double>(n_axes_));
    } else {
    yError() << "Could not get the number of joint_encoders!";
    return false;
    }

    fout.open(filename_);

    if (!fout.is_open()) {
    yError() << "Failed to open " << filename_;
    return false;
    }


    fout << "Time"<< ","
        << "Roll,Pitch,Yaw" << ","
        << "Mot0_Enc"<< ","<< "Mot1_Enc"<< ","<< "Mot2_Enc" << ","
        << "Aea0_Ref"<< ","<< "Aea1_Ref"<< ","<< "Aea2_Ref" << ","
        << "Aea0_Err"<< ","<< "Aea1_Err"<< ","<< "Aea2_Err" << ","
        << "Pid0_Out"<< ","<< "Pid1_Out"<< ","<< "Pid2_Out" <<  std::endl;
    t0 = Time::now();


    if(!port_outs.open(port_name_ + "/pid/out:o") ||
        !port_ref.open(port_name_ + "/pid/ref:o") ||
        !port_fbk.open(port_name_ + "/pid/fbk:o") ||
        !port_fbk.open(port_name_ + "/motor/enc:o") ) {
            yError() << "Could not open one or more ports!";
            return false;
    }

    return true;
}

std::stringstream Reader::constructLine(uint32_t i) {

    std::stringstream line;

    line << time_vector[i];

    for(auto & e : joint_encoders[i])
        {
        line << "," << e;
        }
    for(auto & e : motor_encoders[i]) line << "," << e;

    for(auto & e : pids_reference[i]) line << "," << e;

    for(auto & e : pids_error[i]) line << "," << e;

    for(auto & e : pids_out[i]) line << "," << e;

    line << std::endl;

    return line;
}

void Reader::threadRelease() {

    for(uint32_t i = 0; i < time_vector.size(); ++i) {
    fout << constructLine(i).str();
    }

    fout.close();
    driver.close();
    yInfo() << "Done.";

    // Close I/O resources
}

void Reader::run() {
    time_vector.push_back(static_cast<double>(Time::now() - t0));


    uint8_t i = 0;
    for (auto & ax : axes_to_log_) {
    iEnc->getEncoder(ax, &line_[i]);
    ++i;
    }
    joint_encoders.push_back(line_);

    for (auto & ax : axes_to_log_) {
    iMEnc->getMotorEncoder(ax, &line_[i]);
    ++i;
    }
    motor_encoders.push_back(line_);
    i = 0;

    for (auto & ax : axes_to_log_) {
    iPidCtrl->getPidReference(VOCAB_PIDTYPE_POSITION, ax, &line_[i]);
    ++i;
    bot_ref.addFloat64(line_[i]);
    }
    i = 0;

    pids_reference.push_back(line_);
    if(!port_ref.write(bot_ref)) {
    yError() << "Could not write to port" << port_ref.getName();
    }
    bot_ref.clear();
    yarp::sig::Vector references(n_axes_, line_.data());

    for (auto & ax : axes_to_log_){
    iPidCtrl->getPidError(VOCAB_PIDTYPE_POSITION, ax, &line_[i]);
    ++i;
    }
    i = 0;
    pids_error.push_back(line_);
    yarp::sig::Vector pid_errors(n_axes_, line_.data());

    for(uint8_t j = 0; j < n_axes_; ++j) {
        bot_traj.addFloat64(references[j] - pid_errors[j]);
    }
    if(!port_fbk.write(bot_traj)) {
        yError() << "Could not write to port" << port_fbk.getName();
    }
    bot_traj.clear();

    for (auto & ax : axes_to_log_) {
    iPidCtrl->getPidOutput(VOCAB_PIDTYPE_POSITION, ax, &line_[i]);
    bot_out.addFloat64(line_[i]);
    ++i;
    }
    i = 0;
    pids_out.push_back(line_);
    if(!port_outs.write(bot_out)) {
    yError() << "Could not write to port" << port_ref.getName();
    }
    bot_out.clear();
}