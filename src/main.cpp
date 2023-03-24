/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include <memory>


using namespace yarp::dev;

std::array<std::string, 5> available_interfaces = {
  "joint_encoder",
  "motor_encoder",
  "pid_reference",
  "pid_error",
  "pid_out" };

/******************************************************************************/

class Reader : public yarp::os::PeriodicThread {
  private:

    PolyDriver driver;
    IEncoders * iEnc{ nullptr };
    IPidControl * iPidCtrl{ nullptr };
    IPositionControl * iPosCtrl{ nullptr };
    IMotorEncoders * iMEnc {nullptr};
    std::vector<std::string> interfaces_;
    yarp::os::Property conf_;
    double thread_period;
    int n_axes_;
    std::string filename_;
    std::vector<double> time_vector;
    double t0;
    std::vector<std::vector<double>> joint_encoders;
    std::vector<std::vector<double>> motor_encoders;
    std::vector<std::vector<double>> pids_out;
    std::vector<std::vector<double>> pids_reference;
    std::vector<std::vector<double>> pids_error;
    std::vector<double> line_;
    std::ofstream fout;
    std::vector<int> axes_to_log_;
    double value_;

  public:

    Reader(double period, const yarp::os::Property & conf, 
            const std::string & filename, 
            const std::vector<std::string> & interfaces, const int n_axes, const std::vector<int>& axes_to_log)
    : yarp::os::PeriodicThread(period) {
        n_axes_ = n_axes;
        thread_period = period;
        conf_ = conf;
        filename_ = filename;
        interfaces_ = interfaces;
        axes_to_log_.resize(n_axes_);
        axes_to_log_ = axes_to_log;
    }

    bool threadInit() {

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
    t0 = yarp::os::Time::now();

      return true;
    }

    std::stringstream constructLine(uint32_t i) {

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

    void threadRelease() {

      for(uint32_t i = 0; i < time_vector.size(); ++i) {
        fout << constructLine(i).str();
      }

      fout.close();
      driver.close();
      yInfo() << "Done.";

      // Close I/O resources
    }

    void run() {
      time_vector.push_back(static_cast<double>(yarp::os::Time::now() - t0));


      for (auto & i : axes_to_log_) {
        iEnc->getEncoder(i, &value_);
        line_[i] = value_;
      }
      joint_encoders.push_back(line_);

      for (auto & i : axes_to_log_) {
        iMEnc->getMotorEncoder(i, &value_);
        line_[i] = value_;
      }
      motor_encoders.push_back(line_);

      for (auto & i : axes_to_log_) {
        iPidCtrl->getPidReference(VOCAB_PIDTYPE_POSITION, i, &value_);
        line_[i] = value_;
      }
      pids_reference.push_back(line_);

      for (auto & i : axes_to_log_) {
        iPidCtrl->getPidError(VOCAB_PIDTYPE_POSITION, i, &value_);
        line_[i] = value_;
      }
      pids_error.push_back(line_);

      for (auto & i : axes_to_log_) {
        iPidCtrl->getPidOutput(VOCAB_PIDTYPE_POSITION, i, &value_);
        line_[i] = value_;
      }
      pids_out.push_back(line_);
    }
};

class Module : public yarp::os::RFModule {

private:

  std::unique_ptr<Reader> reader;

public:

  bool configure(yarp::os::ResourceFinder &rf) {

    auto remote = rf.check("remote", yarp::os::Value("/ergocubSim/head")).asString();
    auto filename = rf.check("file", yarp::os::Value("ecub_head.csv")).asString();  // log roll data
    double period = rf.check("period", yarp::os::Value(0.1)).asFloat64();
    int n_axes = 0;
    std::vector<std::string> s;

    if(rf.check("interfaces")) {
      yarp::os::Bottle * b = rf.find("interfaces").asList();

      for(uint8_t i; i < b->size(); ++i)
          s.push_back(b->get(i).asString());
    }
    else 
    {
      yError() << "interfaces not found";
      return false;
    }

    std::vector<int> ax;
    if(rf.check("axes_to_log")) {
      yarp::os::Bottle * b = rf.find("axes_to_log").asList();

      n_axes = b->size();
      yInfo() << "axes " << n_axes;
      for(uint8_t i; i < b->size(); ++i)
          ax.push_back(b->get(i).asInt32());
    }
    else 
    {
      yError() << "axes to log not found";
      return false;
    }

    yarp::os::Property conf;
    conf.put("device", "remote_controlboard");
    conf.put("remote", remote);
    conf.put("local", "/logger");

    reader = std::make_unique<Reader>(period, conf, filename, s, n_axes, ax);

    if(!reader->start())
      return false;

    yInfo() << "Started recording...";

    return true;
  }

  bool updateModule() {

    if(!reader->isRunning()) return false;

    yInfoThrottle(5) << "Recording data...";

    return true;
  }

  bool close() {

    yInfo() << "Catched SIGINT, saving data to file...";
    reader->stop();

    return true;
  }
};

/******************************************************************************/
int main(int argc, char* argv[]) {

  yarp::os::Network yarp;
  if (!yarp.checkNetwork()) {
    yError() << "Unable to find YARP server!";
    return EXIT_FAILURE;
  }

  yarp::os::ResourceFinder rf;
  rf.setDefaultContext("userloghead");
  rf.setDefaultConfigFile("config.ini");
  rf.configure(argc, argv);

  Module m;

  return m.runModule(rf);
}
