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
#include <signal.h>


using namespace yarp::sig;

 /******************************************************************************/

class ReadThread : public yarp::os::RFModule {

private:
  yarp::dev::PolyDriver driver;
  yarp::dev::IEncoders * iEnc{ nullptr };
  yarp::dev::IPidControl * iPidCtrl{ nullptr };
  yarp::dev::IPositionControl * iPosCtrl{ nullptr };
  yarp::dev::IMotorEncoders * iMEnc {nullptr};

  std::vector<std::vector<double>> joint_encoders;
  std::vector<std::vector<double>> motor_encoders;
  std::vector<std::vector<double>> pids_out;
  std::vector<std::vector<double>> pids_reference;
  std::vector<std::vector<double>> pids_error;

  std::vector<double> time_vector;

  std::ofstream fout;

  double period;
  double t0;
  int n_axes;

public:

  bool configure(yarp::os::ResourceFinder &rf) {

    auto remote = rf.check("remote", yarp::os::Value("/ergocub/head")).asString();
    auto filename = rf.check("file", yarp::os::Value("ecub_head.csv")).asString();  // log roll data
    period = rf.check("period", yarp::os::Value(0.1)).asFloat64();

    yarp::os::Property conf;
    conf.put("device", "remote_controlboard");
    conf.put("remote", remote);
    conf.put("local", "/logger");

    if (!driver.open(conf)) {
      yError() << "Failed to connect to" << remote;
      return false;
    }

    if (!(driver.view(iEnc))) {
      yError() << "Failed to open encoder interface";
      driver.close();
      return false;
    }

    if (iEnc->getAxes(&n_axes)) {
      joint_encoders.resize(1, std::vector<double>(n_axes));
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

    if (iPosCtrl->getAxes(&n_axes)) {
      motor_encoders.resize(1, std::vector<double>(n_axes));
      pids_reference.resize(1, std::vector<double>(n_axes));
      pids_error.resize(1, std::vector<double>(n_axes));
      pids_out.resize(1, std::vector<double>(n_axes));
    } else {
      yError() << "Could not get the number of joint_encoders!";
      return false;
    }

    fout.open(filename);

    if (!fout.is_open()) {
      yError() << "Failed to open " << filename;
      driver.close();
      return false;
    }

    fout << "Time"<< ","
      << "Roll, Pitch, Yaw, Cam,"
      << "Mot0_Enc"<< ","<< "Mot1_Enc"<< ","<< "Mot2_Enc" << ","<< "Mot3_Enc" << ","
      << "Mot0_Pwm"<< ","<< "Mot1_Pwm"<< ","<< "Mot2_Pwm" << ","<< "Mot3_Pwm" << ","
      << "Mot0_Ref"<< ","<< "Mot1_Ref"<< ","<< "Mot2_Ref" << ","<< "Mot3_Ref" << ","
      << "Mot0_Err"<< ","<< "Mot1_Err"<< ","<< "Mot2_Err" << ","<< "Mot3_Err" << std::endl;
    t0 = yarp::os::Time::now();

    yInfo() << "Started recording...";

    return true;
  }

  double getPeriod() {
    return period;
  }

  bool updateModule() {

    time_vector.push_back(static_cast<double>(yarp::os::Time::now() - t0));

    std::vector<double> line;
    line.resize(n_axes);

    iEnc->getEncoders(line.data());
    joint_encoders.push_back(line);

    iMEnc->getMotorEncoders(line.data());
    motor_encoders.push_back(line);

    iPidCtrl->getPidReferences(yarp::dev::VOCAB_PIDTYPE_POSITION, line.data());
    pids_reference.push_back(line);

    iPidCtrl->getPidErrors(yarp::dev::VOCAB_PIDTYPE_POSITION, line.data());
    pids_error.push_back(line);

    iPidCtrl->getPidOutputs(yarp::dev::VOCAB_PIDTYPE_POSITION, line.data());
    pids_out.push_back(line);

    yInfoThrottle(10) << "Recording data...";

    return true;
  }

  std::stringstream writeLine(uint32_t i) {

    std::stringstream line;

    line << time_vector[i];

    for(auto & e : joint_encoders[i])
      line << "," << e;
    for(auto & e : motor_encoders[i])
      line << "," << e;
    for(auto & e : pids_reference[i])
      line << "," << e;
    for(auto & e : pids_error[i])
      line << "," << e;
    for(auto & e : pids_out[i])
      line << "," << e;

    line << std::endl;

    return line;
  }

  bool close() {

    yInfo() << "Catched SIGINT, saving data to file...";

    for(uint32_t i = 0; i < time_vector.size(); ++i) {
      fout << writeLine(i).str();
    }

    // Close I/O resources
    fout.close();
    driver.close();

    yInfo() << "Done.";

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

  ReadThread read_thread;

  return read_thread.runModule(rf);
}
