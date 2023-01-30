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

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <signal.h>

 /******************************************************************************/
struct DataExperiment {
  double pwm;
  double pid_out;
  double enc;
  double ref;
  double err;
  double target;
  double taskspace_enc;
};

constexpr int PITCH_JOINT_ID = 0;
constexpr int ROLL_JOINT_ID = 1;
constexpr int YAW_JOINT_ID = 2;
constexpr int CAM_JOINT_ID = 3;

class ReadThread : public yarp::os::RFModule {
private:
  yarp::dev::PolyDriver driver;
  yarp::dev::IEncoders* iEnc{ nullptr };
  yarp::dev::IPidControl* iPidCtrl{ nullptr };
  yarp::dev::IPositionControl* iPosCtrl{ nullptr };
  std::ofstream fout, pitch_fout;
  std::string remote;
  std::string fname;
  std::vector<DataExperiment> yaw_data;
  std::vector<DataExperiment> roll_data;
  std::vector<DataExperiment> pitch_data;
  std::vector<DataExperiment> cam_data;
  std::vector<double> time_vector;
  double period;
  double t0;

public:

  bool configure(yarp::os::ResourceFinder &rf) {
    yInfo() << "ReadThread starting";

    auto remote = rf.check("remote", yarp::os::Value("/ergocub/head")).asString();
    auto roll_filename = rf.check("file", yarp::os::Value("ecub_head.csv")).asString();  // log roll data
    period = rf.check("period", yarp::os::Value(0.03)).asFloat64();

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

    fout.open(roll_filename);

    if (!fout.is_open()) {
      yError() << "Failed to open " << roll_filename;
      driver.close();
      return false;
    }

    fout << "Time"<< ","
      << "Mot0_Pwm"<< ","<< "Mot1_Pwm"<< ","<< "Mot2_Pwm" << ","<< "Mot3_Pwm" << ","
      << "Mot0_Ref"<< ","<< "Mot1_Ref"<< ","<< "Mot2_Ref" << ","<< "Mot3_Ref" << ","
      << "Mot0_Err"<< ","<< "Mot1_Err"<< ","<< "Mot2_Err" << ","<< "Mot3_Err" << ","
      << "Roll,Pitch,Yaw,Cam" << std::endl;
    t0 = yarp::os::Time::now();

    yInfo() << "Started recording...";

    return true;
  }

  double getPeriod() {
    return period;
  }

  bool updateModule() {
    DataExperiment yaw_d;
    DataExperiment roll_d;
    DataExperiment pitch_d;
    DataExperiment cam_d;
    time_vector.push_back(static_cast<double>(yarp::os::Time::now() - t0));

    iEnc->getEncoder(YAW_JOINT_ID, &yaw_d.taskspace_enc);
    iEnc->getEncoder(ROLL_JOINT_ID, &roll_d.taskspace_enc);
    iEnc->getEncoder(PITCH_JOINT_ID, &pitch_d.taskspace_enc);
    iEnc->getEncoder(CAM_JOINT_ID, &cam_d.taskspace_enc);

    iPidCtrl->getPidReference(yarp::dev::VOCAB_PIDTYPE_POSITION, YAW_JOINT_ID,
      &yaw_d.ref);
    iPidCtrl->getPidReference(yarp::dev::VOCAB_PIDTYPE_POSITION, ROLL_JOINT_ID,
      &roll_d.ref);
    iPidCtrl->getPidReference(yarp::dev::VOCAB_PIDTYPE_POSITION, PITCH_JOINT_ID,
      &pitch_d.ref);
    iPidCtrl->getPidReference(yarp::dev::VOCAB_PIDTYPE_POSITION, CAM_JOINT_ID,
      &cam_d.ref);

    if (!iPosCtrl->getTargetPosition(YAW_JOINT_ID, &yaw_d.target))
      yWarning() << "Cannot retrieve yaw tg";
    if (!iPosCtrl->getTargetPosition(ROLL_JOINT_ID, &roll_d.target))
      yWarning() << "Cannot retrieve roll tg";
    if (!iPosCtrl->getTargetPosition(PITCH_JOINT_ID, &pitch_d.target))
      yWarning() << "Cannot retrieve pitch tg";
    if (!iPosCtrl->getTargetPosition(CAM_JOINT_ID, &cam_d.target))
      yWarning() << "Cannot retrieve cam tg";

    iPidCtrl->getPidError(yarp::dev::VOCAB_PIDTYPE_POSITION, YAW_JOINT_ID, &yaw_d.err);
    iPidCtrl->getPidError(yarp::dev::VOCAB_PIDTYPE_POSITION, ROLL_JOINT_ID, &roll_d.err);
    iPidCtrl->getPidError(yarp::dev::VOCAB_PIDTYPE_POSITION, PITCH_JOINT_ID, &pitch_d.err);
    iPidCtrl->getPidError(yarp::dev::VOCAB_PIDTYPE_POSITION, CAM_JOINT_ID, &cam_d.err);

    iPidCtrl->getPidOutput(yarp::dev::VOCAB_PIDTYPE_POSITION, YAW_JOINT_ID, &yaw_d.pwm);
    iPidCtrl->getPidOutput(yarp::dev::VOCAB_PIDTYPE_POSITION, ROLL_JOINT_ID, &roll_d.pwm);
    iPidCtrl->getPidOutput(yarp::dev::VOCAB_PIDTYPE_POSITION, PITCH_JOINT_ID, &pitch_d.pwm);

    yaw_data.push_back(std::move(yaw_d));
    roll_data.push_back(std::move(roll_d));
    pitch_data.push_back(std::move(pitch_d));
    cam_data.push_back(std::move(cam_d));

    yInfoThrottle(10) << "Recording data...";

    return true;
  }

  bool close() {

    yInfo() << "Catched SIGINT, saving data to file...";


    for (size_t i = 0; i < roll_data.size(); ++i) {
      fout << time_vector[i] << ","
        << yaw_data[i].pwm << "," << roll_data[i].pwm << "," << pitch_data[i].pwm << "," << cam_data[i].pwm << ","
        << yaw_data[i].ref << "," << roll_data[i].ref << "," << pitch_data[i].ref << "," << cam_data[i].ref << "," 
        << yaw_data[i].err << "," << roll_data[i].err << "," << pitch_data[i].err << "," << cam_data[i].err << "," 
        << roll_data[i].taskspace_enc << "," << pitch_data[i].taskspace_enc << ","  << yaw_data[i].taskspace_enc << "," << cam_data[i].taskspace_enc 
        << std::endl;
    }

    // Close I/O resources
    fout.close();
    // pitch_fout.close();
    driver.close();

    yInfo() << "Done.";

    return true;
  }
};

/****************************************************************q`**************/
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
