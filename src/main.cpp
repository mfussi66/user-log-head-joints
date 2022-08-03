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
#include <yarp/os/Time.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <signal.h>

 /******************************************************************************/
struct DataExperiment {
  double t;
  double pwm;
  double pid_out;
  double enc;
  double ref;
  double err;
  double target;
};

constexpr int PITCH_JOINT_ID = 0;
constexpr int ROLL_JOINT_ID = 1;

bool TERMINATED = false;

/******************************************************************************/
// The function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
  // Terminate program
  TERMINATED = true;
}

class ReadThread : public yarp::os::PeriodicThread {
private:
  yarp::dev::PolyDriver driver;
  yarp::dev::IEncoders* iEnc{ nullptr };
  yarp::dev::IPidControl* iPidCtrl{ nullptr };
  yarp::dev::IPositionControl* iPosCtrl{ nullptr };
  std::ofstream roll_fout, pitch_fout;
  std::string remote;
  std::vector<DataExperiment> yaw_data;
  std::vector<DataExperiment> roll_data;
  std::vector<DataExperiment> pitch_data;
  double t0;

public:
  ReadThread(yarp::conf::float64_t period, std::string remote_port)
    : PeriodicThread(period) {
    remote = remote_port;
  }

  bool threadInit() {
    yInfo() << "ReadThread starting";

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

    roll_fout.open("logfile.log");

    if (!roll_fout.is_open()) {
      yError() << "Failed to open "
        << "logfile.log";
      driver.close();
      return false;
    }

    roll_fout << "Time"<< ","<< "Mot1"<< ","<< "Mot2"<< ","<< "Mot3"<< ","
      << "Mot1_Ref"<< ","<< "Mot2_Ref"<< ","<< "Mot3_Ref"<< ","
      << "Mot1_Err"<< ","<< "Mot2_Err"<< ","<< "Mot3_Err" << std::endl;

    t0 = yarp::os::Time::now();

    return true;
  }

  void run() {
    DataExperiment yaw_d;
    DataExperiment roll_d;
    DataExperiment pitch_d;
    auto delta_t = yarp::os::Time::now() - t0;
    roll_d.t = pitch_d.t = delta_t;
    yaw_d.t = delta_t;

    iEnc->getEncoder(0, &yaw_d.enc);
    iEnc->getEncoder(1, &roll_d.enc);
    iEnc->getEncoder(2, &pitch_d.enc);

    iPidCtrl->getPidReference(yarp::dev::VOCAB_PIDTYPE_POSITION, 0,
      &yaw_d.ref);
    iPidCtrl->getPidReference(yarp::dev::VOCAB_PIDTYPE_POSITION, 1,
      &roll_d.ref);
    iPidCtrl->getPidReference(yarp::dev::VOCAB_PIDTYPE_POSITION, 2,
      &pitch_d.ref);

    if (!iPosCtrl->getTargetPosition(0, &yaw_d.target))
      yWarning() << "Cannot retrieve yaw tg";
    if (!iPosCtrl->getTargetPosition(1, &roll_d.target))
      yWarning() << "Cannot retrieve roll tg";
    if (!iPosCtrl->getTargetPosition(2, &pitch_d.target))
      yWarning() << "Cannot retrieve pitch tg";

    iPidCtrl->getPidError(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, &yaw_d.err);
    iPidCtrl->getPidError(yarp::dev::VOCAB_PIDTYPE_POSITION, 1, &roll_d.err);
    iPidCtrl->getPidError(yarp::dev::VOCAB_PIDTYPE_POSITION, 2, &pitch_d.err);

    iPidCtrl->getPidOutput(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, &yaw_d.pwm);
    iPidCtrl->getPidOutput(yarp::dev::VOCAB_PIDTYPE_POSITION, 1, &roll_d.pwm);
    iPidCtrl->getPidOutput(yarp::dev::VOCAB_PIDTYPE_POSITION, 2, &pitch_d.pwm);

    yaw_data.push_back(std::move(yaw_d));
    roll_data.push_back(std::move(roll_d));
    pitch_data.push_back(std::move(pitch_d));
  }

  void threadRelease() {
    for (uint32_t i = 0; i < roll_data.size(); ++i) {
      roll_fout << yaw_data[i].t << "," << yaw_data[i].pwm << ","
        << roll_data[i].pwm << "," << pitch_data[i].pwm << ","
        << yaw_data[i].ref << "," << roll_data[i].ref << ","
        << pitch_data[i].ref << "," << yaw_data[i].err << ","
        << roll_data[i].err << "," << pitch_data[i].err << std::endl;
    }
    yInfo() << "Saved data";

    // Close I/O resources
    roll_fout.close();
    // pitch_fout.close();
    driver.close();

    yInfo() << "Done.";
  }
};

/******************************************************************************/
int main(int argc, char* argv[]) {
  // Register signal and signal handler
  signal(SIGINT, signal_callback_handler);

  yarp::os::Network yarp;
  if (!yarp.checkNetwork()) {
    yError() << "Unable to find YARP server!";
    return EXIT_FAILURE;
  }

  yarp::os::ResourceFinder rf;
  rf.configure(argc, argv);

  auto remote = rf.check("remote", yarp::os::Value("/nwa/wrist_mc")).asString();
  auto roll_filename = rf.check("filename", yarp::os::Value("yaw_cmd.log"))
    .asString();  // log roll data
  auto period = rf.check("period", yarp::os::Value(0.01)).asFloat64();

  ReadThread read_thread(period, remote);

  yInfo() << "Started recording...";
  read_thread.start();

  while (!TERMINATED) {
    yInfo() << "Recording data...";
    yarp::os::Time::delay(5);
  }

  if (TERMINATED) {
    yInfo() << "Catched SIGINT, saving data to file...";
    read_thread.stop();
  }

  return EXIT_SUCCESS;
}
