/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <signal.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPWMControl.h>
#include <yarp/dev/IPidControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PidEnums.h>


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
   TERMINATED=true;
}


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
  auto roll_filename  = rf.check("filename", yarp::os::Value("yaw_cmd.log")).asString();  // log roll data
  // auto pitch_filename = rf.check("file-name-pitch", yarp::os::Value("pitch_output.log")).asString();  // log pitch data
  auto Ts = rf.check("Ts", yarp::os::Value(0.001)).asFloat64();

  yarp::dev::PolyDriver driver;
  yarp::dev::IEncoders* iEnc{nullptr};

  yarp::dev::IPidControl* iPidCtrl{nullptr};
  yarp::dev::IPositionControl* iPosCtrl{nullptr};

  yarp::os::Property conf;
  conf.put("device", "remote_controlboard");
  conf.put("remote", remote);
  conf.put("local", "/logger");

  if (!driver.open(conf)) {
    yError() << "Failed to connect to" << remote;
    return EXIT_FAILURE;
  }

  if (!(driver.view(iEnc))) {
    yError() << "Failed to open encoder interface";
    driver.close();
    return EXIT_FAILURE;
  }

  if (!(driver.view(iPidCtrl))) {
    yError() << "Failed to open PID control interface";
    driver.close();
    return EXIT_FAILURE;
  }

  if (!(driver.view(iPosCtrl))) {
    yError() << "Failed to open position control interface";
    driver.close();
    return EXIT_FAILURE;
  }
  // Open I/O resources 
  std::ofstream roll_fout, pitch_fout;
  roll_fout.open(roll_filename.c_str());
  //pitch_fout.open(pitch_filename.c_str());

  if (!roll_fout.is_open()) {
    yError() << "Failed to open" << roll_filename;
    driver.close();
    return EXIT_FAILURE;
  }

  // Write column names

  roll_fout << "Time" << ","
        << "Mot1" << ","
        << "Mot2" << ","
        << "Mot3" << ","
        << "Mot1_Ref" << ","
        << "Mot2_Ref" << ","
        << "Mot3_Ref" << ","
        << "Mot1_Err" << ","
        << "Mot2_Err" << ","
        << "Mot3_Err" << std::endl;

  // if (!pitch_fout.is_open()) {
  //   yError() << "Failed to open" << pitch_filename;
  //   driver.close();
  //   return EXIT_FAILURE;
  // }

  std::vector<DataExperiment> yaw_data;
  std::vector<DataExperiment> roll_data;
  std::vector<DataExperiment> pitch_data;
  auto check_conditions{0};
  auto t0 = yarp::os::Time::now();
  yInfo() << "Started recording...";
  // Log data only
  while(!TERMINATED){
    DataExperiment yaw_d;
    DataExperiment roll_d;
    DataExperiment pitch_d;
    auto delta_t = yarp::os::Time::now() - t0; 
    roll_d.t = pitch_d.t = delta_t;
    yaw_d.t = delta_t;

    iEnc->getEncoder(0, &yaw_d.enc);
    iEnc->getEncoder(1, &roll_d.enc);
    iEnc->getEncoder(2, &pitch_d.enc);

    iPidCtrl->getPidReference(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, &yaw_d.ref);
    iPidCtrl->getPidReference(yarp::dev::VOCAB_PIDTYPE_POSITION, 1, &roll_d.ref);
    iPidCtrl->getPidReference(yarp::dev::VOCAB_PIDTYPE_POSITION, 2, &pitch_d.ref);

    if(!iPosCtrl->getTargetPosition(0, &yaw_d.target)) yWarning() << "Cannot retrieve yaw tg";
    if(!iPosCtrl->getTargetPosition(1, &roll_d.target))  yWarning() << "Cannot retrieve roll tg";
    if(!iPosCtrl->getTargetPosition(2, &pitch_d.target))  yWarning() << "Cannot retrieve pitch tg";

    iPidCtrl->getPidError(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, &yaw_d.err);
    iPidCtrl->getPidError(yarp::dev::VOCAB_PIDTYPE_POSITION, 1, &roll_d.err);
    iPidCtrl->getPidError(yarp::dev::VOCAB_PIDTYPE_POSITION, 2, &pitch_d.err);

    iPidCtrl->getPidOutput(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, &yaw_d.pwm);
    iPidCtrl->getPidOutput(yarp::dev::VOCAB_PIDTYPE_POSITION, 1, &roll_d.pwm);
    iPidCtrl->getPidOutput(yarp::dev::VOCAB_PIDTYPE_POSITION, 2, &pitch_d.pwm);

    //yInfo() << "Roll PID ref: " << roll_d.ref << " PID err: " << roll_d.err << "pwm: " << roll_d.pwm  << " enc: " << roll_d.enc;
    //yInfo() << "Pitch PID ref: " << pitch_d.ref << " PID err: " << pitch_d.err << "pwm: " << pitch_d.pwm << " enc: " << pitch_d.enc;
    //yInfo() << "Yaw PID ref: " << yaw_d.ref << " PID err: " << yaw_d.err << "pwm: " << yaw_d.pwm << " enc: " << yaw_d.enc;

    // save data
    yaw_data.push_back(std::move(yaw_d));
    roll_data.push_back(std::move(roll_d));
    pitch_data.push_back(std::move(pitch_d));

    yarp::os::Time::delay(Ts);
  }

  if(TERMINATED)
    yInfo() << "Catched SIGINT, saving data to file...";
    

  for(uint32_t i = 0; i < roll_data.size(); ++i) {
    roll_fout << yaw_data[i].t << ","
              << yaw_data[i].pwm << ","
              << roll_data[i].pwm << ","
              << pitch_data[i].pwm << ","
              << yaw_data[i].ref << ","
              << roll_data[i].ref << ","
              << pitch_data[i].ref << ","
              << yaw_data[i].err << ","
              << roll_data[i].err << ","
              << pitch_data[i].err << std::endl;
  }
    
  // write down roll data
  // for (const auto & d : roll_data) {
  //   roll_fout << d.t << ","
  //       << d.pid_out << ","
  //       << d.pwm << ","
  //       << d.enc << ","
  //       << d.ref << ","
  //       << d.err << std::endl;
  // }
  
  yInfo() << "Saved roll data";

  // write down pitch data
  // for (const auto & d : pitch_data) {
  //     pitch_fout << d.t << ","
  //       << d.pid_out << ","
  //       << d.pwm << ","
  //       << d.enc << ","
  //       << d.ref << ","
  //       << d.err << std::endl;
  // }

  // yInfo() << "Saved pitch data";

  // Close I/O resources 
  roll_fout.close();
  // pitch_fout.close();
  driver.close();

  yInfo() << "Done.";


  return EXIT_SUCCESS;
}
