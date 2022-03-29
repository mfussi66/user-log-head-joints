/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <cstdlib>
#include <string>
#include <limits>
#include <vector>
#include <fstream>
#include <iostream>
#include <utility>
#include <algorithm>
#include <unistd.h>
#include <cstdlib>
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

  auto remote = rf.check("remote", yarp::os::Value("/icub/head")).asString();
  auto roll_filename  = rf.check("file-name-roll", yarp::os::Value("roll_output.log")).asString();  // log roll data
  auto pitch_filename = rf.check("file-name-pitch", yarp::os::Value("pitch_output.log")).asString();  // log pitch data
  auto Ts = rf.check("Ts", yarp::os::Value(.01)).asFloat64();

  yarp::dev::PolyDriver driver;
  yarp::dev::IEncoders* iEnc{nullptr};

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

  if (!(driver.view(iPosCtrl))) {
    yError() << "Failed to open position control interface";
    driver.close();
    return EXIT_FAILURE;
  }

  // Open I/O resources 
  std::ofstream roll_fout, pitch_fout;
  roll_fout.open(roll_filename.c_str());
  pitch_fout.open(pitch_filename.c_str());

  if (!roll_fout.is_open()) {
    yError() << "Failed to open" << roll_filename;
    driver.close();
    return EXIT_FAILURE;
  }

  if (!pitch_fout.is_open()) {
    yError() << "Failed to open" << pitch_filename;
    driver.close();
    return EXIT_FAILURE;
  }


  std::vector<DataExperiment> roll_data;
  std::vector<DataExperiment> pitch_data;
  auto check_conditions{0};
  auto t0 = yarp::os::Time::now();

  // Log data only
  while(!TERMINATED){
    DataExperiment roll_d;
    DataExperiment pitch_d;
    auto delta_t = yarp::os::Time::now() - t0; 
    roll_d.t = pitch_d.t = delta_t;

    // roll
    iEnc->getEncoder(ROLL_JOINT_ID, &roll_d.enc);
    
    // pitch
    iEnc->getEncoder(PITCH_JOINT_ID, &pitch_d.enc);

    iPosCtrl->getTargetPosition(ROLL_JOINT_ID, &roll_d.ref);
    iPosCtrl->getTargetPosition(PITCH_JOINT_ID, &pitch_d.ref);

    yInfo() << "Pitch ref: " << pitch_d.ref << " enc: " << pitch_d.enc << "; Roll ref: " << roll_d.ref << " enc: " << roll_d.enc;

    // save data
    roll_data.push_back(std::move(roll_d));
    pitch_data.push_back(std::move(pitch_d));

    yarp::os::Time::delay(Ts);
  }

  if(TERMINATED)
    yInfo() << "Catched SIGINT, saving data to file...";
    
    
  // write down roll data
  for (const auto & d : roll_data) {
    roll_fout << d.t << ","
        << d.pid_out << ","
        << d.pwm << ","
        << d.enc << ","
        << d.ref << std::endl;
  }
  
  yInfo() << "Saved roll data";

  // write down pitch data
  for (const auto & d : pitch_data) {
      pitch_fout << d.t << ","
        << d.pid_out << ","
        << d.pwm << ","
        << d.enc << ","
        << d.ref << std::endl;
  }
  
  yInfo() << "Saved pitch data";

  // Close I/O resources 
  roll_fout.close();
  pitch_fout.close();
  driver.close();

  yInfo() << "Done.";


  return EXIT_SUCCESS;
}
