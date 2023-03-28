/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "Reader.h"

using namespace yarp::os;

class Module : public RFModule {

private:

  std::unique_ptr<Reader> reader;

public:

  bool configure(ResourceFinder &rf) {

    auto remote = rf.check("remote", Value("/icub/head")).asString();
    auto filename = rf.check("file", Value("file.csv")).asString();  // log roll data
    double period = rf.check("period", Value(0.1)).asFloat64();
    int n_axes = 0;
    std::vector<std::string> s;

    if(rf.check("interfaces")) {
      Bottle * b = rf.find("interfaces").asList();

      for(uint8_t i; i < b->size(); ++i)
          s.push_back(b->get(i).asString());
    }
    else 
    {
      yError() << "interfaces not found";
      return false;
    }

    std::vector<int> axes_indices;
    if(rf.check("axes_indices")) {
      Bottle * b = rf.find("axes_indices").asList();

      n_axes = b->size();
      for(uint8_t i; i < b->size(); ++i)
          axes_indices.push_back(b->get(i).asInt32());
    }
    else
    {
      yError() << "axes to log not found";
      return false;
    }

    std::vector<std::string> axes_names;
    if(rf.check("axes_names")) {
      Bottle * b = rf.find("axes_names").asList();

      if(b->size() != axes_indices.size()) {
        yError() << "Axes indices and names sizes are not consistent!";
        return false;
      }

      for(uint8_t i; i < b->size(); ++i)
          axes_names.push_back(b->get(i).asString());
    }
    else
    {
      yError() << "axes to log not found";
      return false;
    }

    Property conf;
    conf.put("device", "remote_controlboard");
    conf.put("remote", remote);
    conf.put("local", "/logger");

    reader = std::make_unique<Reader>(period, conf, filename, s, n_axes, axes_indices, axes_names);

    if(!reader->start())
      return false;

    yInfo() << "Started recording...";

    return true;
  }

  bool updateModule() {

    if(!reader->isRunning()) return false;

    yInfoThrottle(10) << "Recording data...";

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

  Network yarp;
  if (!yarp.checkNetwork()) {
    yError() << "Unable to find YARP server!";
    return EXIT_FAILURE;
  }

  ResourceFinder rf;
  rf.setDefaultContext("controlboardlogger");
  rf.setDefaultConfigFile("config.ini");
  rf.configure(argc, argv);

  Module m;

  return m.runModule(rf);
}
