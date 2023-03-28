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
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include <memory>

using namespace yarp::dev;
using namespace yarp::os;

class Reader : public PeriodicThread {
  private:

    PolyDriver driver;
    IEncoders * iEnc{ nullptr };
    IPidControl * iPidCtrl{ nullptr };
    IPositionControl * iPosCtrl{ nullptr };
    IMotorEncoders * iMEnc {nullptr};
    std::vector<std::string> interfaces_;
    Property conf_;
    double thread_period_;
    int n_axes_;
    std::string filename_;
    std::vector<double> time_vector;
    double t0;
    std::vector<std::vector<double>> joint_encoders;
    std::vector<std::vector<double>> motor_encoders;
    std::vector<std::vector<double>> pids_out;
    std::vector<std::vector<double>> pids_reference;
    std::vector<std::vector<double>> pids_error;
    Port port_fbk;
    Port port_ref;
    Port port_outs;
    Port port_mot_enc;
    Bottle bot_traj;
    Bottle bot_ref;
    Bottle bot_out;
    Bottle bot_mot_enc;
    std::vector<double> line_;
    std::ofstream fout;
    std::vector<int> axes_to_log_;
    std::vector<std::string> axes_names_;
    double value_;

  public:

    Reader(double period, const Property & conf, const std::string & filename,
            const std::vector<std::string> & interfaces, const int n_axes,
            const std::vector<int>& axes_indices, const std::vector<std::string>& axes_names);

    bool threadInit();

    std::stringstream constructLine(uint32_t i);

    void threadRelease();

    void run();
};