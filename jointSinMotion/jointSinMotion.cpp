
#include <cmath>
#include <string>

#include <yarp/os/all.h>
#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;


class jointControl: public RFModule {

    private:
        // BufferedPort<yarp::sig::Vector> outPort;
        BufferedPort<Bottle>  outPort;
        PolyDriver            jointDriver;
        IControlLimits       *ilim;
        IEncoders            *ienc;
        IControlMode         *imod;
        IPositionControl     *iposd;

        int joint;
        double period;
        double t0, t1;
        double min_lim, max_lim;
        double pos, des_pos, init_pos;
        double des_vel;
        double sin_period, sin_amplitude;

    public:
        bool configure(ResourceFinder &rf){
            // get simulation true or false
            int sim   = rf.check("simulation", Value(0)).asInt();

            // get parameters
            setName((rf.check("name", Value("/jointSinMotion")).asString()).c_str());
            std::string body_part = rf.check("body_part", Value("head")).asString();
            joint            = rf.check("joint", Value(2)).asInt();
            period           = rf.check("period", Value(0.002)).asDouble();
            sin_period       = rf.check("sin_period",Value(3.0)).asDouble();
            sin_amplitude    = rf.check("sin_amplitude",Value(15.0)).asDouble();

            // open communication with encoders
            Property optArm;
            optArm.put("device","remote_controlboard");
            optArm.put("local","/encReader/" + getName() + "/" + body_part);

            if (sim) {
                optArm.put("remote","/icubSim/" + body_part);
                if (!jointDriver.open(optArm)) {
                    yError() << "Unable to connect to /icubSim/" << body_part;
                    return false;
                }
            } else {
                optArm.put("remote","/icub/" + body_part);
                if (!jointDriver.open(optArm)) {
                    yError() << "Unable to connect to /icub/" << body_part;
                    return false;
                }
            }

            bool ok = true;
            ok = ok && jointDriver.view(ienc);
            ok = ok && jointDriver.view(ilim);
            ok = ok && jointDriver.view(imod);
            ok = ok && jointDriver.view(iposd);

            if (!ok){
                yError() << "Unable to open views";
                return false;
            }

            // open output port
            if (!outPort.open("/jointSinMotion/" + getName() + "/pos:o")) {
                yError() << "Unable to open jointSinMotion port";
                return false;
            }

            // compute joint limits
            ilim->getLimits(joint,&min_lim,&max_lim);
            yWarning() << "joint min limit: " << min_lim << " -   joint max limit: " << max_lim;

            // set control mode
            imod->setControlMode(0,VOCAB_CM_POSITION);
            imod->setControlMode(1,VOCAB_CM_POSITION);
            imod->setControlMode(2,VOCAB_CM_POSITION);


            // move joint to center
            init_pos = (max_lim + min_lim) / 2;
            iposd->setRefSpeed(joint,10);
            iposd->positionMove(joint,init_pos);
            yInfo() << "Waiting 3 seconds to reach init_pose";
            Time::delay(3);

            // check that amplitude !> motion allowed by the robot
            ienc->getEncoder(joint,&pos);
            double higher_lim = max_lim - pos;
            double lower_lim = min_lim - pos;
            yInfo() << init_pos << pos << higher_lim << lower_lim;
            if (higher_lim < sin_amplitude){
                sin_amplitude = higher_lim - 3; // the -3 is for avoiding joint limits
                yWarning() << "Amplitude not allowed by joint limits! -> Reduced in accordance with them to A =" << higher_lim-3;
            }else if (lower_lim > sin_amplitude){
                sin_amplitude = lower_lim + 3;
                yWarning() << "Amplitude not allowed by joint limits! -> Reduced in accordance with them to A =" << lower_lim+3;
            }
            // upper bound for sin period
            if (sin_period < 3.0) {
                sin_period = 3.0;
                yWarning() << "Period too short! -> Now set to a safer 3.0 seconds";
            }

            iposd->setRefSpeed(0,30);
            iposd->setRefSpeed(1,30);
            iposd->setRefSpeed(2,30);


            Time::delay(1);
            t0=t1=Time::now();

            return true;
        }

        bool updateModule(){
            // generate target and sed command
            double t = Time::now();
            des_pos = init_pos + sin_amplitude*sin(2.0*M_PI*(t-t0)/sin_period);
            joint = std::fmod(t-t0,27)/9;
            iposd->positionMove(joint,des_pos);

/*
            ienc->getEncoder(joint,&pos);
            double des_vel1 = init_pos + sin_amplitude*cos(2.0*M_PI*(t-t0)/sin_period);
            des_vel = (des_pos - pos)/period;
            iposd->getRefSpeed(joint, &des_vel);
            yInfo() << des_pos-pos;// << "   " << des_vel << "   " << des_vel1*2*M_PI/sin_period;
            iposd->setRefSpeed(joint,des_vel);
            double vel;
            ienc->getEncoderSpeed(joint,&vel);
*/

/*
            // write out
            Bottle& output = outPort.prepare();
            output.clear();
            output.addDouble(des_pos);
            output.addDouble(pos);
            output.addDouble(des_vel);
            output.addDouble(vel);
            outPort.write();
*/

            // print status with a 0.5s period
//            if (t-t1 >= 0.5){
//                yInfo()<<"+++++++++";
//                yInfo()<<"desired pos      [deg] = " << des_pos;
//                yInfo()<<"current pos      [deg] = " << pos;
//                yInfo()<<"---------";
//                t1=t;
//            }

            yInfo()<<t-t0<<" "<< joint<<  " "<<std::fmod(t-t0,27);



            return true;
        }

        double getPeriod(){
            return period;
        }

        bool interruptModule(){
            yInfo() << "Interrupting module ...";
            outPort.interrupt();
            return RFModule::interruptModule();
        }

        bool close(){
            yInfo() << "Closing the module...";
            jointDriver.close();
            outPort.close();
            yInfo() << "...done!";
            return RFModule::close();
        }

    };


    int main(int argc, char* argv[]) {
    Network yarp;
    if(!yarp.checkNetwork()) {
        yError() << "Network not found";
        return -1;
    }

    ResourceFinder rf;
    rf.configure(argc, argv);

    jointControl jCtrl;
    return jCtrl.runModule(rf);
    }
