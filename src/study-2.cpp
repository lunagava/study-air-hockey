/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <cmath>
#include <limits>
#include <iterator>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>
#include <fstream>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IEncoders.h>

#include <iCub/ctrl/minJerkCtrl.h>

#include "../../spline/src/spline.h"
#include "matplotlib-cpp/matplotlibcpp.h"
#include "linearTraj.h"

#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/os/all.h>
#include <event-driven/all.h>

using namespace ev;

namespace plt = matplotlibcpp;

/************************************************************************/
class ControllerModule: public yarp::os::RFModule, public yarp::os::Thread
{
    std::vector<yarp::dev::PolyDriver> drv{3};
    std::vector<yarp::dev::IPositionControl*> ipos{3};
    std::vector<yarp::dev::IPositionDirect*> iposd{3};

    yarp::dev::PolyDriver drv_arm;
    yarp::dev::ICartesianControl* arm;

    yarp::dev::PolyDriver drv_gaze;
    yarp::dev::IGazeControl* gaze;

    yarp::dev::IEncoders *ienc;

    std::string table_file;
    std::vector<std::shared_ptr<tk::spline>> interp;
    double y_min, y_max;
    int whichImagePlane;

    int u,v, puck_tracked;
    double pix_stamp;
    yarp::sig::Vector puck_rf{0.,0.,0.};
    std::vector<double> fingers_posture;
    std::string which_arm, robot;
    double velTraj;

    std::ofstream myFile1, myFile2;
    double puck_velocity;

    deque<LabelledAE> out_queue;
    LabelledAE ev;

    vWritePort output_port;
    yarp::os::BufferedPort<yarp::os::Bottle> targetPort;
    yarp::os::BufferedPort<yarp::os::Bottle> yposPort, puckPort, hand_pix_port;
    yarp::os::RpcServer handlerPort;

    linearTraj genLinTraj;

    double target;
    bool reached;

    bool mobility_condition;

    /********************************************************************/
    void helperOpenDevice(const int i, const std::string& device_name) {
        yarp::os::Property options;
        options.put("device", "remote_controlboard");
        options.put("remote", "/"+ robot +"/" + device_name);
        options.put("local", getName()+"/" + device_name);
        drv[i].open(options);

        yarp::dev::IControlMode* imod;
        yarp::dev::IControlLimits* ilim;
        drv[i].view(ipos[i]);
        drv[i].view(iposd[i]);
        drv[i].view(imod);
        drv[i].view(ilim);
        drv[i].view(ienc);
        int naxes;
        ipos[i]->getAxes(&naxes);
        std::vector<int> modes(naxes, VOCAB_CM_POSITION);
        std::vector<double> vels(naxes, 10.);
        std::vector<double> accs(naxes, std::numeric_limits<double>::max());
        std::vector<double> poss(naxes, 0.);
        imod->setControlModes(modes.data());
        ipos[i]->setRefSpeeds(vels.data());
        ipos[i]->setRefAccelerations(accs.data());
//        if (naxes==16){
//            double pinkie_min, pinkie_max;
//            ilim -> getLimits(15, &pinkie_min, &pinkie_max);
//            fingers_posture={60., 80., 40., 35., 40., 35., 40., 35., pinkie_max};
//        }
        switch (naxes) {
            case 3:
                for (size_t i = 0; i < 3; i++) {
                    poss[2 - i] = interp[i]->operator()(0.);
                }
                break;
            case 16:
                for (size_t i = 0; i < 7; i++) {
                    poss[i] = interp[3 + i]->operator()(0.);
                }
//                for (size_t i = 7; i < 16; i++){
//                    poss[i] = fingers_posture[i-7]; //for grasping the paddle
//                }
                break;
            case 6:
                for (size_t i = 0; i < 6; i++) {
                    poss[i] = interp[3 + 7 + i]->operator()(0.);
                }
                break;
        }
        ipos[i]->positionMove(poss.data());
        auto done = false;
        while(!done) {
            yarp::os::Time::delay(1.);
            ipos[i]->checkMotionDone(&done);
        }

        std::fill(begin(modes), end(modes), VOCAB_CM_POSITION_DIRECT);
        imod->setControlModes(modes.data());
    }

    /********************************************************************/
    auto readTable() {
        std::ifstream fin(table_file);
        if (fin.is_open()) {
            std::vector<double> y;
            std::vector<std::vector<double>> q(16);
            double read;
            while (!fin.eof()) {
                fin >> read;
                y.push_back(read);
                for (size_t j = 0; j < q.size(); j++) {
                    fin >> read;
                    q[j].push_back(read);
                }
            }
            y_min = *std::min_element(begin(y), end(y));
            y_max = *std::max_element(begin(y), end(y));
            for (const auto& q_:q) {
                interp.push_back(std::make_shared<tk::spline>(y, q_));
            }
            fin.close();
            yInfo() << "Table successfully read from" << table_file;
            return true;
        } else {
            yError() << "Unable to read from file" << table_file;
            return false;
        }
    }

    /********************************************************************/
    auto helperWaitDevice(yarp::dev::PolyDriver& driver,
                          const yarp::os::Property& options,
                          const std::string& device_name) {
        const auto t0 = yarp::os::Time::now();
        while (yarp::os::Time::now() - t0 < 10.) {
            if (driver.open(const_cast<yarp::os::Property&>(options))) {
                return true;
            }
            yarp::os::Time::delay(1.);
        }

        yError() << "Unable to open the Device Driver:" << device_name;
        return false;
    }

    yarp::sig::Vector projectToVisualSpace(const yarp::sig::Vector robPos){
        yarp::sig::Vector imagePos;
        imagePos.resize(2);

        gaze->get2DPixel(whichImagePlane, robPos, imagePos); // project the hand Cartesian position into the image plane

//        std::cout<<"get2DPixel results: "<<imagePos[0]<<" "<<imagePos[1]<<std::endl;

        return imagePos;
    }

    yarp::sig::Vector projectToRobotSpace(int u_puck, int v_puck) {

        yarp::sig::Vector puckPix_vec, puckPos_rob;
        puckPix_vec.resize(2); puckPos_rob.resize(3);
        puckPix_vec[0] = u_puck;
        puckPix_vec[1] = v_puck;

        yarp::sig::Vector plane{0., 0., 1.0, 0.09};

        // useful to know the position wrt robot reference frame without knowing the plane on which the puck is moving
        gaze->get3DPointOnPlane(whichImagePlane, puckPix_vec, plane, puckPos_rob);

        // defensive movements along y direction (x and z are set equal to the initial Cartesian position)
        puckPos_rob[0] = -0.3;
        puckPos_rob[2] = 0;

        return puckPos_rob;
    }

    /********************************************************************/
    bool configure(yarp::os::ResourceFinder& rf) override {
        table_file = rf.findFile("table-file");
        velTraj= rf.check("vel", yarp::os::Value(0.3)).asDouble();
        whichImagePlane = rf.check("camera", yarp::os::Value(0)).asInt(); // 0 for left and 1 for right
        y_min = rf.check("y_min", yarp::os::Value(-0.12)).asDouble();
        y_max = rf.check("y_max", yarp::os::Value(0.23)).asDouble();

        if (!readTable()) {
            return false;
        }

        setName((rf.check("name", yarp::os::Value("/study-air-hockey")).asString()).c_str());
        robot = rf.check("robot", yarp::os::Value("icubSim")).asString();
        which_arm = rf.check("arm", yarp::os::Value("left_arm")).asString();

        helperOpenDevice(0, "torso");
        helperOpenDevice(1, which_arm);
        helperOpenDevice(2, "head");

        yarp::os::Property options_arm;
        options_arm.put("device", "cartesiancontrollerclient");
        options_arm.put("remote", "/" + robot + "/cartesianController/" + which_arm);
        options_arm.put("local", getName()+"/cartesian/" + which_arm);
        if (!helperWaitDevice(drv_arm, options_arm, "Cartesian Controller")) {
            return false;
        }

        yarp::os::Property options_gaze;
        options_gaze.put("device", "gazecontrollerclient");
        options_gaze.put("remote", "/iKinGazeCtrl");
        options_gaze.put("local", getName()+"/gaze");
        if (!helperWaitDevice(drv_gaze, options_gaze, "Gaze Controller")) {
            drv_arm.close();
            return false;
        }

        drv_arm.view(arm);
        drv_gaze.view(gaze);

        targetPort.open(getName() + "/target");
        yposPort.open(getName() + "/ypos");
        puckPort.open(getName() + "/puck");
        hand_pix_port.open(getName() + "/hand-pixels");

        genLinTraj.init(velTraj, getPeriod());

        // rpc = remote procedure call
        // open rpc port
        if(!handlerPort.open(getName() + "/" + which_arm + "/rpc:i")) {
            yError() << "Could not open rpc port";
            return false;
        }

        // attach the callback respond()
        attach(handlerPort);

        myFile1.open("/code/luna/study-air-hockey/COM_pix.csv");
        if (!myFile1.is_open())
        {
            yError()<<"Could not open file for printing COMs";
            return -1;
        }

        myFile2.open("/code/luna/study-air-hockey/COM_robot.csv");
        if (!myFile2.is_open())
        {
            yError()<<"Could not open file for printing COMs";
            return -1;
        }

        if(!output_port.open(getName() + "/LAE:o"))
            return false;

        return Thread::start();
    }

    void run() {

//        std::cout << "inside RUN"<<std::endl;

        while (Thread::isRunning()) {

//            std::cout << "inside LOOP"<<std::endl;
            yarp::os::Bottle *b = puckPort.read();
            u = b->get(0).asInt(); // x coordinate of the pixel within the image plane
            v = b->get(1).asInt(); // y coordinate of the pixel within the image plane
            pix_stamp = b->get(2).asInt();
//            puck_tracked = b->get(2).asInt(); //verify if the target is tracked or not
//            puck_velocity = b->get(3).asDouble(); // puck velocity along y
            std::cout << "u_puck = " << u << ", v_puck = " << v << std::endl;

        }
    }

    bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
    {

        std::string cmd=command.get(0).asString();
        yInfo() << cmd;
        if (cmd=="help")
        {
            reply.addVocab(yarp::os::Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- move (move to the desired pose)");
            reply.addString("- quit");
        }
        else if (cmd=="move"){
            yarp::sig::Vector desPos, desOrient, xc, oc;

            desPos.resize(3); desOrient.resize(4);

            desPos[0]=command.get(1).asDouble();
            desPos[1]=command.get(2).asDouble();
            desPos[2]=command.get(3).asDouble();
            double t=command.get(4).asDouble();

            arm -> getPose(xc, oc);
            desOrient = oc;

            arm->goToPose(desPos, desOrient, t);

            reply.addString("ack");
            reply.addString("iCub reached the desired pose!");
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);

        return true;
    }

    /********************************************************************/
    bool close() override {

        for (auto& i:ipos) {
            i->stop();
        }
        for (auto& d:drv) {
            d.close();
        }

        drv_arm.close();
        drv_gaze.close();

        targetPort.close();
        yposPort.close();
        puckPort.close();
        hand_pix_port.close();
        output_port.close();

        myFile1.close();
        myFile2.close();

        return true;
    }

    /********************************************************************/
    double getPeriod() override {
        return 0.01;
    }

    /********************************************************************/
    bool updateModule() override {

//        yarp::sig::Vector currentArmPos, currentArmOrient, hand_pix;
//        arm->getPose(currentArmPos, currentArmOrient);
//        hand_pix = projectToVisualSpace(currentArmPos);
//
////            std::cout << "u_hand:" << hand_pix[0] << ", v_hand:" << hand_pix[1] << std::endl;
//
//        // BOTTLE for printing the current u and v of the arm and of the puck in the visual space
//        yarp::os::Bottle &hand_pix_bottle = hand_pix_port.prepare();
//        hand_pix_bottle.clear();
//        hand_pix_bottle.addDouble(currentArmPos[1]);
//        hand_pix_bottle.addDouble(hand_pix[0]);
//        hand_pix_bottle.addDouble(hand_pix[1]);
//
//        hand_pix_port.write();

        Stamp ystamp;
        puckPort.getEnvelope(ystamp);

        ev.ID = 0;
        ev.x = u;
        ev.y = v;
        ev.stamp = pix_stamp/vtsHelper::tsscaler;
        out_queue.push_back(ev);
        output_port.write(out_queue, ystamp);
        out_queue.clear();

        puck_rf = projectToRobotSpace(u, v);

//        if (puck_tracked && v>50 && puck_velocity>0) {

//            mobility_condition = true;

//                    static double t0 = yarp::os::Time::now();
//                    myFile1 << u << "," << yarp::os::Time::now() - t0 << "," << std::endl;
//                    myFile2 << puck_rf[1] << "," << yarp::os::Time::now() - t0 << "," << std::endl;

        target = puck_rf[1];

        std::cout<< "y TARGET: "<<puck_rf[1]<<std::endl;

        if (target > y_max)
            target = y_max;
        if (target < y_min)
            target = y_min;

//        }
//        else if (puck_tracked && puck_velocity<0){
//            if (reached)
//                target = 0;
//        }
//        else{
//            mobility_condition=false;
//        }

        genLinTraj.computeCoeff(target);

        std::vector<double> pos_torso(3);
        for (size_t i = 0; i < pos_torso.size(); i++) {
            pos_torso[pos_torso.size() - 1 - i] = interp[i]->operator()(genLinTraj.getPos());
        }
        iposd[0]->setPositions(pos_torso.data());

        std::vector<double> pos_arm(7);
        for (size_t i = 0; i < pos_arm.size(); i++) {
            pos_arm[i] = interp[pos_torso.size() + i]->operator()(genLinTraj.getPos());
        }
        iposd[1]->setPositions(pos_arm.size(), std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7}).data(),
                               pos_arm.data());

        std::vector<double> pos_head(6);
        for (size_t i = 0; i < pos_head.size(); i++) {
            pos_head[i] = interp[pos_torso.size() + pos_arm.size() + i]->operator()(genLinTraj.getPos());
        }
        iposd[2]->setPositions(pos_head.data());

        yarp::sig::Vector x_actual, o_actual;
        arm->getPose(x_actual, o_actual);

        yarp::os::Bottle &ypos_bottle = yposPort.prepare();
        ypos_bottle.clear();
        ypos_bottle.addDouble(x_actual[1]);
        ypos_bottle.addDouble(puck_rf[1]);
        ypos_bottle.addInt(u);
        ypos_bottle.addDouble(puck_velocity);
        ypos_bottle.addDouble(mobility_condition);
        yposPort.write();

        return true;

    }
};


/************************************************************************/
int main(int argc, char* argv[]) {
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        yError() << "YARP doesn't seem to be available";
        return EXIT_FAILURE;
    }

    ControllerModule mod;
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("study-air-hockey");
    rf.setDefault("table-file", "table.tsv");
    rf.configure(argc, argv);
    return mod.runModule(rf);
}
