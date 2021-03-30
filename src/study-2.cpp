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

#include <iCub/ctrl/minJerkCtrl.h>

#include "../../spline/src/spline.h"
#include "matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;

/************************************************************************/
class ControllerModule: public yarp::os::RFModule
{
    std::vector<yarp::dev::PolyDriver> drv{3};
    std::vector<yarp::dev::IPositionControl*> ipos{3};
    std::vector<yarp::dev::IPositionDirect*> iposd{3};

    yarp::dev::PolyDriver drv_arm;
    yarp::dev::ICartesianControl* arm;

    yarp::dev::PolyDriver drv_gaze;
    yarp::dev::IGazeControl* gaze;

    yarp::dev::IControlLimits* ilim;

    std::string table_file;
    std::vector<std::shared_ptr<tk::spline>> interp;
    double y_min, y_max;

    bool first;
    double artificial_y_pos_puck, artificial_y_pos_puck_prec;
    int direction;
    std::vector<double> head_yaw, y_range, x_ee, y_ee, z_ee, x_des, y_des, z_des, elapsed_time;
    bool artificial=true;
    int u,v, puck_tracked;
    yarp::sig::Vector puck_rf{0.,0.,0.};
    std::vector<double> fingers_posture;
    std::ofstream myfile;
    typedef std::tuple <double, double> my_tuple;
    std::vector<my_tuple> head_status;

    yarp::os::BufferedPort<yarp::os::Bottle> targetPort;
    yarp::os::BufferedPort<yarp::os::Bottle> headPort, yposPort, puckPort, hand_pix_port;
    std::shared_ptr<iCub::ctrl::minJerkTrajGen> reference;
    yarp::sig::Vector target{0.};

    /********************************************************************/
    void helperOpenDevice(const int i, const std::string& device_name) {
        yarp::os::Property options;
        options.put("device", "remote_controlboard");
        options.put("remote", "/icubSim/" + device_name);
        options.put("local", "/study-arm-hockey/" + device_name);
        drv[i].open(options);
        
        yarp::dev::IControlMode* imod;
        drv[i].view(ipos[i]);
        drv[i].view(iposd[i]);
        drv[i].view(imod);
        drv[i].view(ilim);
        int naxes;
        ipos[i]->getAxes(&naxes);
        std::vector<int> modes(naxes, VOCAB_CM_POSITION);
        std::vector<double> vels(naxes, 10.);
        std::vector<double> accs(naxes, std::numeric_limits<double>::max());
        std::vector<double> poss(naxes, 0.);
        imod->setControlModes(modes.data());
        ipos[i]->setRefSpeeds(vels.data());
        ipos[i]->setRefAccelerations(accs.data());
        if (naxes==16){
            double pinkie_min, pinkie_max;
            ilim -> getLimits(15, &pinkie_min, &pinkie_max);
            fingers_posture={60., 80., 40., 35., 40., 35., 40., 35., pinkie_max};
        }
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
                for (size_t i = 7; i < 16; i++){
                    poss[i] = fingers_posture[i-7]; //for grasping the paddle
                }
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

    double incrementOutput(int dir, double y_cart)
    {
        y_cart += 0.006 * dir; // 5cm

        return y_cart;
    }

    yarp::sig::Vector projectToVisualSpace(yarp::sig::Vector robPos){
        yarp::sig::Vector imagePos;
        imagePos.resize(2);

        gaze->get2DPixel(0, robPos, imagePos); // project the hand Cartesian position into the image plane

        return imagePos;
    }

    yarp::sig::Vector projectToRobotSpace(int u_puck, int v_puck) {

        yarp::sig::Vector puckPix_vec, puckPos_rob;
        puckPix_vec.resize(2); puckPos_rob.resize(3);
        puckPix_vec[0] = u_puck;
        puckPix_vec[1] = v_puck;
        int whichImagePlane = 0; // image plane: 0 for left and 1 for right

        yarp::sig::Vector plane{0., 0., 1.0, 0.09};

        // useful to know the position wrt robot reference frame without knowing the plane on which the puck is moving
        gaze->get3DPointOnPlane(whichImagePlane, puckPix_vec, plane, puckPos_rob);

        // defensive movements along y direction (x and z are set equal to the initial Cartesian position)
        puckPos_rob[0] = -0.25;
        puckPos_rob[2] = -0.05;

        return puckPos_rob;
    }

    static auto sortbysec(const std::tuple<double, double>& a,
                   const std::tuple<double, double>& b)
    {
        return (std::get<1>(a) < std::get<1>(b));
    }

    /********************************************************************/
    bool configure(yarp::os::ResourceFinder& rf) override {
        table_file = rf.findFile("table-file");
        const auto T = std::abs(rf.check("T", yarp::os::Value(1.)).asDouble());

        if (!readTable()) {
            return false;
        }

        helperOpenDevice(0, "torso");
        helperOpenDevice(1, "left_arm");
        helperOpenDevice(2, "head");

        yarp::os::Property options_arm;
        options_arm.put("device", "cartesiancontrollerclient");
        options_arm.put("remote", "/icubSim/cartesianController/left_arm");
        options_arm.put("local", "/study-arm-hockey/cartesian/left_arm");
        if (!helperWaitDevice(drv_arm, options_arm, "Cartesian Controller")) {
            return false;
        }

        yarp::os::Property options_gaze;
        options_gaze.put("device", "gazecontrollerclient");
        options_gaze.put("remote", "/iKinGazeCtrl");
        options_gaze.put("local", "/study-arm-hockey/gaze");
        if (!helperWaitDevice(drv_gaze, options_gaze, "Gaze Controller")) {
            drv_arm.close();
            return false;
        }

        drv_arm.view(arm);
        drv_gaze.view(gaze);

        targetPort.open("/study-arm-hockey/target");
        headPort.open("/study-arm-hockey/head");
        yposPort.open("/study-arm-hockey/ypos");
        puckPort.open("/study-arm-hockey/puck");
        hand_pix_port.open("/study-arm-hockey/hand-pixels");
        reference = std::make_shared<iCub::ctrl::minJerkTrajGen>(target, getPeriod(), T);
        first=true;

        myfile.open("head.txt");
        if (!myfile.is_open())
        {
            yError()<<"Could not open file for printing head yaw wrt torso yaw";
            return 0;
        }

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

        targetPort.close();
        headPort.close();
        yposPort.close();
        puckPort.close();
        hand_pix_port.close();

        yInfo()<<"Graphs done";

        return true;
    }

    /********************************************************************/
    double getPeriod() override {
        return .01;
    }

    /********************************************************************/
    bool updateModule() override {
//        if (auto* b = targetPort.read(false)) {
//            const auto p = (b->get(0).asDouble() * (y_max - y_min) + (y_max + y_min)) / 2.;
//            target[0] = std::max(std::min(p, y_max), y_min); // to avoid going beyond the limits ymin and ymax
//        }

        double max = 0.15;
        double min = -0.35;

        if (artificial){
            if (first) {
                artificial_y_pos_puck = incrementOutput(1, 0);
                first = false;
                direction = 1;
            } else {
                artificial_y_pos_puck = incrementOutput(direction, artificial_y_pos_puck_prec);
            }

            int dir_mul = 1;
            if (artificial_y_pos_puck >= max || artificial_y_pos_puck <= min) {
                static double holdon_time = yarp::os::Time::now();
                if (yarp::os::Time::now() - holdon_time < 2) {
                    dir_mul = 0;
                } else {
                    dir_mul = 1;
                    direction *= -1 * dir_mul;
                    holdon_time = yarp::os::Time::now();
                }
            } else
                artificial_y_pos_puck_prec = artificial_y_pos_puck;

            puck_tracked=true;

            target[0] = artificial_y_pos_puck;
        }
        else{

            yarp::sig::Vector currentArmPos, currentArmOrient, hand_pix;
            arm->getPose(currentArmPos, currentArmOrient);
            hand_pix = projectToVisualSpace(currentArmPos);

            std::cout << "u_hand:"<<hand_pix[0]<<", v_hand:"<<hand_pix[1]<<std::endl;

            // BOTTLE for printing the current u and v of the arm and of the puck in the visual space
            yarp::os::Bottle &hand_pix_bottle = hand_pix_port.prepare();
            hand_pix_bottle.clear();
            hand_pix_bottle.addDouble(hand_pix[0]); // x position of the hand wrt image plane
            hand_pix_bottle.addDouble(hand_pix[1]); // y position of the hand wrt image plane
//        // communicate if the hand is present in the image plane
//        if (hand_pix[0]>0 && hand_pix[0]<304 && hand_pix[1]>0 && hand_pix[1]<240)
//            hand_pix_bottle.addInt(1);
//        else
//            hand_pix_bottle.addInt(0);

            hand_pix_port.write();

            yarp::os::Bottle *b = puckPort.read();
            u = b->get(0).asInt(); // x coordinate of the pixel within the image plane
            v = b->get(1).asInt(); // y coordinate of the pixel within the image plane
            puck_tracked = b->get(2).asInt(); //verify if the target is tracked or not
            std::cout << "u_puck = "<<u<<", v_puck = "<<v<<", tracked="<<puck_tracked<<std::endl;

            puck_rf = projectToRobotSpace(u,v);

            target[0]=puck_rf[1];

            if (target[0]>min && target[0]<max)
                puck_tracked=false;

            std::cout<<"TARGET y="<<target[0]<<std::endl;
        }

        if (puck_tracked){
            reference->computeNextValues(target);

            std::vector<double> pos_torso(3);
            for (size_t i = 0; i < pos_torso.size(); i++) {
                pos_torso[pos_torso.size() - 1 - i] = interp[i]->operator()(reference->getPos()[0]);
            }
            iposd[0]->setPositions(pos_torso.data());

            std::vector<double> pos_arm(7);
            for (size_t i = 0; i < pos_arm.size(); i++) {
                pos_arm[i] = interp[pos_torso.size() + i]->operator()(reference->getPos()[0]);
            }
            iposd[1]->setPositions(pos_arm.size(), std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7}).data(),
                                   pos_arm.data());

            std::vector<double> pos_head(6);
            for (size_t i = 0; i < pos_head.size(); i++) {
                pos_head[i] = interp[pos_torso.size() + pos_arm.size() + i]->operator()(reference->getPos()[0]);
            }
            iposd[2]->setPositions(pos_head.data());

            std::vector<double> actual_pos_torso(3);
            iposd[0]->getRefPositions(actual_pos_torso.data());

            std::vector<double> actual_pos_head(6);
            iposd[2]->getRefPositions(actual_pos_head.data());

            yarp::os::Bottle &head_bottle = headPort.prepare();
            head_bottle.clear();
            head_bottle.addDouble(actual_pos_head[2]-actual_pos_torso[0]);
            headPort.write();

            static double time = yarp::os::Time::now();
            yarp::sig::Vector x_actual, o_actual;
            arm->getPose(x_actual, o_actual);

//            head_yaw.push_back(actual_pos_head[2]-actual_pos_torso[0]);
            elapsed_time.push_back(yarp::os::Time::now()-time);
            x_ee.push_back(x_actual[0]);
            x_des.push_back(-0.25);

            y_ee.push_back(x_actual[1]);
            y_des.push_back(target[0]);

            z_ee.push_back(x_actual[2]);
            z_des.push_back(-0.05);
            head_status.push_back(std::make_tuple(x_actual[1],actual_pos_head[2]-actual_pos_torso[0]));

            std::sort(begin(head_status), end(head_status)); //sort by first element

            if (x_actual[1]<min+0.01){

                for (auto entry = head_status.begin(); entry != head_status.end(); entry++) {
                    y_range.push_back(std::get<0>(*entry));
                    head_yaw.push_back(std::get<1>(*entry));
                }

                yInfo()<<y_range;
                yInfo()<<head_yaw;

                plt::figure(1);
                plt::plot(y_range, head_yaw, "r-");
                plt::title("Head yaw wrt torso yaw");
                plt::xlabel("y [m]"); plt::ylabel("[deg]");
                plt::save("head_y.png");

                plt::figure(2);
                plt::named_plot("actual",elapsed_time, z_ee, "b-");
                plt::named_plot("desired",elapsed_time, z_des, "k-");
                plt::title("z position");
                plt::xlabel("Time [s]"); plt::ylabel("z[m]");
                plt::legend();
                plt::save("z_pos_-0.3<y<0.12.png");

                plt::figure(3);
                plt::named_plot("actual",elapsed_time, y_ee, "g-");
                plt::named_plot("desired",elapsed_time, y_des, "k-");
                plt::title("y position");
                plt::xlabel("Time [s]"); plt::ylabel("y[m]");
                plt::legend();
                plt::save("y_pos_-0.3<y<0.12.png");

                plt::figure(4);
                plt::named_plot("actual",elapsed_time, x_ee, "r-");
                plt::named_plot("desired",elapsed_time, x_des, "k-");
                plt::title("x position");
                plt::xlabel("Time [s]"); plt::ylabel("x[m]");
                plt::legend();
                plt::save("x_pos_-0.3<y<0.12.png");

                return 0;

            }

//            myfile << target[0] << "\t" << x_actual[1] << "\t" << actual_pos_head[2]-actual_pos_torso[0] << '\n';

            yarp::os::Bottle &ypos_bottle = yposPort.prepare();
            ypos_bottle.clear();
            ypos_bottle.addDouble(x_actual[1]);
            ypos_bottle.addDouble(target[0]);
            yposPort.write();
        }

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
