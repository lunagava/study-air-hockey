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

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>

#include <iCub/ctrl/minJerkCtrl.h>

#include "../../spline/src/spline.h"
#include "linearTraj.h"

#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/os/all.h>
//#include <event-driven/all.h>

//using namespace ev;
using namespace cv;

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
    double prev_t;

    std::ofstream myFile1, myFile2, myFile3;
    double puck_velocity;

    int label;
    yarp::sig::Vector eye_pos, eye_orient, head_pos, head_orient;

//    deque<LabelledAE> out_queue;
//    LabelledAE ev;

//    vWritePort output_port;
    yarp::os::BufferedPort<yarp::os::Bottle> targetPort;
    yarp::os::BufferedPort<yarp::os::Bottle> yposPort, puckPort, hand_pix_port, eyePort;
    yarp::os::RpcServer handlerPort;
//    ev::vReadPort< vector<ev::AE> > gen1_input_port;

    linearTraj genLinTraj;

    double target;

    bool mobility_condition, first_cycle, reached;
    yarp::sig::Vector right_bottom_vertex, left_bottom_vertex, right_top_vertex, left_top_vertex;
    yarp::sig::Vector star_right_bottom, star_left_bottom, star_right_top, star_left_top;

    cv::Mat proj_image;

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
        std::vector<double> vels(naxes, 20.);
        std::vector<double> accs(naxes, std::numeric_limits<double>::max());
        std::vector<double> poss(naxes, 0.);
        imod->setControlModes(modes.data());
        ipos[i]->setRefSpeeds(vels.data());
        ipos[i]->setRefAccelerations(accs.data());
        if (naxes==16){
            double pinkie_min, pinkie_max;
            ilim -> getLimits(15, &pinkie_min, &pinkie_max);
            fingers_posture={30., 76., 19., 61.325, 48.588, 116.986, 50.6, 95.389, pinkie_max};
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

        y_max = 0.23;

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
        eyePort.open(getName() + "/eye");
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

//        if(!gen1_input_port.open(getName() + "/AE:i"))
//            return false;

        myFile1.open("/code/luna/study-air-hockey/movingCam_exp2.txt");
        if (!myFile1.is_open())
        {
            yError()<<"Could not open file for updateModule loop time";
            return -1;
        }

//        myFile2.open("/code/luna/study-air-hockey/info.txt");
//        if (!myFile2.is_open())
//        {
//            yError()<<"Could not open file for printing COMs";
//            return -1;
//        }
//
//        myFile3.open("/code/luna/study-air-hockey/projection_tests.txt");
//        if (!myFile3.is_open())
//        {
//            yError()<<"Could not open file for printing projections";
//            return -1;
//        }

//        if(!output_port.open(getName() + "/LAE:o"))
//            return false;

        first_cycle = true;
        reached = true;

        prev_t = yarp::os::Time::now();

        proj_image = cv::Mat::zeros(240,304,CV_8UC3);

        cv::namedWindow("proj_image", cv::WINDOW_AUTOSIZE);
        cv::moveWindow("proj_image", 600,600);

        return Thread::start();
    }

    void run() {

//        std::cout << "inside RUN"<<std::endl;

//        Stamp ystamp;
//        while (Thread::isRunning()) {
//
//            unsigned int nqs = gen1_input_port.queryunprocessed();
//            for (int i = 0; i < nqs; i++) {
//                const vector<ev::AE> *q = gen1_input_port.read(ystamp);
//                if (!q || Thread::isStopping()) return;
//                for (auto &v : *q) {
//                    proj_image.at<cv::Vec3b>(v.y,v.x)=cv::Vec3b(255, 255, 255);
//                }
//            }

////            std::cout << "inside LOOP"<<std::endl;
//            yarp::os::Bottle *b = puckPort.read();
//            u = b->get(0).asInt(); // x coordinate of the pixel within the image plane
//            v = b->get(1).asInt(); // y coordinate of the pixel within the image plane
//            pix_stamp = b->get(2).asDouble();
//            puck_velocity =  b->get(3).asInt();
//            // std::cout << "u_puck = " << u << ", v_puck = " << v << std::endl;

//        }
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
        eyePort.close();
        puckPort.close();
        hand_pix_port.close();
//        output_port.close();

        myFile1.close();
        myFile2.close();

        return true;
    }

    /********************************************************************/
    double getPeriod() override {
        return 0.05;
    }

    std::tuple<yarp::sig::Vector, yarp::sig::Vector, yarp::sig::Vector, yarp::sig::Vector> project_four_vertices_table(){

        yarp::sig::Vector right_bottom, left_bottom, right_top, left_top;
        yarp::sig::Vector right_bottom_pix, left_bottom_pix, right_top_pix, left_top_pix;
        std::vector<yarp::sig::Vector> table_vertices;

        right_bottom.resize(3); left_bottom.resize(3); right_top.resize(3); left_top.resize(3);

        right_bottom[0] = -0.3; left_bottom[0] = -0.3; right_top[0] = -2.23; left_top[0] = -2.23;
        right_bottom[1] = 0.56; left_bottom[1] = -0.56; right_top[1] = 0.56; left_top[1] = -0.56;
        right_bottom[2] = -0.09; left_bottom[2] = -0.09; right_top[2] = -0.09; left_top[2] = -0.09;

        right_bottom_pix = projectToVisualSpace(right_bottom);
        left_bottom_pix = projectToVisualSpace(left_bottom);
        right_top_pix = projectToVisualSpace(right_top);
        left_top_pix = projectToVisualSpace(left_top);

        return std::make_tuple(right_bottom_pix,left_bottom_pix,right_top_pix,left_top_pix);
    }

    std::tuple<yarp::sig::Vector, yarp::sig::Vector, yarp::sig::Vector, yarp::sig::Vector> project_four_stars_table(){

        yarp::sig::Vector right_bottom_3d, left_bottom_3d, right_top_3d, left_top_3d;
        yarp::sig::Vector right_bottom_pix, left_bottom_pix, right_top_pix, left_top_pix;
        std::vector<yarp::sig::Vector> table_stars;

        right_bottom_3d.resize(3); left_bottom_3d.resize(3); right_top_3d.resize(3); left_top_3d.resize(3);

        double height_table_root = -0.11;
        left_bottom_3d[0]=-0.44; right_bottom_3d[0]=-0.44; left_top_3d[0]=-1.84; right_top_3d[0]=-1.84;
        left_bottom_3d[1]=-0.22; right_bottom_3d[1]=0.20; left_top_3d[1]=-0.22; right_top_3d[1]=0.20;
        left_bottom_3d[2]=height_table_root; right_bottom_3d[2]=height_table_root; left_top_3d[2]=height_table_root; right_top_3d[2]=height_table_root;

        right_bottom_pix = projectToVisualSpace(right_bottom_3d);
        left_bottom_pix = projectToVisualSpace(left_bottom_3d);
        right_top_pix = projectToVisualSpace(right_top_3d);
        left_top_pix = projectToVisualSpace(left_top_3d);

        return std::make_tuple(right_bottom_pix,left_bottom_pix,right_top_pix,left_top_pix);
    }

    /********************************************************************/
    bool updateModule() override {

        std::tie(right_bottom_vertex, left_bottom_vertex, right_top_vertex, left_top_vertex) = project_four_vertices_table();
        std::tie(star_right_bottom, star_left_bottom, star_right_top, star_left_top) = project_four_stars_table();

//        yInfo()<<star_right_bottom[0]<<" "<<star_right_bottom[1]<<" "<<star_left_bottom[0]<<" "<<star_left_bottom[1];
//        yInfo()<<star_right_top[0]<<" "<<star_right_top[1]<<" "<<star_left_top[0]<<" "<<star_left_top[1];

//        cv::circle(proj_image, cv::Point(star_right_bottom[0], star_right_bottom[1]), 5, cv::Scalar(0, 255, 0), -1, 8);
//        cv::circle(proj_image, cv::Point(star_left_bottom[0], star_left_bottom[1]), 5, cv::Scalar(0, 255, 0), -1, 8);
//        cv::circle(proj_image, cv::Point(star_right_top[0], star_right_top[1]), 5, cv::Scalar(0, 255, 0), -1, 8);
//        cv::circle(proj_image, cv::Point(star_left_top[0], star_left_top[1]), 5, cv::Scalar(0, 255, 0), -1, 8);
//
//        cv::circle(proj_image, cv::Point(right_bottom_vertex[0], right_bottom_vertex[1]), 5, cv::Scalar(255, 0, 0), -1, 8);
//        cv::circle(proj_image, cv::Point(left_bottom_vertex[0], left_bottom_vertex[1]), 5, cv::Scalar(255, 0, 0), -1, 8);
//        cv::circle(proj_image, cv::Point(right_top_vertex[0], right_top_vertex[1]), 5, cv::Scalar(255, 0, 0), -1, 8);
//        cv::circle(proj_image, cv::Point(left_top_vertex[0], left_top_vertex[1]), 5, cv::Scalar(255, 0, 0), -1, 8);

        yarp::sig::Vector currentArmPos, currentArmOrient, hand_pix;
        arm->getPose(currentArmPos, currentArmOrient);
        hand_pix = projectToVisualSpace(currentArmPos);

        cv::circle(proj_image, cv::Point(hand_pix[0], hand_pix[1]), 5, cv::Scalar(0, 0, 255), -1, 8);

        cv::imshow("proj_image", proj_image);
        cv::waitKey(1);

        proj_image = 0;

//            std::cout << "u_hand:" << hand_pix[0] << ", v_hand:" << hand_pix[1] << std::endl;

        // BOTTLE for printing the current u and v of the arm and of the puck in the visual space
//        yarp::os::Bottle &hand_pix_bottle = hand_pix_port.prepare();
//        hand_pix_bottle.clear();
//        hand_pix_bottle.addDouble(currentArmPos[1]);
//        hand_pix_bottle.addDouble(hand_pix[0]);
//        hand_pix_bottle.addDouble(hand_pix[1]);
//        hand_pix_bottle.addDouble(right_bottom_vertex[0]);
//        hand_pix_bottle.addDouble(right_bottom_vertex[1]);
//        hand_pix_bottle.addDouble(left_bottom_vertex[0]);
//        hand_pix_bottle.addDouble(left_bottom_vertex[1]);
//        hand_pix_bottle.addDouble(left_top_vertex[0]);
//        hand_pix_bottle.addDouble(left_top_vertex[1]);
//        hand_pix_bottle.addDouble(right_top_vertex[0]);
//        hand_pix_bottle.addDouble(right_top_vertex[1]);
//        if (reached)
//            hand_pix_bottle.addInt(1);
//        else
//            hand_pix_bottle.addInt(0);
////        std::cout<<"Vertices: (" << right_bottom_vertex[0]<<","<<right_bottom_vertex[1]<<"), ("<<left_bottom_vertex[0]<<","<<left_bottom_vertex[1]<<"), ("<<left_top_vertex[0]<<", "<<left_top_vertex[1]<<"), ("<<right_top_vertex[0]<<", "<<right_top_vertex[1]<<")"<<std::endl;
//
//        hand_pix_port.write();
//
        static double t_start = yarp::os::Time::now();
//        static double t0 = yarp::os::Time::now();
//
////        std::cout<<"u: "<<u<<", v: "<<v<<", label:"<<label<<std::endl;
////        if (u>0 && u<304 && v>0 && v<240) {
////
//////            Stamp ystamp;
//////            puckPort.getEnvelope(ystamp);
//////
//////            if (label == 1)
//////                ev.ID = 1;
//////            else
//////                ev.ID = 0;
//////            ev.x = u;
//////            ev.y = v;
//////            ev.stamp = pix_stamp / vtsHelper::tsscaler;
//////            out_queue.push_back(ev);
//////            output_port.write(out_queue, ystamp);
//////            out_queue.clear();
////
////            if (v>70 && puck_velocity>0) {
////
////                puck_rf = projectToRobotSpace(u, v);
////
////                //        yarp::sig::Vector pix_reprojected = projectToVisualSpace(puck_rf);
////                //
////                //        double u_check = 152;
////                //        double v_check = 120;
////                //        yarp::sig::Vector projection_check = projectToRobotSpace(u_check, v_check);
////
////                target = puck_rf[1];
////
////                //        std::cout<< "y TARGET: "<<puck_rf[1]<<std::endl;
////
////                if (target > y_max)
////                    target = y_max;
////                if (target < y_min)
////                    target = y_min;
////
////                mobility_condition = true;
////            }
////            else if (v>70 && puck_velocity<0){
////                target = 0;
////
////                mobility_condition = false;
////            }
////        }
//
        if (first_cycle){
            target = y_max;
            first_cycle = false;
        }

        genLinTraj.computeCoeff(target);

//        std::cout<<"target reached: "<<reached<<std::endl;

        std::vector<double> pos_torso(3);
        for (size_t i = 0; i < pos_torso.size(); i++) {
            pos_torso[pos_torso.size() - 1 - i] = interp[i]->operator()(genLinTraj.getPos());
        }
//        iposd[0]->setPositions(pos_torso.data());

        std::vector<double> pos_arm(7);
        for (size_t i = 0; i < pos_arm.size(); i++) {
            pos_arm[i] = interp[pos_torso.size() + i]->operator()(genLinTraj.getPos());
        }
//        iposd[1]->setPositions(pos_arm.size(), std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7}).data(),
//                               pos_arm.data());

        std::vector<double> pos_head(6);
        for (size_t i = 0; i < pos_head.size(); i++) {
            pos_head[i] = interp[pos_torso.size() + pos_arm.size() + i]->operator()(genLinTraj.getPos());
        }
//        iposd[2]->setPositions(pos_head.data());

        yarp::sig::Vector x_actual, o_actual;
        arm->getPose(x_actual, o_actual);

//        yInfo()<<x_actual[1]<<" "<<yarp::os::Time::now()-prev_t;

        prev_t=yarp::os::Time::now();

//        yarp::os::Bottle &ypos_bottle = yposPort.prepare();
//        ypos_bottle.clear();
//        ypos_bottle.addDouble(x_actual[1]);
//        ypos_bottle.addDouble(puck_rf[1]);
////        ypos_bottle.addDouble(u);
////        ypos_bottle.addDouble(puck_velocity);
//        ypos_bottle.addDouble(mobility_condition);
//        yposPort.write();

        if(abs(x_actual[1]-target)<0.02){
            if(target==y_max)
                target = y_min;
            else
                target = y_max;
        }

        yarp::sig::Vector q_gaze;
        gaze->getJointsDesired(q_gaze);

        yarp::sig::Vector xdhat, odhat, q_arm;
        arm->getDesired(xdhat, odhat, q_arm);

        gaze->getRightEyePose(eye_pos, eye_orient);
        gaze->getHeadPose(head_pos, head_orient);

//        yarp::os::Bottle &eye_bottle = eyePort.prepare();
//        eye_bottle.clear();
//        eye_bottle.addDouble(q_arm[0]);
//        eye_bottle.addDouble(q_arm[1]);
//        eye_bottle.addDouble(q_arm[2]);
//        eye_bottle.addDouble(q_gaze[0]);
//        eye_bottle.addDouble(q_gaze[1]);
//        eye_bottle.addDouble(q_gaze[2]);
//        eye_bottle.addDouble(q_gaze[3]);
//        eye_bottle.addDouble(q_gaze[4]);
//        eye_bottle.addDouble(q_gaze[5]);
//        eyePort.write();

//        std::cout<<"min and max"<<y_min<<", "<<y_max<<std::endl;
//
//        std::cout<<"diff: "<<abs(x_actual[1]-target)<<std::endl;

//        yInfo()<<q_gaze[2]-q_arm[2];
//        yInfo()<<q_gaze[2]-q_arm[0];
//        std::cout<<std::endl;

//        yInfo()<<q_gaze[0]<<" "<<q_gaze[1]<<" "<<q_gaze[2]<<" "<<q_arm[0]<<" "<<q_arm[1]<<" "<<q_arm[2]<<" "<<q_arm[3]<<" "<<q_arm[4]<<" "<<q_arm[5]<<" "<<q_arm[6]<<" "<<q_arm[7]<<" "<<q_arm[8]<<" "<<q_arm[9];
        //myFile1<<q_arm[0]<<" "<<q_arm[1]<<" "<<q_arm[2]<<" "<<q_gaze[0]<<" "<<q_gaze[1]<<" "<<q_gaze[2]<<" "<<q_gaze[3]<<" "<<q_gaze[4]<<" "<<q_gaze[5];

//        myFile1 << yarp::os::Time::now() - t_start << " "<< x_actual[0]<< " " << x_actual[1] << " "<<x_actual[2]<<" "<<q_gaze[0]<<" "<<q_gaze[1]<<" "<<q_gaze[2]<<" "<<q_arm[0]<<" "<<q_arm[1]<<" "<<q_arm[2]<<" "<<eye_pos[0]<<" "<< eye_pos[1]<<" "<<eye_pos[2]<<" "<<eye_orient[0]<<" "<<eye_orient[1]<<" "<<eye_orient[2]<<" "<<eye_orient[3]<< std::endl;
//        myFile2 << x_actual[1] << " " << puck_rf[1] << " "<< u<< " "<<yarp::os::Time::now() - t0<<std::endl;
//        myFile3 << projection_check[1]<<" "<<u<<" "<<pix_reprojected[1]<<" "<<v<<" "<<pix_reprojected[0]<<" "<<yarp::os::Time::now() - t0<< std::endl;

//        yInfo()<<yarp::os::Time::now() - t_start;

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
