/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Luna Gava <luna.gava@iit.it>
 */

#include <cstdlib>
#include <cmath>
#include <tuple>
#include <iterator>
#include <algorithm>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/core/types.hpp>

using namespace cv;

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <iostream>

#include <event-driven/all.h>

using namespace ev;

class Matrix;
class Vector3;

/************************************************************************/
class ControllerModule: public yarp::os::RFModule, public yarp::os::Thread
{
    yarp::dev::PolyDriver drv;
    yarp::dev::IPositionControl* ipos;
    yarp::dev::IPositionDirect* iposd;
    yarp::dev::IControlMode* imod;

    yarp::dev::PolyDriver drv_arm;
    yarp::dev::ICartesianControl* arm;

    yarp::sig::Matrix R;

    yarp::dev::PolyDriver drv_gaze;
    yarp::dev::IGazeControl* gaze;

    yarp::dev::IEncoders* ienc;

    int startup_context_id_arm, startup_context_id_gaze;
    yarp::sig::Vector x0{-.25, .0, -.05}, o0;
    yarp::sig::Vector fixation{-.7, .0, -.05};

    double y_min, y_max, y_delta, y;

    std::string robot, which_arm;

    static constexpr double wait_ping{.1};
    static constexpr double wait_tmo{3.};

    std::string table_file;
    typedef std::tuple<double, yarp::sig::Vector> Entry;
    std::vector<Entry> table;

    yarp::os::BufferedPort<yarp::os::Bottle> handPort;
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > image_out;

    yarp::sig::ImageOf<yarp::sig::PixelBgr> trackMap;
    cv::Mat trackImg;

    vReadPort<vector<AE> > input_port;

    deque<AE> out_queue;

    Stamp ystamp;
    bool hand_segmentation;
    double sum_x, sum_y;
    int n_events;
    int roi_hand_width, roi_hand_height;
    yarp::sig::Vector roi_hand;

    /********************************************************************/
    void helperOpenDevice(const std::string& device_name) {
        yarp::os::Property options;
        options.put("device", "remote_controlboard");
        options.put("remote", "/"+ robot +"/" + device_name);
        options.put("local", getName()+"/" + device_name);
        drv.open(options);

        yarp::dev::IControlLimits* ilim;
        drv.view(ipos);
        drv.view(iposd);
        drv.view(imod);
        drv.view(ilim);
        drv.view(ienc);
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

    /********************************************************************/
    void helperFillVector(yarp::os::ResourceFinder& rf,
                          const std::string& option_name,
                          yarp::sig::Vector& v) {
        if (rf.check(option_name)) {
            if (const yarp::os::Bottle* b = rf.find(option_name).asList()) {
                for (size_t i = 0; i < std::min(v.length(), b->size()); i++) {
                    v[i] = b->get(i).asDouble();
                }
            }
        }
    }

    /********************************************************************/
    const auto helperToString(const yarp::sig::Vector& v) const {
        std::ostringstream str;
        for (auto it = v.begin(); it != v.end(); it++) {
            str << *it;
            if (it != v.end() - 1) {
                str << "\t";
            }
        }
        return str.str();
    }

    /**************************************************************************/
    static auto helperCompareEntries(const Entry& e1, const Entry& e2) {
        return (std::get<0>(e1) < std::get<0>(e2));
    }

    void moveJoint(int joint, double desired_angle){

        imod->setControlMode(joint, VOCAB_CM_POSITION);
        ipos->setRefSpeed(joint, 100.0);
        ipos->positionMove(joint, desired_angle);

        auto done = false;
        double t0=yarp::os::Time::now();

        while((yarp::os::Time::now() - t0) < 2.0 && !done) {
            yarp::os::Time::delay(1.);
            ipos->checkMotionDone(&done);
        }
    }

    void moveJointDirect(int joint, double desired_angle){

        imod->setControlMode(5, VOCAB_CM_POSITION_DIRECT);
        iposd->setPosition(5, desired_angle);

    }

    /********************************************************************/
    bool configure(yarp::os::ResourceFinder& rf) override {
        table_file = rf.getHomeContextPath() + "/" +
                     rf.check("table-file", yarp::os::Value("lut.tsv")).asString();
        setName((rf.check("name", yarp::os::Value("/study-air-hockey")).asString()).c_str());
        robot = rf.check("robot", yarp::os::Value("icubSim")).asString();
        which_arm = rf.check("arm", yarp::os::Value("left_arm")).asString();
        const auto torso_joints = rf.check("torso-joints", yarp::os::Value(1)).asInt();
        const auto torso_pitch = rf.check("torso-pitch", yarp::os::Value(30.)).asDouble();
        y_min = std::abs(rf.check("y-min", yarp::os::Value(.15)).asDouble());
        y_max = std::abs(rf.check("y-max", yarp::os::Value(.15)).asDouble());
        y_delta = std::abs(rf.check("y-delta", yarp::os::Value(.005)).asDouble());
        roi_hand_width = rf.check("roi_hand_width", yarp::os::Value(100)).asInt();
        roi_hand_height = rf.check("roi_hand_height", yarp::os::Value(80)).asInt();
        helperFillVector(rf, "x0", x0);
        helperFillVector(rf, "fixation", fixation);

        helperOpenDevice(which_arm);

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

        arm->storeContext(&startup_context_id_arm);

        arm->setTrajTime(.6);
        yarp::sig::Vector dof;
        arm->getDOF(dof);
        dof = 1.;
        arm->setDOF(dof, dof);
        switch (torso_joints) {
            case 0:
                arm->setLimits(2, 0., 0.);
            case 1:
                arm->setLimits(0, torso_pitch, torso_pitch);
            case 2:
                arm->setLimits(1, 0., 0.);
        }

        R = yarp::math::zeros(3, 3);

        R(0, 0) = -1.; R(2, 1) = -1.; R(1, 2) = -1.;
        o0 = yarp::math::dcm2axis(R);

        arm->goToPoseSync(x0, o0);
        arm->waitMotionDone(wait_ping, wait_tmo);
        arm ->setPosePriority("position");

        drv_gaze.view(gaze);
        gaze->storeContext(&startup_context_id_gaze);

        gaze->setNeckTrajTime(.6);
        gaze->setEyesTrajTime(.3);
        gaze->blockNeckRoll(0.);
        gaze->blockEyes(4.948); // vergence used during calibration

        gaze->lookAtFixationPoint(fixation);
        gaze->waitMotionDone(wait_ping, wait_tmo);

        if (which_arm=="left_arm")
            y=-y_min;
        else
            y=y_max;

        handPort.open(getName() + "/hand-perimeter");

        if (!input_port.open(getName() + "/AE:i"))
            return false;

        hand_segmentation=false;

        trackMap.resize(304,240);

        if (!image_out.open(getName() + "/image:o")) {
            yError() << "Can't open output image port for visualization";
            return false;
        }

        roi_hand.resize(4);
        roi_hand = setROI(304-roi_hand_width, 304, 240-roi_hand_height, 240);

        return Thread::start();
    }

    void run() {

        // read some data to extract the channel
        const vector<AE> *q = input_port.read(ystamp);
        if (!q || Thread::isStopping()) return;

        while (Thread::isRunning()) {

            if (hand_segmentation) {

                int nqs = input_port.queryunprocessed();

                while(nqs>0) {
                    q = input_port.read(ystamp);
                    --nqs;
                }

                q = input_port.read(ystamp);

                for (auto &v : *q)
                    out_queue.push_front(v);
            }
        }
    }

    /********************************************************************/
    auto writeTable() {
        std::sort(begin(table), end(table), helperCompareEntries);
        std::ofstream fout(table_file);
        if (fout.is_open()) {
            for (auto entry = table.begin(); entry != table.end(); entry++) {
                const auto& y = std::get<0>(*entry);
                const auto& perimeter = std::get<1>(*entry);
                fout << y << "\t"
                     << helperToString(perimeter);
                if (entry != table.end() - 1) {
                    fout << std::endl;
                }
            }
            fout.close();
            yInfo() << "Table written to" << table_file;
            return true;
        } else {
            yError() << "Unable to write to file" << table_file;
            return false;
        }
    }

    yarp::sig::Vector setROI(int xl, int xh, int yl, int yh){

        yarp::sig::Vector roi;
        roi.resize(4);

        roi[0] = xl; roi[1] = xh;
        roi[2] = yl; roi[3] = yh;

        return roi;
    }

    //compute standard deviation
    std::vector<int> find_extremes(std::deque<AE> queue) {

        std::vector<int> extremes;
        std::vector<int> x_values, y_values;

        for (auto &v: queue) {
            x_values.push_back(v.x);
            y_values.push_back(v.y);
        }

        int xmax_hand = *max_element(x_values.begin(), x_values.end());
        int ymax_hand = *max_element(y_values.begin(), y_values.end());
        int xmin_hand = *min_element(x_values.begin(), x_values.end());
        int ymin_hand = *min_element(y_values.begin(), y_values.end());

        extremes.push_back(xmin_hand);
        extremes.push_back(xmax_hand);
        extremes.push_back(ymin_hand);
        extremes.push_back(ymax_hand);

        std::cout<<"Extremes: "<<xmin_hand<<" "<<xmax_hand<<" "<<ymin_hand<<" "<<ymax_hand<<std::endl;

        return extremes;
    }

    /********************************************************************/
    bool close() override {

        arm->stopControl();
        arm->restoreContext(startup_context_id_arm);
        drv_arm.close();

        gaze->stopControl();
        gaze->restoreContext(startup_context_id_gaze);
        drv_gaze.close();

        writeTable();

        handPort.close();

        ipos->stop();
        drv.close();

        return true;
    }

    /********************************************************************/
    double getPeriod() override {
        return 0.01;
    }

    /********************************************************************/
    bool updateModule() override {

        yarp::sig::Vector Xwrist, Owrist;
        arm->getPose(8, Xwrist, Owrist); // get position and orientation of the last joint (wrist yaw)

        yarp::sig::Vector Xelbow, Oelbow;
        arm->getPose(6, Xelbow, Oelbow); // get position and orientation of the last joint (wrist yaw)

        double theta = -atan2(Xelbow[1]-Xwrist[1], Xelbow[0]-Xwrist[0]);

        yarp::sig::Matrix Rtheta_y;
        Rtheta_y = yarp::math::zeros(3, 3); // rotation matrix around y

        Rtheta_y(0,0)=cos(theta);
        Rtheta_y(0,1)=0;
        Rtheta_y(0,2)=sin(theta);
        Rtheta_y(1,0)=0;
        Rtheta_y(1,1)=1;
        Rtheta_y(1,2)=0;
        Rtheta_y(2,0)=-sin(theta);
        Rtheta_y(2,1)=0;
        Rtheta_y(2,2)=cos(theta);

        arm->goToPoseSync(x0 + yarp::sig::Vector{0., y, 0.} , yarp::math::dcm2axis(R*Rtheta_y));
        arm->waitMotionDone(wait_ping, wait_tmo);

        std::cout<<y<<std::endl;

        gaze->lookAtFixationPoint(fixation);
        gaze->waitMotionDone(wait_ping, wait_tmo);

        double current_wrist_pitch;
        double hand_vibration=40;

        ienc->getEncoder(5, &current_wrist_pitch);
        yInfo() << current_wrist_pitch;

        yarp::sig::ImageOf<yarp::sig::PixelBgr> &display = image_out.prepare();
        display = trackMap;

        trackMap.zero();

        hand_segmentation = true; sum_x=0; sum_y=0; n_events=0; out_queue.clear();

        moveJoint(5, current_wrist_pitch - hand_vibration);
        moveJoint(5, current_wrist_pitch);
//        moveJoint(5, current_wrist_pitch-hand_vibration);

        hand_segmentation = false;

        std::cout<<"ROI: "<<roi_hand[0]<<" "<<roi_hand[1]<<" "<<roi_hand[2]<<" "<<roi_hand[3]<<std::endl;

        int count=0;
        deque<AE> queue_insideROI; queue_insideROI.clear();
        for (auto &v : out_queue) {
            if (v.x>roi_hand[0] && v.x<roi_hand[1] && v.y>roi_hand[2] && v.y<roi_hand[3] && count<2000) {

                queue_insideROI.push_back(v);

                sum_x += v.x;
                sum_y += v.y;

                n_events++;

                std::cout << "x= " << v.x << ", y= " << v.y << ", ts= " << v.stamp << std::endl;

                yarp::sig::PixelBgr &ePix = trackMap.pixel(v.x, v.y);
                ePix.b = ePix.g = ePix.r = 255;
            }
            count++;
        }

        std::cout << sum_x << ", "<<sum_y<<", "<<n_events<<std::endl;

        double xCoM = sum_x / n_events;
        double yCoM = sum_y / n_events;

        std::cout<< xCoM <<" "<<yCoM<<std::endl;

        if (out_queue.size()!=0)
            roi_hand = setROI(xCoM-roi_hand_width/2, xCoM+roi_hand_width/2, yCoM-roi_hand_height/2, yCoM+roi_hand_height/2);

        std::vector<int> extremes = find_extremes(queue_insideROI);

        trackImg = cv::cvarrToMat((IplImage *) display.getIplImage());

        cv::applyColorMap(trackImg, trackImg, cv::COLORMAP_BONE);

        // puck
        cv::rectangle(trackImg, cv::Point(extremes[0], extremes[2]), cv::Point(extremes[1], extremes[3]),
                      cv::Scalar(255, 255, 0), 1, 8, 0); // print ROI box
        cv::circle(trackImg, cv::Point(xCoM,yCoM), 2, cv::Scalar(255, 255, 0), -1, 8, 0);

        image_out.write();

//        yarp::os::Bottle *handBottle = handPort.read();
//        int u = handBottle->get(0).asInt();
//        int v = handBottle->get(1).asInt();
//        int left = handBottle->get(2).asInt();
//        int right = handBottle->get(3).asInt();
//        int bottom = handBottle->get(4).asInt();
//        int top = handBottle->get(5).asInt();
//
        yarp::sig::Vector perimeter{0,0,0,0,0,0};
        perimeter[0]=xCoM; perimeter[1]=yCoM; perimeter[2]=extremes[0]; perimeter[3]=extremes[1]; perimeter[4]=extremes[2]; perimeter[5]=extremes[3];

        table.push_back(std::make_tuple(y, perimeter));

//        yInfo()<<"CoM: "<<"("<<u<<" , "<<v<<")";

        if (which_arm=="left_arm")
            y += y_delta;
        else
            y -= y_delta;

        if (which_arm=="left_arm")
            return (y <= y_max + y_delta/2.);
        else
            return (y >= -y_min - y_delta/2.);
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
    rf.configure(argc, argv);
    return mod.runModule(rf);
}
