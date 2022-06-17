/*
 *   Copyright (C) 2021 Event-driven Perception for Robotics
 *   Author: luna.gava@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __PUCKPOSITION__
#define __PUCKPOSITION__

#include <yarp/os/all.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>

#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IVelocityControl.h>

#include <event-driven/core.h>
#include <event-driven/comms.h>
#include <event-driven/algs.h>

#include <iostream>
#include <mutex>
#include <cmath>
#include <tuple>
#include <numeric>
#include <vector>
#include <deque>
#include <map>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>

//#include "hpe-core/representations.h"

using namespace ev;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

class PID
{
    double Kp,Ki;
    double integral;

public:
    // constructor
    PID() : Kp(0.0), Ki(0.0), integral(0.0) { }

    // helper function to set up sample time and gains
    void set(const double Kp, const double Ki)
    {
        this->Kp=Kp;
        this->Ki=Ki;
    }

    // compute the control command
    double command(const double reference, const double feedback, const double Ts)
    {
        // the actual error between reference and feedback
        double error=reference-feedback;

        // accumulate the error
        integral+=error*Ts;

        // compute the PID output
        return (Kp*error+Ki*integral);
    }

    void reset()
    {
        integral = 0;
    }
};

class eyeControlPID
{

protected:

    yarp::dev::PolyDriver         vel_driver, pos_driver;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IControlMode *imod;
    yarp::dev::IVelocityControl *ivel;
    yarp::dev::IPositionControl *ipos;
    yarp::dev::IControlLimits *ilim;

    int nAxes;
    std::vector<PID*> controllers;
    std::vector<double> velocity;
    std::vector<double> encs;

    int u_fixation;
    int v_fixation;

public:

    eyeControlPID() : nAxes(6), u_fixation(0), v_fixation(0) {}

    bool initVelControl(int height, int width)
    {
        yarp::os::Property option;
        option.put("device","remote_controlboard");
        option.put("remote","/icub/head");
        option.put("local","/vel_controller");

        if (!vel_driver.open(option))
        {
            yError()<<"Unable to open the device vel_driver";
            return false;
        }

        // open the views

        if(!vel_driver.view(ienc)) {
            yError() << "Driver does not implement encoder mode";
            return false;
        }
        if(!vel_driver.view(imod)) {
            yError() << "Driver does not implement control mode";
            return false;
        }
        if(!vel_driver.view(ivel)) {
            yError() << "Driver does not implement velocity mode";
            return false;
        }

        // retrieve number of axes
        int readAxes;
        ienc->getAxes(&readAxes);
        if(readAxes != nAxes) {
            yError() << "Incorrect number of axes" << readAxes << nAxes;
            return false;
        }

        velocity.resize(nAxes, 0.0);
        encs.resize(nAxes);
        controllers.resize(nAxes);
        for(int i = 0; i < nAxes; i++)
            controllers[i] = new PID;

        // set up our controllers
        controllers[0]->set(1.5, 0.1); //neck pitch
        controllers[1]->set(0.0, 0.0); //neck roll
        controllers[2]->set(1.5, 0.1); //neck yaw

        u_fixation = width / 2;
        v_fixation = height / 2;

        //set velocity control mode
        return setVelocityControl();

    }

    bool initPosControl(){
        yarp::os::Property option;
        option.put("device", "remote_controlboard");
        option.put("remote", "/icub/head");
        option.put("local", "/pos_controller");

        pos_driver.open(option);
        pos_driver.view(ipos);
        pos_driver.view(imod);
        pos_driver.view(ilim);

        return true;
    }

    bool setVelocityControl()
    {
        int naxes;
        ivel->getAxes(&naxes);
        std::vector<int> modes(naxes, VOCAB_CM_VELOCITY);

        imod->setControlModes(modes.data());

        return true;
    }

    void resetRobotHome(){

        int naxes;
        ipos->getAxes(&naxes);
        std::vector<int> modes(naxes, VOCAB_CM_POSITION);
        std::vector<double> vels(naxes, 20.);
        std::vector<double> accs(naxes, std::numeric_limits<double>::max());
        std::vector<double> poss(naxes, 0.);
        poss[2]=-11.689;
        poss[0]=5.977;

        imod->setControlModes(modes.data());
        ipos->setRefSpeeds(vels.data());
        ipos->setRefAccelerations(accs.data());
        ipos->positionMove(poss.data());

        auto done = false;
        while(!done) {
            yarp::os::Time::delay(1.);
            ipos->checkMotionDone(&done);
        }
        
        setVelocityControl();
    }

    void scroll_yaw(){
        int naxes;
        ipos->getAxes(&naxes);
        std::vector<int> modes(naxes, VOCAB_CM_POSITION);
        std::vector<double> vels(naxes, 20.);
        std::vector<double> accs(naxes, std::numeric_limits<double>::max());
        std::vector<double> poss(naxes, 0.);
        poss[2]=-10;
        poss[0]=5.977;

        imod->setControlModes(modes.data());
        ipos->setRefSpeeds(vels.data());
        ipos->setRefAccelerations(accs.data());
        ipos->positionMove(poss.data());

        auto done = false;
        while(!done) {
            yarp::os::Time::delay(1.);
            ipos->checkMotionDone(&done);
        }
    }

    void controlMono(int u, int v, double dt)
    {

        double neck_tilt=controllers[0]->command(v_fixation,v, dt);  // neck pitch
        double neck_pan=controllers[2]->command(u_fixation,u, dt); // neck yaw

        // send commands to the robot head
        velocity[0]=neck_tilt;          // neck pitch
        velocity[1]=0.0;                // neck roll
        velocity[2]=neck_pan;           // neck yaw
        velocity[3]=0.0;                // eyes tilt
        velocity[4]=0.0;                // eyes vers
        velocity[5]=0.0;                // eyes verg

//        yInfo()<<"vel: "<<velocity[0]<<velocity[2];
        //ivel->velocityMove(0, neck_tilt);
        //ivel->velocityMove(2, neck_pan);
//        ivel->velocityMove(velocity.data());

//        double vel_tilt, vel_pan;
//        ivel->getRefVelocity(0,&vel_tilt);
//        ivel->getRefVelocity(2,&vel_pan);
//        yInfo()<<"current vel"<<vel_tilt<<vel_pan;
    }

    double computeErrorDistance(int u, int v){

        double dist=sqrt((u-u_fixation)*(u-u_fixation)+(v-v_fixation)*(v-v_fixation));

        return dist;
    }

    void controlReset()
    {
        for(int i = 0; i < nAxes; i++) {
            controllers[i]->reset();
            velocity[i] = 0;
        }
        ivel->velocityMove(velocity.data());
    }

    double getJointPos(int joint_number){
        ienc->getEncoders(encs.data());
        return encs[joint_number];
    }

    void getJointLimits(int joint_num, double* joint_min, double* joint_max){

        double min, max;
        ilim->getLimits(joint_num, &min, &max);

        *joint_min = min;
        *joint_max = max;
    }

    void closeToLimit(int joint_number){

        double joint_pos_min, joint_pos_max, current_joint;
        ienc->getEncoders(encs.data());
        if (joint_number == 0){
            joint_pos_min=-27; joint_pos_max=20;
            current_joint = encs[0];
        }
        if (joint_number == 2){
            joint_pos_min=-44; joint_pos_max=44;
            current_joint = encs[2];
        }

//        getJointLimits(joint_number, &joint_pos_min, &joint_pos_max);

        double error_min = current_joint - joint_pos_max;
        double error_max = current_joint - joint_pos_min;

        double smallTh = 3;

        if (error_min < smallTh || error_max < smallTh)
            resetRobotHome();

    }


};


// class detection
class detection {

private:

    cv::Rect roi;
    double thresh;
    cv::Mat filter;
    cv::Point max_loc;
    double width, height;
    map< pair<int, int>, cv::Mat> filter_set;
    yarp::os::BufferedPort<yarp::os::Bottle> peakPort;

protected:

public:

    cv::Mat createEllipse(int puck_size){

        width = puck_size;
        height = 0.8*puck_size;
        cv::Point2d origin((width)/2, (height)/2);

        cv::Mat ell_filter = cv::Mat::zeros(height, width, CV_32F);

        for(int x=0; x< ell_filter.cols; x++) {
            for(int y=0; y< ell_filter.rows; y++) {
                double dx = (pow(x,2) -2*origin.x*x + pow(origin.x,2))/pow((width)/2,2);
                double dy = (pow(y,2) -2*origin.y*y + pow(origin.y,2))/pow((height)/2,2);
                double value = dx+ dy;
                if(value > 0.8)
                    ell_filter.at<float>(y, x) = -0.5;
                else if (value > 0.4 && value<=0.8)
                    ell_filter.at<float>(y, x) = 2;
                else
                    ell_filter.at<float>(y, x) = 0;

            }
        }
        return ell_filter;
    }

    void initialize(int filter_width, cv::Rect roi, double thresh){

        this -> roi = roi;
        this -> thresh = thresh;

        // create filter

        filter = createEllipse(filter_width);

//        yInfo()<<filter.rows<<filter.cols;

//        double fw2 = (double)filter_width/2.0;
//        filter = cv::Mat(filter_width, filter_width, CV_32F);
//
//        for(int x = 0; x < filter.cols; x++) {
//            for(int y = 0; y < filter.rows; y++) {
//                float &p = filter.at<float>(y, x);
//                double res = sqrt(pow(x-filter_width/2, 2.0) + pow(y-filter_width/2, 2.0));
//                if(res > fw2-1 + 1.5)
//                    p = -1.0;
//                else if (res < fw2-1 - 1.5)
//                    p = -1.0;
//                else
//                    p = 1.0;
//            }
//        }

//        peakPort.open("/tracker/peak:o");

// visualize filter
//        cv::Mat temp;
//        cv::normalize(filter, temp, 1, 0, cv::NORM_MINMAX);

//        cv::imshow("init filter", filter);
//        cv::waitKey(1);

    }

    bool detect(cv::Mat eros){

        static cv::Mat surface, result_convolution, result_visualization, result_color, result_conv_normalized, heat_map, result_final;
        double min, max; cv::Point min_loc;

        eros(roi).convertTo(surface, CV_32F);

        cv::filter2D(surface, result_convolution, -1, filter, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT); // look at border
        cv::minMaxLoc(result_convolution, &min, &max, &min_loc, &max_loc);

//        yarp::os::Bottle &peak_conv_bottle = peakPort.prepare();
//        peak_conv_bottle.clear();
//        peak_conv_bottle.addDouble(max);
//        peakPort.write();

//        cv::normalize(result_convolution, result_conv_normalized, 255, 0, cv::NORM_MINMAX);
//        result_conv_normalized.convertTo(result_visualization, CV_8U);
////
//        cv::cvtColor(result_visualization, result_color, cv::COLOR_GRAY2BGR);
//        if (max>thresh)
//            cv::circle(result_color, max_loc, 5, cv::Scalar(255, 0, 0), cv::FILLED);
//        else
//            cv::circle(result_color, max_loc, 5, cv::Scalar(0, 0, 255), cv::FILLED);

//        cv::imshow("DETECT_MAP", eros(roi));

//        result_convolution.at<float>(0,0) = 4000;
//        cv::normalize(result_convolution, heat_map, 255, 0, cv::NORM_MINMAX);
//        heat_map.convertTo(heat_map, CV_8U);
//
//        cv::applyColorMap(heat_map, result_final, cv::COLORMAP_JET);
//
//        cv::imshow("DETECT_HEAT_MAP", result_final);
//        cv::waitKey(1);

        max_loc += cv::Point(roi.x, roi.y);

        return max>thresh;
    }

    cv::Point getDetection(){
        return max_loc;
    }

    int getSize(){
        return filter.cols;
    }

};

// class tracking
class tracking{

private:

    cv::KalmanFilter kf;
    cv::Rect roi;
    double factor; // should be positive (roi width > puck size) and fixed
    int puck_size;
    map<int, cv::Mat> filter_bank;
    cv::Rect roi_full;
    map< pair<int, int>, cv::Mat> filter_set;
    int filter_bank_min, filter_bank_max;
    cv::Point2d puck_corr, puck_meas{cv::Point(320,240)};
    cv::Point  prev_peak;
    typedef struct{cv::Point p; double s;} score_point;
    score_point best;
    double first_time;
    cv::Point starting_position;
    cv::Rect around_puck;
    double neck_pitch;

    void createFilterBank(int min, int max){
        for(int i=min; i<=max; i+=2){
            for(int j=min; j<=max; j+=2){
                filter_set[make_pair(i,j)] = createCustom(i, j);
            }
        }
    }

    score_point convolution(cv::Mat eros, cv::Mat filter, double x_puck_pos, double y_puck_pos) {

        static cv::Mat surface, result_convolution, result_visualization, result_surface, result_color, result_final, result_final_filtered, result_conv_normalized, heat_map, H;
        double min, max;
        cv::Point highest_peak_filtered, lowest_peak, highest_peak;

        eros(roi).convertTo(surface, CV_32F);

        cv::filter2D(surface, result_convolution, -1, filter, cv::Point(-1, -1), 0,
                     cv::BORDER_ISOLATED); // look at border

        cv::Rect zoom =
                Rect(x_puck_pos - filter.cols * 0.5 - roi.x, y_puck_pos - filter.rows * 0.5 - roi.y, filter.cols+2,
                     filter.rows+2) & cv::Rect (0,0,result_convolution.cols, result_convolution.rows);

        cv::minMaxLoc(result_convolution(zoom), &min, &max, &lowest_peak, &highest_peak);

        cv::normalize(surface, result_surface, 255, 0, cv::NORM_MINMAX);
        result_surface.convertTo(result_visualization, CV_8U);

        cv::cvtColor(result_visualization, result_color, cv::COLOR_GRAY2BGR);

        cv::normalize(result_convolution, result_conv_normalized, 0, 255, cv::NORM_MINMAX);

        result_conv_normalized.convertTo(heat_map, CV_8U);

        Mat g = getGaussianKernel(zoom.height, 0.5* filter.rows, CV_32F) *
                getGaussianKernel(zoom.width, 0.5* filter.cols, CV_32F).t();

        Mat heat_map_zoom = result_conv_normalized(zoom);
        Mat heat_map_filtered = heat_map_zoom.mul(g);

//        double x_sum = 0;
//        double y_sum = 0;
//        double sum_weights = 0;
//        for (int i = 0; i < heat_map_zoom.rows; i++){
//            for (int j = 0; j < heat_map_zoom.cols; j++) {
//                x_sum += heat_map_zoom.at<float>(i, j) * j;
//                y_sum += heat_map_zoom.at<float>(i, j) * i;
//                sum_weights += heat_map_zoom.at<float>(i, j);
////                std::cout<<result_convolution(zoom).at<float>(i, j)<<" ";
//            }
////            std::cout<<std::endl;
//        }
//        cv::Point2d weighted_pos(zoom.x+x_sum/sum_weights, zoom.y+y_sum/sum_weights);

//        yInfo()<<weighted_pos.x<<" "<<weighted_pos.y;
//        yInfo()<<heat_map_filtered.cols<<" "<<heat_map_filtered.rows;

        cv::minMaxLoc(heat_map_filtered, &min, &max, &lowest_peak, &highest_peak_filtered);
//        yInfo()<<max;
        cv::normalize(heat_map_filtered, heat_map_filtered, 0, 255, cv::NORM_MINMAX);

        heat_map_filtered.convertTo(heat_map_filtered, CV_8U);

        cv::applyColorMap(heat_map, result_final, cv::COLORMAP_JET);
        cv::applyColorMap(heat_map_filtered, result_final_filtered, cv::COLORMAP_JET);

        hconcat(result_color, result_final, H);

        cv::Point new_peak = highest_peak + cv::Point(zoom.x, zoom.y);
        cv::Point new_peak_filtered = highest_peak_filtered + cv::Point(zoom.x, zoom.y);

//        cv::circle(H, new_peak, 2, cv::Scalar(0, 0, 255), cv::FILLED);
//        cv::circle(H, prev_peak-cv::Point(roi.x,roi.y), 2, cv::Scalar(0, 255, 0), cv::FILLED);
//        prev_peak = new_peak+cv::Point(roi.x, roi.y);
//        cv::circle(H, new_peak_filtered, 2, cv::Scalar(0, 0, 255), cv::FILLED);
//        cv::circle(H, weighted_pos, 2, cv::Scalar(255, 0, 0), cv::FILLED);
        cv::rectangle(H, zoom, cv::Scalar(0, 255, 0));
//        cv::ellipse(H, cv::Point(zoom.x+filter.cols*0.5, zoom.y+filter.rows*0.5), Size(filter.cols*0.5, filter.rows*0.5), 0,0,360,cv::Scalar(0,0,255),1);

//        yInfo()<<"width = "<<filter.cols<<", height = "<<filter.rows;

        cv::Rect zoom2 = cv::Rect(zoom.x+result_color.cols, zoom.y, zoom.width, zoom.height);
        cv::rectangle(H, zoom, cv::Scalar(255, 0, 255));
        cv::rectangle(H, zoom2, cv::Scalar(255, 0, 255));

//        cv::imshow("ROI TRACK", H);
//        cv::imshow("ZOOM", result_final(zoom));
//        cv::imshow("GAUSSIAN MUL", result_final_filtered);

//        cv::waitKey(1);

        return {new_peak_filtered + cv::Point(roi.x, roi.y), max};
    }

    cv::Point multi_conv(cv::Mat eros, int width, int height){

//        yInfo()<<width<<" "<<height;
//        width = 51;
//        height = 51;
        auto p = convolution(eros, filter_set[make_pair(width,height)], puck_meas.x, puck_meas.y);

//        for (int i=0;i<filter_set[make_pair(width,height)].rows; i++){
//            for (int j=0;j<filter_set[make_pair(width,height)].cols; j++){
//
//                std::cout<<filter_set[make_pair(width,height)].at<float>(i,j)<<" ";
//            }
//            std::cout<<std::endl;
//        }
//        cv::Mat dog_filter_grey;
//        cv::normalize(filter_set[make_pair(width,height)], dog_filter_grey, 0, 255, cv::NORM_MINMAX);
//        cv::Mat vis_ellipse, color_ellipse;
//        dog_filter_grey.convertTo(vis_ellipse, CV_8U);
//        cv::cvtColor(vis_ellipse,color_ellipse, cv::COLOR_GRAY2BGR);
//        cv::imshow("ell_filter", color_ellipse);

//        cv::imshow("ell_filter",filter_set[make_pair(width,height)]);
//        cv::waitKey(1);

        if(p.s > 0){
            best = p;
        }

        return best.p;
    }

protected:

public:

    void initKalmanFilter(){
//        int stateSize = 4;
//        int measSize = 2;
//        int contrSize = 0;
//
//        unsigned int type = CV_32F;
//        kf.init(stateSize, measSize, contrSize, type);
//
//        // Transition State Matrix A
//        // Note: set dT at each processing step!
//        // [ 1    0    dT 0  ]
//        // [ 0    1    0  dT ]
//        // [ 0    0    1  0  ]
//        // [ 0    0    0  1  ]
//        cv::setIdentity(kf.transitionMatrix);
//
//        // Measure Matrix H
//        // [1 0 0 0]
//        // [0 1 0 0]
//        kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
//        kf.measurementMatrix.at<float>(0) = 1.0f;
//        kf.measurementMatrix.at<float>(5) = 1.0f;
//
//        // Process Noise Covariance Matrix Q
//        // [Eu   0   0      0     ]
//        // [0    Ev  0      0     ]
//        // [0    0   Eudot  0     ]
//        // [0    0   0      Evdot ]
////        cv::setIdentity(kf.processNoiseCov, cv::Scalar(1));
//        kf.processNoiseCov.at<float>(0) = 1;
//        kf.processNoiseCov.at<float>(5) = 1;
//        kf.processNoiseCov.at<float>(10) = 5.0f;
//        kf.processNoiseCov.at<float>(15) = 5.0f;
//
//        // Measurement Noise Covariance Matrix R
//        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-2));

        factor = 2;
        roi_full = cv::Rect(0,0,640,480);

        filter_bank_min = 9;
        filter_bank_max = 105;
        createFilterBank(filter_bank_min, filter_bank_max);

        first_time = yarp::os::Time::now();
    }

    void updateROI(Point2d position, int width, int height){

        float u = position.x;
        float v = position.y;

        //puck_size = 0.1*v;
        int roi_width = factor*width;
        int roi_height = factor*height;

        if(roi_width%2==0)
            roi_width++;

        if(roi_height%2==0)
            roi_height++;

        cv::Rect roi_full = cv::Rect(0,0,640,480);
        roi = cv::Rect(u - roi_width/2, v - roi_height/2, roi_width, roi_height) & roi_full;
//        yInfo()<<"ROI ="<<roi.x<<" "<<roi.y<<" "<<roi.width<<" "<<roi.height;
    }

    void resetKalman(cv::Point starting_position, int puck_size){

        score_point best={starting_position, 5000.0};

        if (puck_size%2 == 0)
            puck_size++;

        this->puck_size=puck_size;
        this->starting_position=starting_position;

//        kf.errorCovPre.at<float>(0) = 1;
//        kf.errorCovPre.at<float>(5) = 1;
//        kf.errorCovPre.at<float>(10) = 1;
//        kf.errorCovPre.at<float>(15) = 1;
//
//        kf.statePost.at<float>(0) = starting_position.x;
//        kf.statePost.at<float>(1) = starting_position.y;
//        kf.statePost.at<float>(2) = 0;
//        kf.statePost.at<float>(3) = 0;

        updateROI(starting_position, 29, 19);



        puck_meas = starting_position;
        roi = cv::Rect(starting_position.x-puck_size/2, starting_position.y-puck_size/2, puck_size*1.6, puck_size);
    }

    cv::Point2d KalmanPrediction(double dT){

        cv::Point2d estimPuckPos;

        // Matrix A
        kf.transitionMatrix.at<float>(2) = dT;
        kf.transitionMatrix.at<float>(7) = dT;

        kf.predict();
        estimPuckPos.x = kf.statePre.at<float>(0);
        estimPuckPos.y = kf.statePre.at<float>(1);

        return estimPuckPos;
    }

    cv::Point2d KalmanCorrection(cv::Point position){

        cv::Point2d corrPuckPos;
        static cv::Mat meas(2,1, CV_32F);

        meas.at<float>(0) = position.x;
        meas.at<float>(1) = position.y;

        kf.correct(meas);
        corrPuckPos.x = kf.statePost.at<float>(0);
        corrPuckPos.y = kf.statePost.at<float>(1);

        return corrPuckPos;
    }

    cv::Mat createCustom(int width, int height){

        cv::Point2d origin((width+4)/2, (height+4)/2);
        cv::Mat ell_filter = cv::Mat::zeros(height+4, width+4, CV_32F);

//        std::cout<<ell_filter.cols<<" "<<ell_filter.rows;

        for(int x=0; x< ell_filter.cols; x++) {
            for(int y=0; y< ell_filter.rows; y++) {
                double dx = (pow(x,2) -2*origin.x*x + pow(origin.x,2))/pow((width)/2,2);
                double dy = (pow(y,2) -2*origin.y*y + pow(origin.y,2))/pow((height)/2,2);
                double value = dx+ dy;
                if(value > 0.8)
                    ell_filter.at<float>(y, x) = -0.2;
                else if (value > 0.4 && value<=0.8)
                    ell_filter.at<float>(y, x) = 3;
                else
                    ell_filter.at<float>(y, x) = -3.0;

//                std::cout<<ell_filter.at<float>(y,x)<<" ";

            }
//            std::cout<<std::endl;
        }

//        std::cout<<std::endl;
//
//        cv::Point2d origin((width+4)/2, (height+4)/2);
//        cv::Mat ell_filter = cv::Mat::zeros(height+4, width+4, CV_32F);
//
//        for(int x=0; x< ell_filter.cols; x++) {
//            for(int y=0; y< ell_filter.rows; y++) {
//                double dx = (pow(x,2) -2*origin.x*x + pow(origin.x,2))/pow((width)/2,2);
//                double dy = (pow(y,2) -2*origin.y*y + pow(origin.y,2))/pow((height)/2,2);
//                double value = dx+ dy;
//                if(value > 1.1)
//                    ell_filter.at<float>(y, x) = -0.5;
//                else if (value > 0.6&& value<=1.1)
//                    ell_filter.at<float>(y, x) = 1;
//                else
//                    ell_filter.at<float>(y, x) = 0;
//
//            }
//        }

        return ell_filter;
    }

    cv::Point track(cv::Mat eros, double dT){

        static cv::Mat eros_bgr;
        int last_x = puck_meas.x;
        int last_y = puck_meas.y;
//        int filter_size = 0.09*last_y+1.8;
//        puck_size = filter_size;

//        if (puck_size%2 == 0)
//            puck_size++;
//
//        if (puck_size<9)
//            puck_size = 9;

//        yInfo()<<puck_size;

        // static scenario params
//        double a0,a1,a2,b0,b1,b2;
//        a0 = 17.715624038044254 , a1 = 0.0035291125887034315 , a2 = 0.09579542600737656;
//        b0 = 6.313115087846054 , b1 = 0.0022264258939534796 , b2 = 0.09551609558936817;
        // gaze following case
        double m_pitch_w, q_pitch_w, m_pitch_h, q_pitch_h;
        m_pitch_w = -1.28, q_pitch_w = 49.78;
        m_pitch_h = -1.32, q_pitch_h = 32.2;
        double width = m_pitch_w*neck_pitch+q_pitch_w;
        double height = m_pitch_h*neck_pitch+q_pitch_h;
//        yInfo()<<neck_pitch;
        // moving scenario params
//        double a0_moving,a1_moving,a2_moving,b0_moving,b1_moving,b2_moving;
//        a0_moving = 12.201650881598578 , a1_moving = 0.009735421154783392 , a2_moving = 0.09872367924641742;
//        b0_moving = 1.92477315662196 , b1_moving = 0.003209378102266541 , b2_moving = 0.09801072879753021;
//        double width = a0_moving+a1_moving*last_x+a2_moving*last_y;
//        double height = b0_moving+b1_moving*last_x+b2_moving*last_y;
        height = 2 * floor(height/2) + 1;
        width = 2 * floor(width/2) + 1;

        if(height<9)
            height = 9;
        if(width<9)
            width = 9;

//        yInfo()<<"filter width"<<width<<", filter height"<<height<<"neck pitch"<<neck_pitch;
        puck_meas = multi_conv(eros, width, height);
//        cv::Point2d puck_pred = KalmanPrediction(dT);
//        puck_corr = KalmanCorrection(puck_meas);

        around_puck = cv::Rect(puck_meas.x - width/2, puck_meas.y - height/2, width, height) & roi_full;
        updateROI(puck_meas, width, height);

        return puck_meas;
    }

    cv::Point getPosition(){
        return puck_meas;
    }

    cv::Rect get_convROI(){
        return roi;
    }

    void setPitch(double pitch){
        this->neck_pitch = pitch;
    }

};

class speedUpEROS{

private:
    cv::Mat eros, eros32f;
    cv::Mat eros_norm;
    cv::Mat result_convolution, result_convolution32f;
    cv::Mat dest;
    cv::Mat roi;
    cv::Mat in[2];
    int cur_buf{0};
    std::mutex m;
    double decay{0};
    double parameter{0.05};
    int k_size{21};

public:

void init(){
    eros=cv::Mat::zeros(480,640, CV_8U);
    in[0]=cv::Mat::zeros(480,640, CV_8U);
    in[1]=cv::Mat::zeros(480,640, CV_8U);

    decay = 0.7;
    yInfo()<<decay;
}

static constexpr void switch_buffer(int &buf_i) {buf_i = (buf_i + 1) % 2;};

template <typename T> void addEvents(T begin, T end){

    m.lock();
    for (auto a = begin; a != end; a++) {
        int x = a->x;
        int y = a->y;
//        yInfo()<<x<<y;
        in[cur_buf].at<unsigned char>(y,x) = 255;
    }
    m.unlock();
}

cv::Mat getEROS(){

    cv::Mat &c_buf = in[cur_buf];
//    yInfo()<<cv::countNonZero(in[cur_buf]);

//    cv::namedWindow("new_eros", cv::WINDOW_NORMAL);
//    cv::imshow("new_eros", in[cur_buf]);
//    cv::waitKey(1);

//    for(int r=0; r<c_buf.rows; r++){
//        for(int c=0; c<c_buf.cols; c++){
//
//            std::cout<<c_buf.at<unsigned char>(c,r)<<" ";
//        }
//        std::cout<<std::endl;
//    }

    m.lock();
    switch_buffer(cur_buf);
    m.unlock();

    double thresh = 1;

    roi = cv::Mat::ones(k_size, k_size, CV_32F);

    cv::Mat c_buf_ones;
    c_buf.copyTo(c_buf_ones);
    c_buf_ones/=255;

    cv::filter2D(c_buf_ones, result_convolution, -1, roi, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);

    cv::Mat decay_matrix = cv::Mat::zeros(480, 640, CV_32F);

    for (int i=0; i<result_convolution.cols; i++){
        for (int j=0; j<result_convolution.rows; j++){
            decay_matrix.at<float>(j,i) = pow(decay, result_convolution.at<unsigned char>(j,i));
        }
    }

//    result_convolution.convertTo(result_convolution32f, CV_32F);
//    cv::threshold(result_convolution32f, dest, thresh, 1-decay, cv::THRESH_BINARY_INV);
//    dest += decay;

//    eros.convertTo(eros32f, CV_32F);
//    eros32f = eros32f.mul(dest);
//    eros32f.convertTo(eros, CV_8U);
//    c_buf.copyTo(eros, c_buf);

    eros.convertTo(eros32f, CV_32F);
    eros32f = eros32f.mul(decay_matrix);
    eros32f.convertTo(eros, CV_8U);
    c_buf.copyTo(eros, c_buf);

//    cv::normalize(eros, eros_norm, 0, 255, cv::NORM_MINMAX);
//    eros_norm.convertTo(eros_norm, CV_8U);

    cv::namedWindow("new_eros", cv::WINDOW_NORMAL);
    cv::imshow("new_eros", eros);
    cv::waitKey(1);

    c_buf = 0;

    return eros;
}

};

class asynch_thread:public Thread{

private:
    cv::Mat eros;
    bool tracking_status;
    std::mutex *m2;
    cv::Point puck_pos;
    int save_file;

    tracking tracker;
    detection detector;
    double first_instant, startLat, startTime, currentTime, latTime;
    double neck_pitch;
    int n_seq;
    bool file_closed;
    eyeControlPID *vc;
    speedUpEROS *fEROS;
    double tau;
    bool control;

protected:


public:
    double eros_latency, computation_latency;
    std::deque< std::array<double, 6> > data_to_save;

    struct fake_latency{
        cv::Point puck;
        double tstamp;
    };

    std::deque<fake_latency> fakeLat_queue;

    asynch_thread(){}

    void run();
    void initialise(cv::Mat &eros, int init_filter_width, cv::Rect roi, double thresh, std::mutex *m2, int n_trial, int n_exp, eyeControlPID* vc, double tau, bool control, speedUpEROS* fEROS);
    void setStatus(int tracking);
    int getStatus();
    cv::Point getState();
    void setFirstTime(double first);
    double getFirstTime();
    void setCurrentTime(double current);
    double getCurrentTime();
    void setLatencyTime(double latency);
    double getLatencyTime();
    void setPitch(double latency);
    double getPitch();
    cv::Rect getTrackROI();
    cv::Point getInitPos();
};

class puckPosModule:public RFModule, public Thread{

private:

    bool pause, first_it, success;
    int n_trial, n_exp;
    std::mutex m, m2;
    int w, h;
    ev::EROS EROS_vis;
    double start_time_latency;
    int save,seq;
    cv::Point puck_position;
    bool first_timer;
    ofstream file;
    double tau;
    bool control;

    ev::window<ev::AE> input_port;
//    ev::BufferedPort<AE> input_port;
    yarp::os::BufferedPort <yarp::sig::ImageOf<yarp::sig::PixelBgr> > image_out;
    yarp::sig::ImageOf<yarp::sig::PixelBgr> puckMap;

    eyeControlPID velocityController;
    asynch_thread dtrack_thread;
    speedUpEROS EROS_fast;

protected:

public:

    // constructor
    puckPosModule(){puckMap.resize(640, 480);}

    //the virtual functions that need to be overloaded

    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual void onStop();
    virtual double getPeriod();
    virtual bool updateModule();

    void run();

};

#endif