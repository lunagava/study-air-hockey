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

#include <event-driven/core.h>

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

#include "hpe-core/representations.h"

using namespace ev;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

// class detection
class detection {

private:

    cv::Rect roi;
    double thresh;
    cv::Mat filter;
    cv::Point max_loc;

protected:

public:


    void initialize(int filter_width, cv::Rect roi, double thresh){

        this -> roi = roi;
        this -> thresh = thresh;

        // create filter
        double fw2 = (double)filter_width/2.0;
        filter = cv::Mat(filter_width, filter_width, CV_32F);

        for(int x = 0; x < filter.cols; x++) {
            for(int y = 0; y < filter.rows; y++) {
                float &p = filter.at<float>(y, x);
                double res = sqrt(pow(x-filter_width/2, 2.0) + pow(y-filter_width/2, 2.0));
                if(res > fw2-1 + 1.5)
                    p = 0.0;
                else if (res < fw2-1 - 1.5)
                    p = -1.0;
                else
                    p = 1.0;
            }
        }

        // visualize filter
        cv::Mat temp;
        cv::normalize(filter, temp, 1, 0, cv::NORM_MINMAX);

        cv::namedWindow("init filter", cv::WINDOW_NORMAL);
        cv::imshow("init filter", filter);
        //cv::waitKey(1);

    }

    bool detect(cv::Mat eros){

        static cv::Mat surface, result_convolution, result_visualization, result_color, result_conv_normalized, heat_map, result_final;
        double min, max; cv::Point min_loc;

        eros(roi).convertTo(surface, CV_32F);

        cv::filter2D(surface, result_convolution, -1, filter, cv::Point(-1, -1), 0, cv::BORDER_ISOLATED); // look at border
        cv::minMaxLoc(result_convolution, &min, &max, &min_loc, &max_loc);

        cv::normalize(result_convolution, result_convolution, 255, 0, cv::NORM_MINMAX);
        result_convolution.convertTo(result_visualization, CV_8U);

//        cv::cvtColor(result_visualization, result_color, cv::COLOR_GRAY2BGR);
//        if (max>thresh)
//            cv::circle(result_color, max_loc, 5, cv::Scalar(255, 0, 0), cv::FILLED);
//        else
//            cv::circle(result_color, max_loc, 5, cv::Scalar(0, 0, 255), cv::FILLED);

        cv::normalize(result_convolution, result_conv_normalized, 0, 255, cv::NORM_MINMAX);

//        for(auto i=0; i<result_conv_normalized.rows; i++){
//            for(auto j=0; j<result_conv_normalized.cols; j++){
//                std::cout<<result_conv_normalized.at<float>(i,j)<<" ";
//            }
//            std::cout<<std::endl;
//        }

        result_conv_normalized.convertTo(heat_map, CV_8U);
        cv::applyColorMap(heat_map, result_final, cv::COLORMAP_JET);

        cv::namedWindow("RESULT", cv::WINDOW_NORMAL);
        cv::imshow("RESULT", result_final);

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
    cv::Rect roi, roi_full;
    double factor; // should be positive (roi width > puck size) and fixed
    int puck_size;
    map<int, cv::Mat> filter_bank;
    int filter_bank_min, filter_bank_max;
    ofstream myfile;
    double first_time;
    cv::Point2d puck_corr, puck_meas;
    cv::Point starting_position;
    typedef struct{cv::Point p; double s;} score_point;
    score_point best;

    void createFilterBank(int min, int max){
        for(int i=min; i<=max; i+=2)
            filter_bank[i] = createFilter(i);

    }

    score_point convolution(cv::Mat eros, cv::Mat filter){

        static cv::Mat surface, result_convolution, result_visualization, result_surface, result_color, result_final, result_conv_normalized, heat_map;
        double min1, max1, min2, max2; cv::Point max_loc, min_loc, highest_peak1, highest_peak2, lowest_peak1, lowest_peak2;

        eros(roi).convertTo(surface, CV_32F);

        cv::filter2D(surface, result_convolution, -1, filter, cv::Point(-1, -1), 0, cv::BORDER_ISOLATED); // look at border
        cv::minMaxLoc(result_convolution, &min1, &max1, &lowest_peak1, &highest_peak1);

        result_convolution.at<float>(highest_peak1.y, highest_peak1.x) = -2000.0;
        cv::minMaxLoc(result_convolution, &min2, &max2, &lowest_peak2, &highest_peak2);

//        for(auto i=0; i<result_convolution.rows; i++){
//            for(auto j=0; j<result_convolution.cols; j++){
//                std::cout<<result_convolution.at<float>(i,j)<<" ";
//            }
//            std::cout<<std::endl;
//        }

//        yInfo() <<highest_peak1.x<<" "<<highest_peak1.y<<" ,"<<highest_peak2.x<<" "<<highest_peak2.y;

        cv::normalize(surface, result_surface, 255, 0, cv::NORM_MINMAX);
        result_surface.convertTo(result_visualization, CV_8U);

        cv::cvtColor(result_visualization, result_color, cv::COLOR_GRAY2BGR);

        cv::normalize(result_convolution, result_conv_normalized, 0, 255, cv::NORM_MINMAX);

//        for(auto i=0; i<result_conv_normalized.rows; i++){
//            for(auto j=0; j<result_conv_normalized.cols; j++){
//                std::cout<<result_conv_normalized.at<float>(i,j)<<" ";
//            }
//            std::cout<<std::endl;
//        }

        result_conv_normalized.convertTo(heat_map, CV_8U);
        cv::applyColorMap(heat_map, result_final, cv::COLORMAP_JET);

        cv::circle(result_color, highest_peak1, 1, cv::Scalar(255, 0, 0), cv::FILLED);
        cv::circle(result_color, highest_peak2, 1, cv::Scalar(0, 0, 255), cv::FILLED);

        cv::namedWindow("ROI TRACK", cv::WINDOW_NORMAL);
        cv::imshow("ROI TRACK", result_final);

        return {highest_peak1 + cv::Point(roi.x, roi.y), max1};
    }

    cv::Point multi_conv(cv::Mat eros, int filter_size){

        auto p = convolution(eros, filter_bank[filter_size]);
        if(p.s > 1000){
            best = p;
        }

        return best.p;
    }

protected:

public:

    void initKalmanFilter(){
        int stateSize = 4;
        int measSize = 2;
        int contrSize = 0;

        unsigned int type = CV_32F;
        kf.init(stateSize, measSize, contrSize, type);

        // Transition State Matrix A
        // Note: set dT at each processing step!
        // [ 1    0    dT 0  ]
        // [ 0    1    0  dT ]
        // [ 0    0    1  0  ]
        // [ 0    0    0  1  ]
        cv::setIdentity(kf.transitionMatrix);

        // Measure Matrix H
        // [1 0 0 0]
        // [0 1 0 0]
        kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
        kf.measurementMatrix.at<float>(0) = 1.0f;
        kf.measurementMatrix.at<float>(5) = 1.0f;

        // Process Noise Covariance Matrix Q
        // [Eu   0   0      0     ]
        // [0    Ev  0      0     ]
        // [0    0   Eudot  0     ]
        // [0    0   0      Evdot ]
        //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
        kf.processNoiseCov.at<float>(0) = 1e-4;
        kf.processNoiseCov.at<float>(5) = 1e-4;
        kf.processNoiseCov.at<float>(10) = 5.0f;
        kf.processNoiseCov.at<float>(15) = 5.0f;


        // Measurement Noise Covariance Matrix R
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1));

        factor = 2;
        roi_full = cv::Rect(0,0,640,480);

        filter_bank_min = 9;
        filter_bank_max = 49;
        createFilterBank(filter_bank_min, filter_bank_max);

        myfile.open("/data/kalman.txt");
        if (!myfile.is_open())
        {
            yError()<<"Could not open file for kalman prediction and correction";
            return;
        }

        first_time = yarp::os::Time::now();
    }

    void updateROI(Point2d position){

        float u = position.x;
        float v = position.y;

        //puck_size = 0.1*v;
        int roi_width = factor*puck_size;

        roi = cv::Rect(u - roi_width/2, v - roi_width/2, roi_width, roi_width) & roi_full;
    }

    void resetKalman(cv::Point starting_position, int puck_size){

        score_point best={starting_position, 5000.0};

        if (puck_size%2 == 0)
            puck_size++;

        this->puck_size=puck_size;
        this->starting_position=starting_position;

        kf.errorCovPre.at<float>(0) = 1;
        kf.errorCovPre.at<float>(5) = 1;
        kf.errorCovPre.at<float>(10) = 1;
        kf.errorCovPre.at<float>(15) = 1;

        kf.statePost.at<float>(0) = starting_position.x;
        kf.statePost.at<float>(1) = starting_position.y;
        kf.statePost.at<float>(2) = 0;
        kf.statePost.at<float>(3) = 0;

        updateROI(starting_position);

        puck_corr = starting_position;
    }

    void updateDetectedPos(cv::Point starting_position, int puck_size){
        if (puck_size%2 == 0)
            puck_size++;

        this->puck_size=puck_size;
        this->starting_position=starting_position;

        updateROI(starting_position);
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

    cv::Mat createFilter(int puck_size){ // in the future create an array of possible filters (all possible puck sizes)

        if (puck_size%2 == 0)
            puck_size++;

        if (puck_size<9)
            puck_size = 9;

        double fw2 = (double)puck_size/2.0;
        cv::Mat filter = cv::Mat(puck_size, puck_size, CV_32F);

        for(int x = 0; x < filter.cols; x++) {
            for(int y = 0; y < filter.rows; y++) {
                float &p = filter.at<float>(y, x);
                double res = sqrt(pow(x-puck_size/2, 2.0) + pow(y-puck_size/2, 2.0));
                if(res > fw2-1 + 1.5)
                    p = 0.0;
                else if (res < fw2-1 - 1.5)
                    p = -1.0;
                else
                    p = 1.0;
            }
        }

        return filter;
    }



    void track(cv::Mat eros, double dT){

        static cv::Mat eros_bgr;
        int last_y = puck_corr.y;
        int filter_size = 0.08*last_y+1.8;
        puck_size = filter_size;

        if (puck_size%2 == 0)
            puck_size++;

        if (puck_size<9)
            puck_size = 9;

        cv::Point2d puck_meas = multi_conv(eros, puck_size);
        cv::Point2d puck_pred = KalmanPrediction(dT);
        puck_corr = KalmanCorrection(puck_meas);

        updateROI(puck_corr);

        cv::cvtColor(eros, eros_bgr, cv::COLOR_GRAY2BGR);
        cv::circle(eros_bgr, puck_corr, 5, cv::Scalar(255, 0, 255), cv::FILLED);
        cv::circle(eros_bgr, puck_pred, 5, cv::Scalar(0, 255, 255));
        cv::circle(eros_bgr, puck_meas, 5, cv::Scalar(0, 0, 255));
        cv::rectangle(eros_bgr, roi, cv::Scalar(0,255,0));

        cv::namedWindow("FULL IMAGE", cv::WINDOW_NORMAL);
        cv::imshow("FULL IMAGE", eros_bgr);
        //cv::waitKey(0);

        yInfo()<<puck_size;
        yInfo()<<"("<<puck_meas.x<<","<<puck_meas.y<<") ("<<kf.statePre.at<float>(0)<<","<<kf.statePre.at<float>(1)<<") ("<<kf.statePost.at<float>(0)<<","<<kf.statePost.at<float>(1)<<")";
//        yInfo()<<puck_meas.x<<","<<puck_meas.y<<","<<puck_pred.x<<","<<puck_pred.y<<","<<puck_corr.x<<","<<puck_corr.y;
        myfile<<puck_size<<","<<(yarp::os::Time::now()-first_time)<<","<<puck_meas.x<<","<<puck_meas.y<<","<<puck_pred.x<<","<<puck_pred.y<<","<<puck_corr.x<<","<<puck_corr.y<<endl;
    }


};

class asynch_thread:public Thread{

private:
    cv::Mat eros;

    tracking tracker;
    detection detector;
protected:


public:
    asynch_thread(){}

    void run();
    void initialise(cv::Mat &eros, int init_filter_width, cv::Rect roi, double thresh);

};

class puckPosModule:public RFModule, public Thread{

private:

    bool success;
    bool pause;
    std::mutex m;
    int w, h;
    hpecore::surface EROS_vis;

    ev::BufferedPort<AE> input_port;

    asynch_thread eros_thread;

protected:

public:

    // constructor
    puckPosModule(){}

    //the virtual functions that need to be overloaded

    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual void onStop();
    virtual double getPeriod();
    virtual bool updateModule();
    void run();

};

#endif
//empty line