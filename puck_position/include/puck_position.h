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

    cv::Mat createEllipse(int puck_size){

        double height = puck_size;
        double width = 2*puck_size;
        cv::Point origin((width)/2, (height)/2);

        cv::Mat ell_filter = cv::Mat::zeros(height, width, CV_32F);

        for(int x=0; x< ell_filter.cols; x++) {
            for(int y=0; y< ell_filter.rows; y++) {
                double dx = (pow(x,2) -2*origin.x*x + pow(origin.x,2))/pow((width)/2,2);
                double dy = (pow(y,2) -2*origin.y*y + pow(origin.y,2))/pow((height)/2,2);
                double value = dx+ dy;
                if(value > 0.8)
                    ell_filter.at<float>(y, x) = 0;
                else if (value > 0.5 && value<=0.8)
                    ell_filter.at<float>(y, x) = 1;
                else
                    ell_filter.at<float>(y, x) = 0;

            }
        }

//        for(int i=0; i<ell_filter.rows;i++){
//            for(int j=0; j<ell_filter.cols;j++){
//                std::cout<<ell_filter.at<float>(i,j)<<" ";
//            }
//            std::cout<<std::endl;
//        }

        return ell_filter;
    }


    void initialize(int filter_width, cv::Rect roi, double thresh){

        this -> roi = roi;
        this -> thresh = thresh;

        // create filter

//        filter = createEllipse(filter_width);

        double fw2 = (double)filter_width/2.0;
        filter = cv::Mat(filter_width, filter_width, CV_32F);

        for(int x = 0; x < filter.cols; x++) {
            for(int y = 0; y < filter.rows; y++) {
                float &p = filter.at<float>(y, x);
                double res = sqrt(pow(x-filter_width/2, 2.0) + pow(y-filter_width/2, 2.0));
                if(res > fw2-1 + 1.5)
                    p = -1.0;
                else if (res < fw2-1 - 1.5)
                    p = -1.0;
                else
                    p = 1.0;
            }
        }

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

        cv::filter2D(surface, result_convolution, -1, filter, cv::Point(-1, -1), 0, cv::BORDER_ISOLATED); // look at border
        cv::minMaxLoc(result_convolution, &min, &max, &min_loc, &max_loc);

        cv::normalize(result_convolution, result_convolution, 255, 0, cv::NORM_MINMAX);
        result_convolution.convertTo(result_visualization, CV_8U);

        cv::cvtColor(result_visualization, result_color, cv::COLOR_GRAY2BGR);
        if (max>thresh)
            cv::circle(result_color, max_loc, 5, cv::Scalar(255, 0, 0), cv::FILLED);
        else
            cv::circle(result_color, max_loc, 5, cv::Scalar(0, 0, 255), cv::FILLED);

//        cv::imshow("RESULT", result_color);
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
    cv::Rect roi, roi_full;
    double factor; // should be positive (roi width > puck size) and fixed
    int puck_size;
    map<int, cv::Mat> filter_bank;
    map< pair<int, int>, cv::Mat> filter_set;
    int filter_bank_min, filter_bank_max;
    ofstream myfile;
    double first_time;
    cv::Point2d puck_corr, puck_meas;
    cv::Point starting_position;
    typedef struct{cv::Point p; double s;} score_point;
    score_point best;

    void createFilterBank(int min, int max){
        for(int i=min; i<=max; i+=2){
            for(int j=min; j<=max; j+=2){
                filter_set[make_pair(i,j)] = createCustom(i, j);
            }
        }
//            filter_bank[i] = createEllipse(i);
//            filter_bank[i] = createEllipse2(i*2,i);
//            filter_bank[i] = createFilter(i);

    }

    score_point convolution(cv::Mat eros, cv::Mat filter){

        static cv::Mat surface, result_convolution, result_visualization, result_surface, result_color, result_final, result_final_filtered, result_conv_normalized, heat_map, H;
        double min, max; cv::Point highest_peak, highest_peak_filtered, lowest_peak;

        eros(roi).convertTo(surface, CV_32F);

        cv::filter2D(surface, result_convolution, -1, filter, cv::Point(-1, -1), 0, cv::BORDER_ISOLATED); // look at border

        cv::Rect zoom = Rect((roi.width-filter.cols)*0.5, (roi.height-filter.rows)*0.5, filter.cols, filter.rows);
        cv::minMaxLoc(result_convolution(zoom), &min, &max, &lowest_peak, &highest_peak);

        cv::normalize(surface, result_surface, 255, 0, cv::NORM_MINMAX);
        result_surface.convertTo(result_visualization, CV_8U);

        cv::cvtColor(result_visualization, result_color, cv::COLOR_GRAY2BGR);

        cv::normalize(result_convolution, result_conv_normalized, 0, 255, cv::NORM_MINMAX);

        result_conv_normalized.convertTo(heat_map, CV_8U);

        Mat g = getGaussianKernel(filter.rows, 0.8*filter.rows, CV_32F) * getGaussianKernel(filter.cols, 0.8*filter.cols, CV_32F).t();
        Mat heat_map_zoom = result_conv_normalized(zoom);
        Mat heat_map_filtered = heat_map_zoom.mul(g);

        cv::normalize(heat_map_filtered, heat_map_filtered, 0, 255, cv::NORM_MINMAX);
        heat_map_filtered.convertTo(heat_map_filtered, CV_8U);

        cv::applyColorMap(heat_map, result_final, cv::COLORMAP_JET);
        cv::applyColorMap(heat_map_filtered, result_final_filtered, cv::COLORMAP_JET);

        hconcat(result_color, result_final, H);

        cv::minMaxLoc(heat_map_filtered, &min, &max, &lowest_peak, &highest_peak_filtered);

        cv::Point new_peak = highest_peak + cv::Point(zoom.x, zoom.y);
        cv::Point new_peak_filtered = highest_peak_filtered + cv::Point(zoom.x, zoom.y);

        cv::circle(H, new_peak, 5, cv::Scalar(0, 0, 255), cv::FILLED);
        cv::circle(H, new_peak_filtered, 5, cv::Scalar(0, 255, 0), cv::FILLED);
        cv::rectangle(H, zoom, cv::Scalar(0, 255, 0));
        cv::ellipse(H, cv::Point(zoom.x+filter.cols*0.5, zoom.y+filter.rows*0.5), Size(filter.cols*0.5, filter.rows*0.5), 0,0,360,cv::Scalar(0,0,255),1);

        yInfo()<<"width = "<<filter.cols<<", height = "<<filter.rows;

        cv::Rect zoom2 = cv::Rect(zoom.x+result_color.cols, zoom.y, zoom.width, zoom.height);
        cv::rectangle(H, zoom, cv::Scalar(0, 255, 0));
        cv::rectangle(H, zoom2, cv::Scalar(0, 255, 0));

        cv::imshow("ROI TRACK", H);
        cv::imshow("GAUSSIAN MUL", result_final_filtered);
//        cv::waitKey(1);

        return {new_peak_filtered + cv::Point(roi.x, roi.y), max};
    }

    cv::Point multi_conv(cv::Mat eros, int width, int height){

        auto p = convolution(eros, filter_set[make_pair(width,height)]);

        cv::Mat dog_filter_grey;
        cv::normalize(filter_set[make_pair(width,height)], dog_filter_grey, 0, 255, cv::NORM_MINMAX);
        cv::Mat vis_ellipse, color_ellipse;
        dog_filter_grey.convertTo(vis_ellipse, CV_8U);
        cv::cvtColor(vis_ellipse,color_ellipse, cv::COLOR_GRAY2BGR);
        cv::imshow("ell", color_ellipse);

        if(p.s > 0){
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

        filter_bank_min = 5;
        filter_bank_max = 51;
        createFilterBank(filter_bank_min, filter_bank_max);

//        cv::Mat vis_ellipse, color_ellipse;
//        cv::Mat ellipse = createEllipse(1001);
//        cv::normalize(ellipse, ellipse, 0, 255, cv::NORM_MINMAX);
//        ellipse.convertTo(vis_ellipse, CV_8U);
//
//        cv::cvtColor(vis_ellipse,color_ellipse, cv::COLOR_GRAY2BGR);
//        cv::imshow("ell", color_ellipse);


//        myfile.open("/data/kalman.txt");
//        if (!myfile.is_open())
//        {
//            yError()<<"Could not open file for kalman prediction and correction";
//            return;
//        }

        first_time = yarp::os::Time::now();
    }

    void updateROI(Point2d position, int width, int height){

        float u = position.x;
        float v = position.y;

        //puck_size = 0.1*v;
        int roi_width = factor*width;

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

        updateROI(starting_position, 19, 38);

        puck_meas = starting_position;
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

    cv::Mat createCustom(int width, int height){

        cv::Point2d origin((width)/2, (height)/2);

        cv::Mat ell_filter = cv::Mat::zeros(height, width, CV_32F);

        for(int x=0; x< ell_filter.cols; x++) {
            for(int y=0; y< ell_filter.rows; y++) {
                double dx = (pow(x,2) -2*origin.x*x + pow(origin.x,2))/pow((width)/2,2);
                double dy = (pow(y,2) -2*origin.y*y + pow(origin.y,2))/pow((height)/2,2);
                double value = dx+ dy;
                if(value > 1)
                    ell_filter.at<float>(y, x) = 0;
                else if (value > 0.6 && value<=1)
                    ell_filter.at<float>(y, x) = 1;
                else
                    ell_filter.at<float>(y, x) = 0;

            }
        }

//        for(int i=0; i<ell_filter.rows;i++){
//            for(int j=0; j<ell_filter.cols;j++){
//                std::cout<<ell_filter.at<float>(i,j)<<" ";
//            }
//            std::cout<<std::endl;
//        }

        return ell_filter;
    }

    cv::Mat createEllipse(int puck_size){

        double height = puck_size;
        double width = 1.5*puck_size;
        cv::Point origin((width)/2, (height)/2);

        cv::Mat ell_filter = cv::Mat::zeros(height, width, CV_32F);

        for(int x=0; x< ell_filter.cols; x++) {
            for(int y=0; y< ell_filter.rows; y++) {
                double dx = (pow(x,2) -2*origin.x*x + pow(origin.x,2))/pow((width)/2,2);
                double dy = (pow(y,2) -2*origin.y*y + pow(origin.y,2))/pow((height)/2,2);
                double value = dx+ dy;
                if(value > 1)
                    ell_filter.at<float>(y, x) = 0;
                else if (value > 0.7 && value<=1)
                    ell_filter.at<float>(y, x) = 1;
                else
                    ell_filter.at<float>(y, x) = 0;

            }
        }

//        for(int i=0; i<ell_filter.rows;i++){
//            for(int j=0; j<ell_filter.cols;j++){
//                std::cout<<ell_filter.at<float>(i,j)<<" ";
//            }
//            std::cout<<std::endl;
//        }

        return ell_filter;
    }

    cv::Mat createEllipse2(int filter_width, int filter_height){

        Mat g1 = getGaussianKernel(filter_height/2, 0.2*filter_height, CV_32F) * getGaussianKernel(filter_width/2, 0.2*filter_width, CV_32F).t();
        cv::normalize(g1, g1, 0, 1, cv::NORM_MINMAX);
        Mat g2 = getGaussianKernel(filter_height/2, 0.1*filter_height, CV_32F) * getGaussianKernel(filter_width/2, 0.1*filter_width, CV_32F).t();
        cv::normalize(g2, g2, 0, 1, cv::NORM_MINMAX);
        Mat dog_filter = g1 - g2;

//        cv::imshow("g1", g1);
//        cv::imshow("g2", g2);

        cv::Mat dog_filter_norm, dog_filter_grey;
        cv::normalize(dog_filter, dog_filter_norm, 0, 1, cv::NORM_MINMAX);

//        for(int i=0; i<dog_filter_norm.rows;i++){
//            for(int j=0; j<dog_filter_norm.cols;j++){
//                std::cout<<dog_filter_norm.at<float>(i,j)<<" ";
//            }
//            std::cout<<std::endl;
//        }
//        std::cout<<std::endl;
        for(int i=0; i<dog_filter_norm.rows;i++){
            for(int j=0; j<dog_filter_norm.cols;j++){
                if(dog_filter_norm.at<float>(i,j)>=0.9)
                    dog_filter_norm.at<float>(i,j)=1;
                else
                    dog_filter_norm.at<float>(i,j)=0;
            }
        }
        return dog_filter_norm;
    }



    void track(cv::Mat eros, double dT){

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

        double width = 6.819010562059759-0.0015986852510628132*last_x+0.1246702563975636*last_y;
        double height = 1.4487326851033693-0.0012157382868591081*last_x+0.0927668580003834*last_y;
        height = 2 * floor(height/2) + 1;
        width = 2 * floor(width/2) + 1;

        if(height<7)
            height = 7;
        if(width<7)
            width = 7;
        puck_meas = multi_conv(eros, width, height);
//        cv::Point2d puck_pred = KalmanPrediction(dT);
//        puck_corr = KalmanCorrection(puck_meas);

        updateROI(puck_meas, width, height);

//        cv::cvtColor(eros, eros_bgr, cv::COLOR_GRAY2BGR);
//        cv::circle(eros_bgr, puck_corr, 5, cv::Scalar(255, 0, 255), cv::FILLED);
//        cv::circle(eros_bgr, puck_pred, 5, cv::Scalar(0, 255, 255), cv::FILLED);
//        cv::circle(eros_bgr, puck_meas, 5, cv::Scalar(0, 0, 255), cv::FILLED);
//        cv::rectangle(eros_bgr, roi, cv::Scalar(0,255,0));

//        cv::imshow("FULL TRACK", eros_bgr);

//        yInfo()<<puck_size;
//        yInfo()<<"("<<puck_meas.x<<","<<puck_meas.y<<") ("<<kf.statePre.at<float>(0)<<","<<kf.statePre.at<float>(1)<<") ("<<kf.statePost.at<float>(0)<<","<<kf.statePost.at<float>(1)<<")";
////        yInfo()<<puck_meas.x<<","<<puck_meas.y<<","<<puck_pred.x<<","<<puck_pred.y<<","<<puck_corr.x<<","<<puck_corr.y;
//        myfile<<puck_size<<","<<(yarp::os::Time::now()-first_time)<<","<<puck_meas.x<<","<<puck_meas.y<<","<<puck_pred.x<<","<<puck_pred.y<<","<<puck_corr.x<<","<<puck_corr.y<<endl;
    }

    cv::Point getPosition(){
        return puck_meas;
    }


};

class asynch_thread:public Thread{

private:
    cv::Mat eros;
    bool tracking_status;
    std::mutex *m2;

    tracking tracker;
    detection detector;
protected:


public:
    asynch_thread(){}

    void run();
    void initialise(cv::Mat &eros, int init_filter_width, cv::Rect roi, double thresh, std::mutex *m2);
    void setStatus(int tracking);
    int getStatus();
    cv::Point getState();
};

class puckPosModule:public RFModule, public Thread{

private:

    bool pause, first_it, success;
    std::mutex m, m2;
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