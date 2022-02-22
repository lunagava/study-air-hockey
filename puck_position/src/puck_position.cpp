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

#include "puck_position.h"

using namespace ev;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

bool puckPosModule::configure(yarp::os::ResourceFinder& rf) {

    // options and parameters
    w = rf.check("w", Value(640)).asInt();
    h = rf.check("h", Value(480)).asInt();
    n_trial = rf.check("n_trial", Value(1)).asInt();
    n_exp = rf.check("n_exp", Value(1)).asInt();

    // module name
    setName((rf.check("name", Value("/puck_position")).asString()).c_str());

    if (!input_port.open(getName() + "/AE:i"))
        return false;

    EROS_vis.init(w, h, 7, 0.1);
//    EROS_vis.init(w, h, 5, 0.3);

    yarp::os::Network::connect("/atis3/AE:o", getName("/AE:i"), "fast_tcp");

    cv::Mat temp = EROS_vis.getSurface();
    eros_thread.initialise(temp, 19, cv::Rect(60, 150, 500, 100), 3000, &m2, n_trial, n_exp);
    eros_thread.start();

    pause = false;
    first_it = true;

//    cv::namedWindow("RESULT", cv::WINDOW_AUTOSIZE);
//    cv::moveWindow("RESULT", 300,300);
//
//    cv::namedWindow("DETECT_MAP", cv::WINDOW_NORMAL);
//    cv::moveWindow("DETECT_MAP", 500,500);
//
//    cv::namedWindow("ROI TRACK", cv::WINDOW_NORMAL);
//    cv::moveWindow("ROI TRACK", 0,0);
//
//    cv::namedWindow("ZOOM", cv::WINDOW_NORMAL);
//    cv::moveWindow("ZOOM", 1200,1200);

    cv::namedWindow("FINAL TRACK", cv::WINDOW_NORMAL);
    cv::moveWindow("FINAL TRACK", 600,600);

//    cv::namedWindow("GAUSSIAN MUL", cv::WINDOW_NORMAL);
//    cv::moveWindow("GAUSSIAN MUL", 1000,1000);
//
//    cv::namedWindow("ell_filter", cv::WINDOW_NORMAL);
//    cv::moveWindow("ell_filter", 1400,1400);

    return Thread::start();
}

void puckPosModule::run() {

    while (Thread::isRunning()) {

        int qs = std::max((int)input_port.getPendingReads(), 1);
        if (qs > 1)
            yWarning() << qs;

        //for(int i = 0; i < qs; i++) {

        ev::packet<AE> *q = input_port.read();
        static double first_instance = yarp::os::Time::now();
        eros_thread.setFirstTime(first_instance);
//        start_time_latency = yarp::os::Time::now();
//        eros_thread.setTimeLat(start_time_latency);
        if (!q) return;
        for(auto &v : *q)
            success = EROS_vis.EROSupdate(v.x, v.y);
        //}

        if (pause)
            m.lock();
        else
            m.unlock();

    }
}

double puckPosModule::getPeriod() {
    return 0.01;
}

bool puckPosModule::updateModule() {

//    cv::waitKey(1);
//    char key = 0;
//    m2.lock();

    cv::Mat eros_bgr, eros_filtered;
    cv::Mat eros_surface = EROS_vis.getSurface();
    cv::cvtColor(eros_surface, eros_bgr, cv::COLOR_GRAY2BGR);

    cv::Point puck_position = eros_thread.getState();

//    yInfo()<<puck_position.x<<" "<<puck_position.y;
    cv::circle(eros_bgr, puck_position,5, cv::Scalar(0,0,255), cv::FILLED);
//    cv::rectangle(eros_bgr, cv::Point(puck_roi.x, puck_roi.y), cv::Point(puck_roi.x+puck_roi.width, puck_roi.y+puck_roi.height), cv::Scalar(0,255,0), 3);
//    cv::rectangle(eros_bgr, cv::Point(60, 150), cv::Point(560, 250), cv::Scalar(255,0,0), 3);

    cv::GaussianBlur(eros_bgr, eros_filtered, cv::Size(5, 5), 0);

    cv::imshow("FINAL TRACK", eros_bgr);
    cv::waitKey(1);

//    yInfo()<<"Puck position"<<puck_position.x<<" "<<puck_position.y;

//    key = cv::waitKey(1);
//    m2.unlock();
//
//    if (key == 'p'){  // press p to pause
//        pause = !pause;
//        if(pause)
//            m.try_lock();
//        else
//            m.unlock();
//    }
//    else if(key == 'n'){ // next
//        m.unlock();
//    }
//    else if(key==32){
//        eros_thread.setStatus(0);
//    }

//    yInfo()<<"UPDATE MODULE";

    return Thread::isRunning();
}

bool puckPosModule::interruptModule() {
    eros_thread.stop();
    return Thread::stop();
}

void puckPosModule::onStop() {
    eros_thread.stop();
    input_port.close();
}

void asynch_thread::initialise(cv::Mat &eros, int init_filter_width, cv::Rect roi, double thresh, std::mutex *m2, int n_trial, int n_exp)
{
    this->eros = eros; //shallow copy (i.e. pointer)
    this->m2=m2;

    detector.initialize(init_filter_width, roi, thresh); // create and visualize filter for detection phase
    tracker.initKalmanFilter();

    first_instant = yarp::os::Time::now();

    file.open("/data/iros_datasets/exp"+std::to_string(n_exp)+"/trajs/trial"+std::to_string(n_trial)+".txt");
    if (!file.is_open())
    {
        yError()<<"Could not open file for kalman prediction and correction";
        return;
    }

}

void asynch_thread::setStatus(int tracking_status) {
    this->tracking_status=tracking_status;
}

void asynch_thread::setTimeLat(double start) {
    this->startLat=start;
}

double asynch_thread::getTimeLat(){
    return startLat;
}

void asynch_thread::setFirstTime(double first) {
    this->startTime=first;
}

double asynch_thread::getFirstTime(){
    return startTime;
}

int asynch_thread::getStatus(){
    return tracking_status;
}

cv::Point asynch_thread::getState(){
    return tracker.getPosition();
}

void asynch_thread::run() {

    cv::Mat eros_filtered, kernel, result_visualization, temp;
    double tic = yarp::os::Time::now();
    double detection_time, elapsed_time;
    bool first_detection=true;
    setStatus(0);

    while(!isStopping())
    {
        eros.copyTo(temp);
        cv::GaussianBlur(temp, eros_filtered, cv::Size(5, 5), 0);

        // --- DETECTION PHASE ----
//        m2->lock();
        if (!getStatus())
        {
            if(detector.detect(eros_filtered)){
                setStatus(1);
                tracker.resetKalman(detector.getDetection(), detector.getSize());
                if(first_detection==true){
                    detection_time = yarp::os::Time::now();
                    yInfo()<<detection_time-getFirstTime();
                    file<<detection_time-getFirstTime()<<","<<detector.getDetection().x<<","<<detector.getDetection().y<<",0"<<endl;
                    first_detection=false;
                }
//                yInfo()<<"first detected = ("<<detector.getDetection().x<<","<<detector.getDetection().y<<")";
            }
        }
        // ---- TRACKING PHASE ----
        else{

            // get from from main thread the last time you received a packet
//            double start_time_latency = getTimeLat();

            // port to open with this instance
//            ev::window<AE>();
//            w.getAll return information of data get input

            double dT = yarp::os::Time::now() - tic;
            tic += dT;
//            yInfo() << "Running at a cool " << 1.0 / dT << "Hz";
            puck_pos = tracker.track(eros_filtered, dT);

//            double end_time_latency = yarp::os::Time::now();
//            yInfo()<<1/(end_time_latency-start_time_latency);

            file<<yarp::os::Time::now()-detection_time<<","<<puck_pos.x<<","<<puck_pos.y<<","<<1.0 / dT<<endl;

            if (puck_pos.x<=5){
                setStatus(0);
            }
//            if(detector.detect(eros_filtered)){
//                setStatus(1);
//                tracker.resetKalman(detector.getDetection(), detector.getSize());
//                detection_time = yarp::os::Time::now();
////                yInfo()<<"first detected = ("<<detector.getDetection().x<<","<<detector.getDetection().y<<")";
//            }

        }
//        m2->unlock();
    }

}

int main(int argc, char *argv[]) {
    /* initialize yarp network */
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("event-driven");
//    rf.setDefaultConfigFile( "tracker.ini" );
    rf.setVerbose(false);
    rf.configure(argc, argv);

    /* create the module */
    puckPosModule puckpos;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return puckpos.runModule(rf);
}