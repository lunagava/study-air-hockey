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

    // module name
    setName((rf.check("name", Value("/puck_position")).asString()).c_str());

    if (!input_port.open(getName() + "/AE:i"))
        return false;

    EROS_vis.init(w, h, 5, 0.3);

    yarp::os::Network::connect("/atis3/AE:o", getName("/AE:i"), "fast_tcp");

    cv::Mat temp = EROS_vis.getSurface();
    eros_thread.initialise(temp, 19, cv::Rect(40, 150, 550, 100), 4000, &m2);
    eros_thread.start();

    pause = false;
    first_it = true;

//    cv::namedWindow("FULL TRACK", cv::WINDOW_NORMAL);
//    cv::moveWindow("FULL TRACK", 0,0);
//
//    cv::namedWindow("init filter", cv::WINDOW_AUTOSIZE);
//    cv::moveWindow("init filter", 300,300);
//
//    cv::namedWindow("ell", cv::WINDOW_AUTOSIZE);
//    cv::moveWindow("ell", 500,500);
//
//    cv::namedWindow("RESULT", cv::WINDOW_NORMAL);
//    cv::moveWindow("RESULT", 400,400);
//
    cv::namedWindow("ROI TRACK", cv::WINDOW_NORMAL);
    cv::moveWindow("ROI TRACK", 600,600);

    cv::namedWindow("GAUSSIAN MUL", cv::WINDOW_NORMAL);
    cv::moveWindow("GAUSSIAN MUL", 1000,1000);

    return Thread::start();
}

void puckPosModule::run() {

    while (Thread::isRunning()) {

        int qs = std::max((int)input_port.getPendingReads(), 1);
        if (qs > 1)
            yWarning() << qs;

        //for(int i = 0; i < qs; i++) {
        ev::packet<AE> *q = input_port.read();
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
    char key = 0;
    m2.lock();

    cv::Mat eros_bgr;
    cv::Mat eros_surface = EROS_vis.getSurface();
    cv::cvtColor(eros_surface, eros_bgr, cv::COLOR_GRAY2BGR);

    cv::Point puck_position = eros_thread.getState();

    cv::circle(eros_bgr, puck_position,5, cv::Scalar(0,0,255), cv::FILLED);
    cv::imshow("FULL TRACK", eros_bgr);

    key = cv::waitKey(1);
    m2.unlock();

    if (key == 'p'){  // press p to pause
        pause = !pause;
        if(pause)
            m.try_lock();
        else
            m.unlock();
    }
    else if(key == 'n'){ // next
        m.unlock();
    }
    else if(key==32){
        eros_thread.setStatus(0);
    }

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

void asynch_thread::initialise(cv::Mat &eros, int init_filter_width, cv::Rect roi, double thresh, std::mutex *m2)
{
    this->eros = eros; //shallow copy (i.e. pointer)
    this->m2=m2;

    detector.initialize(init_filter_width, roi, thresh); // create and visualize filter for detection phase
    tracker.initKalmanFilter();

}

void asynch_thread::setStatus(int tracking_status) {
    this->tracking_status=tracking_status;
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
    setStatus(0);

    while(!isStopping())
    {
        eros.copyTo(temp);
        cv::GaussianBlur(temp, eros_filtered, cv::Size(5, 5), 0);

        // --- DETECTION PHASE ----
        m2->lock();
        if (!getStatus())
        {
            if(detector.detect(eros_filtered)){
                setStatus(1);
                tracker.resetKalman(detector.getDetection(), detector.getSize());
                yInfo()<<"first detected = ("<<detector.getDetection().x<<","<<detector.getDetection().y<<")";
            }
        }
        // ---- TRACKING PHASE ----
        else{
            double dT = yarp::os::Time::now() - tic;
            tic += dT;
            yInfo() << "Running at a cool " << 1.0 / dT << "Hz";

            tracker.track(eros_filtered, dT);
//            if(detector.detect(eros_filtered)){
//                setStatus(1);
//                tracker.resetKalman(detector.getDetection(), detector.getSize());
////                yInfo()<<"first detected = ("<<detector.getDetection().x<<","<<detector.getDetection().y<<")";
//            }
        }
        m2->unlock();
//        yInfo() << "RUN ASYNCH THREAD";

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