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

#include "gaussianTracker.h"

using namespace ev;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

bool GaussianTracker::configure(yarp::os::ResourceFinder& rf) {

    // module name
    setName((rf.check("name", Value("/gaussianTracker")).asString()).c_str());

    width = rf.check("width", yarp::os::Value(640)).asInt();
    height = rf.check("height", yarp::os::Value(480)).asInt();
    alphaShape =rf.check("alphaShape", yarp::os::Value(0.001)).asDouble();
    //how quickly cluster position changes given new information
    alphaPos = rf.check("alphaPos", yarp::os::Value(0.1)).asDouble();
    //at what threshold a cluster becomes active given activity
    SigX = rf.check("sigX", yarp::os::Value(19)).asDouble();
    SigY = rf.check("sigY", yarp::os::Value(29)).asDouble();
    SigXY = rf.check("sigXY", yarp::os::Value(0)).asDouble();
    //are clusters fixed to circular gaussians?
    Fixedshape = rf.check("fixedShape", yarp::os::Value(false)).asBool();
    max_dist = rf.check("maxDist", yarp::os::Value(10)).asDouble();
    //first detected coordinates
    startX = rf.check("xstart", yarp::os::Value(96)).asDouble();
    startY = rf.check("ystart", yarp::os::Value(214)).asDouble();
    startTime = rf.check("tstart", yarp::os::Value(0.76496)).asDouble();
    n_exp = rf.check("num_exp", yarp::os::Value(1)).asDouble();
    n_seq = rf.check("num_seq", yarp::os::Value(16)).asDouble();

    blobtracker.initialiseShape(SigX, SigY, SigXY, alphaPos, alphaShape, Fixedshape);
    blobtracker.initialisePosition(startX, startY);

    /* now open input and output port */
    if (!input_port.open(getName() + "/AE:i"))
        return false;

//    if(!image_out.open(getName() + "/image:o")){
//        yError()<<"Can't open output image port for visualization";
//        return false;
//    }

//    file.open("/data/workshop/data_CL.txt");

    file.open("/data/iros_datasets/live_test/clust"+std::to_string(n_exp)+".txt");
    file2.open("/data/iros_datasets/exp2/vCluster/failure1.txt");

//    file.open("/data/iros_datasets/exp"+std::to_string(n_exp)+"/vCluster/Clust"+std::to_string(n_seq)+".txt");
    if (!file.is_open())
    {
        yError()<<"Could not open file for cluster";
        return false;
    }

    if (!file2.is_open())
    {
        yError()<<"Could not open file for cluster";
        return false;
    }


    yarp::os::Network::connect("/atis3/AE:o", getName("/AE:i"), "fast_tcp");
//    yarp::os::Network::connect(getName("/image:o"), "/clustView", "fast_tcp");

    clustImg = cv::Mat::zeros(480,640,CV_8UC3);
    cv::namedWindow("clustView", cv::WINDOW_NORMAL);
    cv::imshow("clustView", cv::WINDOW_NORMAL);

    pause = false;

    return Thread::start();
}

void GaussianTracker::CallBackFunc(int event, int x, int y, int flags, void* param)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        Point*p = (Point*)param;
        p->x = x;
        p->y = y;
    }
}

void GaussianTracker::run() {

//    yInfo()<<startTime<<" "<<startX<<","<<startY<<" 0";

    file<<startTime<<" "<<startX<<" "<<startY<<" "<<0.0<<std::endl;

    double time_offset = -1.0;
    double time_now = -1.0;
    double current_time;
    double time_init;

    while(time_now-time_offset <= startTime) {

        ev::info read_stats = input_port.readChunkN(1);
        if(input_port.isStopping()) return;
        for(auto a = input_port.begin(); a != input_port.end(); a++) {
//            yInfo()<<(*a).y<<(*a).x;
            clustImg.at<cv::Vec3b>((*a).y, (*a).x) = cv::Vec3b(255,255,255);
//            yarp::sig::PixelBgr &ePix1 = clustMap.pixel((*a).x,(*a).y);
//            ePix1.r = ePix1.b = ePix1.g = 255;
        }
        time_now = input_port.begin().timestamp();
        if(time_offset < 0) {
            time_offset = input_port.begin().timestamp();
        }
    }

    time_init = yarp::os::Time::now();

    while (Thread::isRunning()) {
\
        if (pos_click.x != pos_click_prec.x && pos_click.y!=  pos_click_prec.y){
//        yInfo()<<"clicked pos=("<<pos_click.x<<","<<pos_click.y<<")";
//            bool initialisation = blobtracker.initialisePosition(pos_click.x, pos_click.y);
//            yInfo()<<initialisation;
            file2<<yarp::os::Time::now()-time_init<<std::endl;

        }

        if(input_port.isStopping())
            break;

        ev::info read_stats = input_port.readChunkN(1);

        for(auto a = input_port.begin(); a != input_port.end(); a++) {

//            yarp::sig::PixelBgr &ePix1 = clustMap.pixel((*a).x,(*a).y);
//            ePix1.r = ePix1.b = ePix1.g = 255;
            clustImg.at<cv::Vec3b>((*a).y, (*a).x) = cv::Vec3b(255,255,255);

            // only among the Active and Inactive clusters
            if(blobtracker.dist2event((*a).x, (*a).y) < max_dist){
                blobtracker.addActivity((*a).x, (*a).y, a.timestamp(), 0.0, 0.0);
            }
            current_time = a.timestamp();
        }
//        file<< current_time - time_offset<<" "<< blobtracker.get_x()<<" "<< blobtracker.get_y()<<" "<< input_port.stats_unprocessed().duration<<std::endl;

//        yInfo()<<input_port.getUnprocessedDelay();


        if (pause)
            m.lock();
        else
            m.unlock();
    }



}

double GaussianTracker::getPeriod() {
    return 0.03;
}

bool GaussianTracker::updateModule() {

    char key = 0;
    m2.lock();

//    yarp::sig::ImageOf<yarp::sig::PixelBgr> &display= image_out.prepare();
//    display = clustMap;
//    cv::Mat clustImg = cv::cvarrToMat((IplImage *)display.getIplImage());

    double sig_x2_ = blobtracker.get_sigx2();
    double sig_y2_ = blobtracker.get_sigy2();
    double sig_xy_ = blobtracker.get_sigxy();

    double tmp = sqrt( (sig_x2_ - sig_y2_) * (sig_x2_ - sig_y2_) + 4*sig_xy_*sig_xy_ );
    double l_max = 0.5*(sig_x2_ + sig_y2_ + tmp);
    double l_min = 0.5*(sig_x2_ + sig_y2_ - tmp);

    if(l_min < -5) {
        yWarning() << "l_min error: shape distorted";
    }

    double a = sqrt(std::fabs(l_max)) * 5;
    double b = sqrt(std::fabs(l_min)) * 5;
    double alpha = 0.5*atan2f(2*sig_xy_, sig_y2_ - sig_x2_);

//    yInfo()<<blobtracker.get_x()<<" "<<blobtracker.get_y();

    alpha = alpha * 180 / M_PI; //convert to degrees for openCV ellipse function
    cv::ellipse(clustImg, cv::Point(blobtracker.get_x(), blobtracker.get_y()), cv::Size(a,b), alpha, 0, 360, cv::Scalar(44,160,44), 2);
    cv::circle(clustImg, cv::Point(blobtracker.get_x(), blobtracker.get_y()), 5, cv::Scalar(44,160,44), -1);

    setMouseCallback("clustView", CallBackFunc, &pos_click);
    cv::imshow("clustView", clustImg);
//    cv::waitKey(1);

    pos_click_prec = pos_click;

    key = cv::waitKey(1);
    m2.unlock();

    if (key == 'p') {  // press p to pause
        pause = !pause;
        if (pause){
            m.try_lock();
        }
        else{
            m.unlock();
        }
    } else if (key == 'n') { // next
        m.unlock();
    }
    clustImg=0;

//    image_out.write();
//    clustMap.zero();

    return Thread::isRunning();
}

bool GaussianTracker::interruptModule() {
    return Thread::stop();
}

void GaussianTracker::onStop() {
    input_port.stop();
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
    GaussianTracker gaussianTracker;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return gaussianTracker.runModule(rf);
}
