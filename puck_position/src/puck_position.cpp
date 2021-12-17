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

    roi_init = cv::Rect(20, 150, 550, 80);

    yarp::os::Network::connect("/atis3/AE:o", getName("/AE:i"), "fast_tcp");

    cv::Mat temp = EROS_vis.getSurface();
    eros_thread.initialise(temp);
    eros_thread.start();

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
            for(auto &v : *q){

//                if (v.stamp >= p0.start_ts)
//                  if (v.x > p0.start_x - roi_width && v.x < p0.start_x + roi_width && v.y > p0.start_y - roi_height && v.y < p0.start_y + roi_height)

                // call EROSupdate
                success = EROS_vis.EROSupdate(v.x, v.y);
            }
        //}


    }
}

double puckPosModule::getPeriod() {
    return 0.01;
}

bool puckPosModule::updateModule() {

    cv::waitKey(1);
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

void asynch_thread::initialise(cv::Mat &eros)
{

    this->eros = eros; //shallow copy (i.e. pointer)

    int filter_width = 23;
    double fw2 = (double)filter_width/2.0;
    dog_filter = cv::Mat (filter_width, filter_width, CV_32F);

    for(int x = 0; x < dog_filter.cols; x++) {
        for(int y = 0; y < dog_filter.rows; y++) {
            float &p = dog_filter.at<float>(y, x);
            double res = sqrt(pow(x-filter_width/2, 2.0) + pow(y-filter_width/2, 2.0));
            if(res > fw2-1 + 1.5)
                p = 0.0;
            else if (res < fw2-1 - 1.5)
                p = -1.0;
            else
                p = 1.0;
        }
    }
    cv::Mat temp;
    cv::normalize(dog_filter, temp, 1, 0, cv::NORM_MINMAX);

    cv::namedWindow("DoG", cv::WINDOW_NORMAL);
    cv::imshow("DoG", temp);
    cv::waitKey(1);
}

void asynch_thread::run() {

    cv::Mat intern, surface_matrix;
    double min, max;
    cv::Point min_loc, max_loc;
    Mat result_conv, result_vis;

    while(!isStopping())
    {

        // get surface;
        cv::GaussianBlur(eros, intern, cv::Size(7, 7), 0);

        intern.convertTo(surface_matrix, CV_32F);

        // do convolution
        cv::filter2D(surface_matrix, result_conv, -1, dog_filter, cv::Point(-1, -1), 0, cv::BORDER_ISOLATED);

//        cv::Rect();

        cv::minMaxLoc(result_conv, &min, &max, &min_loc, &max_loc);
//        yInfo() << max;

       cv::normalize(result_conv, result_conv, 255, 0, cv::NORM_MINMAX);
//        result_conv *= (255.0/3000.0);
//        cv::Mat result_vis;
        result_conv.convertTo(result_vis, CV_8U);

        cv::cvtColor(result_vis, result_vis, cv::COLOR_GRAY2BGR);
//        // find the highest peak

//        if(max > 3000)
            cv::circle(result_vis, max_loc, 5, cv::Scalar(255, 0, 0), cv::FILLED);
//
//        cv::rectangle(result_vis, roi_init, cv::Scalar(0, 255, 0), 1);

        // setROI of the next EROS...
//        p0.start_x = p0.start_x + displacement;
//        p0.start_y = p0.start_y + displacement;

//        cv::namedWindow("RESULT", cv::WINDOW_NORMAL);
//        cv::imshow("RESULT", result_vis);
//        cv::waitKey(3);

        // visualize EROS
        cv::namedWindow("EROS", cv::WINDOW_NORMAL);
        cv::imshow("EROS", intern);

        cv::namedWindow("RESULT", cv::WINDOW_NORMAL);
        cv::imshow("RESULT", result_vis);
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
