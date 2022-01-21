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

#include <yarp/math/Math.h>
#include <yarp/os/all.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/all.h>

#include <event-driven/all.h>

#include <iostream>
#include <tuple>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace ev;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;


class egoMotionModule: public yarp::os::RFModule, public yarp::os::Thread {

    cv::Mat frequencyEvents;
    Stamp ystamp;

//    std::ofstream myfile;
    int n_events;

    vReadPort<vector<AE> > input_port;

    bool configure(yarp::os::ResourceFinder &rf) {

        // options and parameters


        // module name and control
        setName((rf.check("name", Value("/puck_far")).asString()).c_str());

        if (!input_port.open(getName() + "/AE:i"))
            return false;

        n_events=0;

//        myfile.open("/code/luna/study-air-hockey/heat_map.csv");
//        if (!myfile.is_open())
//        {
//            yError()<<"Could not open file for printing the count of events due to ego-motion";
//            return -1;
//        }


        return Thread::start();
    }

//************************************************ RUN *****************************************************

    void run() {


        while (Thread::isRunning()) {

            cv::Mat frequencyEvents(50, 150, CV_8U);
//            frequencyEvents.setTo(0);

            const vector<AE> *q = input_port.read(ystamp);
            if (!q || Thread::isStopping()) return;

            int qs = input_port.queryunprocessed() - 1; // how many packets we have processed yet
            if (qs < 1) qs = 1; // if we do not have any queues
            if (qs > 4) yWarning() << qs;

            for (int i = 0; i < qs; i++) {
                for (auto &v : *q) {

                    if (v.y>25 && v.y<45 && v.x>100 && v.x<200){
                        n_events += 1;
                        frequencyEvents.at<unsigned char>(v.y-25, v.x-100) = 255;
                    }

                }
            }

            cv::namedWindow("roi_far_puck", cv::WINDOW_NORMAL);
            cv::imshow("roi_far_puck", frequencyEvents);
            cv::waitKey(3);

        }

    }

    double getPeriod() {
        return 0.1;
    }

    bool updateModule() {

//        frequencyEvents.convertTo(frequencyEvents, CV_8UC1,abs(dWTA-dmin)*255/dmaxPlot);

//        cv::Mat frequencyEvents(240, 304, CV_8U);

//        cv::namedWindow("roi_far_puck", cv::WINDOW_NORMAL);
//        cv::imshow("roi_far_puck", frequencyEvents);
//        cv::waitKey(3);

        yInfo()<<n_events;

//        frequencyEvents.setTo(0);
        n_events = 0;

        return Thread::isRunning();
    }

    bool close() {


//        myfile.close();

        input_port.close();

    }

};

/*////////////////////////////////////////////////////////////////////////////*/
// MAIN
/*////////////////////////////////////////////////////////////////////////////*/

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
//    rf.setDefault("table-file", "lut.tsv");
    rf.configure(argc, argv);

    /* create the module */
    egoMotionModule tracker;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return tracker.runModule(rf);
}