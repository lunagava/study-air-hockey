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

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>

using namespace ev;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;


class droppingModule: public yarp::os::RFModule, public yarp::os::Thread {

    yarp::sig::Matrix events_acquisition_map;
    Stamp ystamp;
    yarp::sig::ImageOf<yarp::sig::PixelBgr> droppingMap;
    cv::Mat droppingImg;
    double period;
    AE ev;
    deque<AE> out_queue;

    std::string dropping_file;

    vReadPort<vector<AE> > input_port;
    vWritePort output_port;
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > image_out;

    auto readEgoMotionPixels() {

        std::ifstream fin(dropping_file);
        if (fin.is_open()) {
            std::vector<double> x, y;

            double read;
            while (!fin.eof()) {
                fin >> read;
                x.push_back(read);
                fin >> read;
                y.push_back(read);
            }

            events_acquisition_map = yarp::math::zeros(304, 240);

            for (auto i=0;i<x.size();i++){
                events_acquisition_map(x[i], y[i]) = 1;
            }

            fin.close();
            yInfo() << "Table successfully read from" << dropping_file;
            return true;
        } else {
            yError() << "Unable to read from file" << dropping_file;
            return false;
        }
    }

    bool configure(yarp::os::ResourceFinder &rf) {

        // options and parameters
        period = rf.check("period", yarp::os::Value(0.01)).asDouble();
        dropping_file = rf.getHomeContextPath() + "/" +
                     rf.check("table-file", yarp::os::Value("/egoMotion_50.tsv")).asString();

        // module name and control
        setName((rf.check("name", Value("/dropping")).asString()).c_str());

        if (!input_port.open(getName() + "/AE:i"))
            return false;

        if(!output_port.open(getName() + "/AE:o"))
            return false;

        if (!image_out.open(getName() + "/image:o")) {
            yError() << "Can't open output image port for visualization";
            return false;
        }

        if (!readEgoMotionPixels()) {
            return false;
        }

        droppingMap.resize(304, 240);

        return Thread::start();
    }


//************************************************ RUN *****************************************************

    void run() {

        cv::Mat frame;
        Size imsize;

        imsize.height = 304;
        imsize.width = 240;
        frame = Mat(imsize, CV_32F);
        frame.setTo(0.0f);

        for (int i=0;i<events_acquisition_map.rows(); i++){
            for (int j=0;j<events_acquisition_map.cols(); j++) {
                if (events_acquisition_map(i,j)==1)
                    frame.at<float>(i,j) = 255;
            }
        }

        imwrite("map.jpg", frame);

        while (Thread::isRunning()) {

            const vector<AE> *q = input_port.read(ystamp);
            if (!q || Thread::isStopping()) return;

            int qs = input_port.queryunprocessed() - 1; // how many packets we have processed yet
            if (qs < 1) qs = 1; // if we do not have any queues
            if (qs > 4) yWarning() << qs;

            for (int i = 0; i < qs; i++) {
                for (auto &v : *q) {
                    if (events_acquisition_map(v.x,v.y) == 0){

                        ev.x = v.x;
                        ev.y = v.y;
                        ev.stamp = v.stamp;
                        out_queue.push_back(v);
                        output_port.write(out_queue, ystamp);
                        out_queue.clear();

                        yarp::sig::PixelBgr &ePix = droppingMap.pixel(v.x,v.y);
                        ePix.b = ePix.g = ePix.r = 255;
                    }
                }
            }
        }

    }

    double getPeriod() {
        return period;

    }

    bool updateModule() {

        yarp::sig::ImageOf<yarp::sig::PixelBgr> &display = image_out.prepare();
        display = droppingMap;
        droppingMap.zero();
        droppingImg = cv::cvarrToMat((IplImage *) display.getIplImage());
        cv::applyColorMap(droppingImg, droppingImg, cv::COLORMAP_BONE);

        image_out.write();

        return Thread::isRunning();
    }

    bool close() {

        input_port.close();
        output_port.close();

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
    droppingModule tracker;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return tracker.runModule(rf);
}