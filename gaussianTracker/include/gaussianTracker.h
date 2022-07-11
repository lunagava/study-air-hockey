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

#ifndef __GAUSSIANTRACKER__
#define __GAUSSIANTRACKER__

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

#include "blobTracker.h"

using namespace ev;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

class GaussianTracker: public yarp::os::RFModule, public yarp::os::Thread {

private:

    ev::window<ev::AE> input_port;
//    yarp::os::BufferedPort <yarp::sig::ImageOf<yarp::sig::PixelBgr> > image_out;
//    yarp::sig::ImageOf<yarp::sig::PixelBgr> clustMap;
    cv::Mat clustImg;
    BlobTracker blobtracker;
    double max_dist;
    double alphaShape, alphaPos, SigX, SigY, SigXY, Fixedshape, startX, startY, startTime;
    double width, height;
    ofstream file, file2;
    int n_exp, n_seq;
    cv::Point pos_click, pos_click_prec;
    bool pause;
    std::mutex m, m2;


    static void CallBackFunc(int event, int x, int y, int flags, void* param);

protected:

public:

    // constructor
    GaussianTracker(){
//        clustMap.resize(640, 480);
    }

    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual void onStop();
    virtual double getPeriod();
    virtual bool updateModule();

    void run();

};

#endif
//empty line