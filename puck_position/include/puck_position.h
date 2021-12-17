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

class asynch_thread:public Thread{

private:
    cv::Mat eros;
    cv::Mat dog_filter;

protected:


public:
    asynch_thread(){}

    void run();
    void initialise(cv::Mat &eros);

};

struct startPos{
    int start_x = 150;
    int start_y = 150;
    int start_ts = 15;
};

class puckPosModule:public RFModule, public Thread{

private:

    bool success;
    int w, h;
    hpecore::surface EROS_vis;
    cv::Rect roi_init;


    ev::BufferedPort<AE> input_port;


    asynch_thread eros_thread;

    struct startPos p0;


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