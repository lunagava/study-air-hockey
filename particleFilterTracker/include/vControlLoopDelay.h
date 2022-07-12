/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
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
#pragma once

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <event-driven/core.h>
#include "vParticle.h"
#include <mutex>
#include <deque>
#include <array>
#include <deque>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>

/*////////////////////////////////////////////////////////////////////////////*/
// ROIQ
/*////////////////////////////////////////////////////////////////////////////*/
class roiq
{
public:

    std::deque<ev::AE> q;
    unsigned int n;
    yarp::sig::Vector roi;
    bool use_TW;

    roiq();
    void setSize(unsigned int value);
    void setROI(int xl, int xh, int yl, int yh);
    yarp::sig::Vector getROI();
    int add(const ev::AE &v);

};

/*////////////////////////////////////////////////////////////////////////////*/
// DELAYCONTROL
/*////////////////////////////////////////////////////////////////////////////*/

class delayControl : public yarp::os::RFModule, public yarp::os::Thread
{
private:

    //data structures and ports
    yarp::os::Port rpcPort; 
    ev::window<ev::AE> input_port;
    yarp::os::BufferedPort< yarp::sig::ImageOf<yarp::sig::PixelBgr> > debug_port;
    yarp::sig::ImageOf<yarp::sig::PixelBgr> ptMap;
    roiq qROI;
    vParticlefilter vpf;
    std::mutex m;
    cv::Mat ptImg;
    cv::Point pos_click, pos_click_prec;

    //variables
    ev::resolution res;
    double avgx, avgy, avgr;
    int maxRawLikelihood;
    double gain;
    double detectionThreshold;
    double resetTimeout;
    double motionVariance;
    int batch_size;
    double output_sample_delta;
    double start_time;

    //diagnostics
    double filterPeriod;
    unsigned int targetproc;
    double dx;
    double dy;
    double dr;
    double px, py, pr;
    ev::benchmark cpuusage;
    std::deque< std::array<double, 4> > data_to_save;
    std::ofstream fs, file2;
    double time_init;

    static void CallBackFunc(int event, int x, int y, int flags, void* param);

public:

    delayControl() {
        ptMap.resize(640, 480);
    }

    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual double getPeriod();
    virtual bool updateModule();
    virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    bool open(std::string name, unsigned int qlimit = 0);
    void performReset(int x = -1, int y = -1, int r = -1);
    void setFilterInitialState(int x, int y, int r);

    void setMinRawLikelihood(double value);
    void setMaxRawLikelihood(int value);
    void setNegativeBias(int value);
    void setInlierParameter(int value);
    void setMotionVariance(double value);
    void setTrueThreshold(double value);
    void setAdaptive(double value = true);
    void setOutputSampleDelta(double value);

    void setGain(double value);
    void setMinToProc(int value);
    void setResetTimeout(double value);

    yarp::sig::Vector getTrackingStats();

    //bool threadInit();
    void pause();
    void resume();
    void onStop();
    void run();
    //void threadRelease();


};
