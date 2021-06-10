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

#ifndef __TRACKER__
#define __TRACKER__

#include <yarp/os/all.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/all.h>

#include <event-driven/all.h>
#include "event-driven/vIPT.h"

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

#include "../../../spline/src/spline.h"

using namespace ev;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/*////////////////////////////////////////////////////////////////////////////*/
// ROIQ
/*////////////////////////////////////////////////////////////////////////////*/

class roiq {

public:

    deque<LabelledAE> q;
    unsigned int n;
    yarp::sig::Vector roi;
    yarp::sig::Vector roi_hand;

    roiq() {
        roi.resize(4);
        roi_hand.resize(4);
        n = 1000;
        roi[0] = 0; roi[1] = 1000;
        roi[2] = 0; roi[3] = 1000;
//        roi_hand[0] = 91; roi_hand[1] = 191;//141
//        roi_hand[2] = 139; roi_hand[3] = 239;//189
    }

    void setSize(unsigned int value) {
        n = value;
        int deleted =0;
        //cout << "-----------------------------------------"<<endl;
        //cout << "queue size: "<< q.size() << endl;
        while(q.size() > n){
            //cout << "x: "<<  q.back().x << ", y: "<<  q.back().y << ", disp: "<<q.back().ID<<endl;
            q.pop_back();
            deleted++;
        }

        //cout <<"pop-back: "<<deleted<< endl;
    }

    yarp::sig::Vector setROI(int xl, int xh, int yl, int yh) {
        roi[0] = xl; roi[1] = xh;
        roi[2] = yl; roi[3] = yh;

        return roi;
    }

    yarp::sig::Vector getROI(){
        return roi;
    }

    void setROI_hand(int xl, int xh, int yl, int yh){
        roi_hand[0] = xl; roi_hand[1] = xh;
        roi_hand[2] = yl; roi_hand[3] = yh;
    }

    int add(const LabelledAE &v) {

        if (v.x < roi[0] || v.x > roi[1] || v.y < roi[2] || v.y > roi[3]) {
            return 0;
        }
        if (v.x>roi_hand[0] && v.x<roi_hand[1] && v.y>roi_hand[2] && v.y<roi_hand[3]) {
            return 0;
        }
        q.push_front(v);

        return 1;
    }

    void remove_handEvents(){
        for(auto &v : q) {
            if (v.x>roi_hand[0] && v.x<roi_hand[1] && v.y>roi_hand[2] && v.y<roi_hand[3]) {
//                cout << "hand events: x="<<v.x << ", y="<<v.y<<endl;
                q.pop_front();
            }
        }
    }

    void clean(){

//           q.clear();

        // leave only the events that are inside the last ROI
        deque<LabelledAE> _q;
        for(auto &v : q) {
            if(v.x < roi[0] || v.x > roi[1] || v.y < roi[2] || v.y > roi[3])
                continue;
            else
                _q.push_back(v);
        }
        q = _q;
    }

    void clear(){
        q.clear();
    }

};

/*////////////////////////////////////////////////////////////////////////////*/
// CENTER-OF-MASS TRACKER
/*////////////////////////////////////////////////////////////////////////////*/

class trackerModule : public yarp::os::RFModule, public yarp::os::Thread {

private:

    // data structures and ports
    roiq qROI;
    std::mutex m;

    resolution res;

    vReadPort<vector<AE> > input_port;
    vWritePort output_port;
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > image_out;
    BufferedPort<Bottle> posObj_port;
    BufferedPort<Bottle> hand_location;

    std::ofstream myfile;

    AE ev;
    Stamp ystamp;
    deque<AE> out_queue;

    bool stationary;

    //variables
    int n_rate, n_mass, n_objects, numEventsAccepted, numNewEvents;
    double widthROI, heightROI;
    double min_widthROI, min_heightROI, max_widthROI, max_heightROI;
    double prev_t;
    double reset_time;
    bool tracking, detected;
    bool visualization; // activate if you want to visualize geometric elements in yarp image
    bool start = true;

    double n_events_insideROI;
    int n_events_acquired, n_events_acquired_insideROI, n_events_acquired_insideROI1, n_events_acquired_insideROI2;

    yarp::sig::Vector ROI, currentROI;
    cv::Mat trackImg;
    int leftRect, rightRect, bottomRect, topRect;
    int n_iterations = 0;

    unsigned int i = 0;

    std::vector<int> s1_x, s1_y, s2_x, s2_y; // vector to store positions and disparities of two sets of events

    // IMAGE MOMENTS
    double m00; // zero-order moment
    double m10, m01; // first-order moments
    double m20, m02, m11; // second-order moments
    cv::Point2d COM, COM_prec;
    cv::Point2d COM_insideROI;

    // ELLIPSE PARAMETERS
    double thetaRad; // orientation of ellipse in radians
    double l; double w; // semi-major and semi-minor axis of ellipse

    // TRACKING
    double y_hand_position;
    std::vector <double> x_insideROI, y_insideROI;
    double std;
    double handROI_width, handROI_height;
    int hand_visible;

    std::string table_file;
    std::vector<std::shared_ptr<tk::spline>> interp;
    double y_min, y_max;

    double x_var, y_var;

    void resetTracker();
    auto readTable();
    double compute_std(roiq qROI, cv::Point avg);
    std::tuple <double, double> compute_stdev(roiq qROI, cv::Point avg);
    std::tuple <double, double, double, double, double> computeCOM(roiq qROI);
    std::tuple <double, double, double> ellipseParam(double m00, double m10, double m01, double m11, double m20, double m02);

protected:

    yarp::sig::ImageOf<yarp::sig::PixelBgr> trackMap;
    int Xlimit, Ylimit;

public:
    Stamp local_stamp;

    // constructor
    trackerModule() : Xlimit(304), Ylimit(240) {
        cout << "inside constructor";
        trackMap.resize(Xlimit, Ylimit);
    }

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual void onStop();
    virtual double getPeriod();
    virtual bool updateModule();
    void run();

};

#endif
//empty line to make gcc happy
