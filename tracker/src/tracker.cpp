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

#include <tracker.h>

using namespace ev;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

auto trackerModule::readTable() {
    std::ifstream fin(table_file);
    if (fin.is_open()) {
        std::vector<double> y;
        std::vector<std::vector<double>> roi_hand(6);
        double read;
        while (!fin.eof()) {
            fin >> read;
            y.push_back(read);
            for (size_t j = 0; j < roi_hand.size(); j++) {
                fin >> read;
                roi_hand[j].push_back(read);
            }
        }
        y_min = *std::min_element(begin(y), end(y));
        y_max = *std::max_element(begin(y), end(y));
        for (const auto& q_:roi_hand) {
            interp.push_back(std::make_shared<tk::spline>(y, q_));
        }
        fin.close();
        yInfo() << "Table successfully read from" << table_file;
        return true;
    } else {
        yError() << "Unable to read from file" << table_file;
        return false;
    }
}

//*** CONFIGURE ***

bool trackerModule::configure(yarp::os::ResourceFinder &rf) {

    // options and parameters
    n_mass = rf.check("events", Value(200)).asInt();
    reset_time = rf.check("reset_time", Value(1.0)).asDouble();
    min_widthROI = rf.check("min_roi_width", Value(40)).asInt();
    min_heightROI = rf.check("min_roi_height", Value(40)).asInt();
    max_widthROI = rf.check("max_roi_width", Value(80)).asInt();
    max_heightROI = rf.check("max_roi_height", Value(80)).asInt();
    n_objects = rf.check("number_of_objects", Value(1)).asInt();
    visualization = rf.check("visualization", Value(false)).asBool();
    numEventsAccepted = rf.check("num_events_accepted_insideROI", Value(20)).asInt();
    numNewEvents = rf.check("new_events_insideROI", Value(4)).asInt();
    table_file = rf.findFile("table-file");

    if (!readTable()) {
        return false;
    }

    // module name and control
    setName((rf.check("name", Value("/tracker")).asString()).c_str());

    if (!input_port.open(getName() + "/AE:i"))
        return false;

    if (!output_port.open(getName() + "/AE:o"))
        return false;

    if (!posObj_port.open(getName() + "/posObj:o"))
        return false;

    if (!hand_location.open(getName() + "/handPos:i"))
        return false;

    if (!image_out.open(getName() + "/image:o")) {
        yError() << "Can't open output image port for visualization";
        return false;
    }

    res.height = 240;
    res.width = 304;

//    handROI_width = 100;
//    handROI_height = 150;

    ROI.resize(8);

    resetTracker();

    widthROI=min_widthROI;
    heightROI=min_heightROI;

    return Thread::start();
}

//************************************************ RUN *****************************************************

void trackerModule::run() {

    // read some data to extract the channel
    const vector<AE> *q = input_port.read(ystamp);
    if (!q || Thread::isStopping()) return;

    while (Thread::isRunning()) {
        int qs = input_port.queryunprocessed() - 1; // how many packets we have processed yet
        if (qs < 1) qs = 1; // if we do not have any queues
        if (qs > 4) yWarning() << qs;
        n_events_acquired = 0;
        n_events_acquired_insideROI = 0;
        // for all data that is currently queued -> add it to qROI
        m.lock();
        for (int i = 0; i < qs; i++) {
            const vector<AE> *q = input_port.read(ystamp);
            for (auto &v : *q) {
                n_iterations++;
                n_events_acquired++;
                if (v.x > ROI[0] && v.x < ROI[1] && v.y > ROI[2] && v.y < ROI[3]) {
                    n_events_acquired_insideROI++;
                }

                qROI.add(v);
            }
        }

        qROI.setSize(n_mass);

        // initialization
        m00 = qROI.q.size();
        n_events_insideROI = 0;

        // --------------------------------- GLOBAL ROI -----------------------------------
        std::tie(m10, m01, m11, m20, m02) = computeCOM(
                qROI); // function to compute center of mass and firs-order moments of events inside the global ROI
        std::tie(l, w, thetaRad) = ellipseParam(m00, m10, m01, m11, m20,
                                                m02); // function to compute "global" ellipse parameters

        COM_prec.x = COM.x;
        COM_prec.y = COM.y;

        COM.x = m10;
        COM.y = m01;

        std = compute_std(qROI, COM);

        std::tie(x_var, y_var) = compute_stdev(qROI, COM);

        if (start == true) {
            prev_t = yarp::os::Time::now();
            start = false;
        }

        if (COM_prec.x != COM.x && COM_prec.y != COM.y){
            int sigma=5;
            widthROI = sigma * x_var;
            heightROI = sigma * y_var;
        }

        leftRect = COM.x - widthROI / 2;
        rightRect = COM.x + widthROI / 2;
        bottomRect = COM.y - heightROI / 2;
        topRect = COM.y + heightROI / 2;

//          cout << "COM: "<<COM.x <<", "<<COM.y<<endl;
//          cout << "rect: "<<leftRect <<" "<< rightRect <<" "<< bottomRect<< " "<< topRect << endl;

        x_insideROI.clear();
        y_insideROI.clear();

        for (auto i = 0; i < m00; i++) {
            if (qROI.q[i].x > leftRect && qROI.q[i].x < rightRect && qROI.q[i].y > bottomRect &&
                qROI.q[i].y < topRect) {
                n_events_insideROI += 1;
                x_insideROI.push_back(qROI.q[i].x);
                y_insideROI.push_back(qROI.q[i].y);
            }
        }

        currentROI=qROI.getROI();

        cout << "-----ROI-----"<<currentROI[0]<< ", "<<currentROI[1]<< ", "<<currentROI[2]<< ", "<<currentROI[3]<<endl;

        if (x_insideROI.size() != 0) {
            COM_insideROI.x = accumulate(x_insideROI.begin(), x_insideROI.end(), 0) / (x_insideROI.size());
            COM_insideROI.y = accumulate(y_insideROI.begin(), y_insideROI.end(), 0) / (y_insideROI.size());
        }

//          cout <<"tracking: " <<tracking << endl;
//          cout <<"start: " <<start<<endl;

        if (start == false) {
            double curr_time = yarp::os::Time::now();
            cout << "Time passed: "<< curr_time - prev_t <<std::endl;
            if ((curr_time - prev_t) > reset_time) {
                resetTracker();
                yInfo() << "Tracker resetted after " << (curr_time - prev_t) << " seconds.";
            } else {
                std::cout<<"ACQUIRED EVENTS:  "<<n_events_acquired_insideROI<<std::endl;
                std::cout<<"EVENTS INSIDE RECT:  "<<n_events_insideROI<<std::endl;
                std::cout << "STD: "<<std<<endl;
                if (n_events_insideROI > numEventsAccepted && n_events_acquired_insideROI > numNewEvents && std<40) {

                    cout << "INSIDE-----------------"<<std::endl;
                    tracking = true;
                    ROI = qROI.setROI(leftRect, rightRect, bottomRect, topRect);
                    prev_t = yarp::os::Time::now();

                }
            }
        }

        m.unlock();
        Bottle &pos_Obj = posObj_port.prepare();
        pos_Obj.clear();

        if (tracking == true) {
            pos_Obj.addInt(COM.x);
            pos_Obj.addInt(COM.y);
            pos_Obj.addInt(1);
            cout << "tracked" << "(" << COM.x << " , " << COM.y << ")" << endl;
        } else {
            pos_Obj.addInt(0);
            pos_Obj.addInt(0);
            pos_Obj.addInt(0);
            cout << "NOT tracked" << endl;
        }

        posObj_port.write();

        if (visualization) {
            m.lock();
            yarp::sig::ImageOf<yarp::sig::PixelBgr> &display = image_out.prepare();
            display = trackMap;

            trackMap.zero();
            trackImg = cv::cvarrToMat((IplImage *) display.getIplImage());

            cv::applyColorMap(trackImg, trackImg, cv::COLORMAP_BONE);

            if (tracking){
                // puck
                cv::rectangle(trackImg, cv::Point(leftRect, bottomRect), cv::Point(rightRect, topRect),
                              cv::Scalar(0, 255, 0), 1, 8, 0); // print ROI box
                cout << "ELLIPSE: "<< l << " "<<w<<endl;
                cv::ellipse(trackImg, COM, cv::Size(x_var, y_var), 0, 0, 360, cv::Scalar(255, 0, 0), 2, 8, 0); // print ROI ellipse
                cv::circle(trackImg, COM, 1, cv::Scalar(0, 0, 255), -1, 8, 0);
            }

            // hand
//        qROI.setROI_hand(hand_roi[2], hand_roi[3],hand_roi[4], hand_roi[5]);
//        cv::circle(trackImg, cv::Point2d(hand_roi[0], hand_roi[1]), 3, cv::Scalar(255, 255, 0), -1, 8,
//                   0);
//        cv::rectangle(trackImg,
//                      cv::Point(hand_roi[2], hand_roi[4]),
//                      cv::Point(hand_roi[3], hand_roi[5]), cv::Scalar(255, 255, 0),
//                      1, 8, 0); // print ROI box
//
//        qROI.remove_handEvents();

            image_out.write();
            m.unlock();
        }

    }
}

void trackerModule::resetTracker() {

    qROI.clean();
    qROI.setSize(n_mass);
    ROI = qROI.setROI(0, res.width, 0, res.height);

    tracking = false;
    start = true;
}

//compute standard deviation
double trackerModule::compute_std(roiq qROI, cv::Point avg) {

    double diff = 0, sq_diff_sum = 0, std_dev = 0;

    for (auto i = 0; i < qROI.q.size(); i++) {
        diff = (qROI.q[i].x - avg.x) * (qROI.q[i].x - avg.x) + (qROI.q[i].y - avg.y) * (qROI.q[i].y - avg.y);
        sq_diff_sum += diff;
    }
    std_dev = std::sqrt(sq_diff_sum / qROI.q.size());

    return std_dev;
}

std::tuple<double, double> trackerModule::compute_stdev(roiq qROI, cv::Point avg) {

    double x_sum = 0, y_sum = 0;
    double x_variance=0, y_variance=0;

    for (auto i = 0; i < qROI.q.size(); i++) {

        yarp::sig::PixelBgr &ePix = trackMap.pixel(qROI.q[i].x,qROI.q[i].y);
        ePix.b = ePix.g = ePix.r = 255;

        x_sum += (qROI.q[i].x - avg.x) * (qROI.q[i].x - avg.x);
        y_sum += (qROI.q[i].y - avg.y) * (qROI.q[i].y - avg.y);
    }

    x_variance=sqrt(x_sum/(qROI.q.size()-1));
    y_variance=sqrt(y_sum/(qROI.q.size()-1));

    return std::make_tuple(x_variance,y_variance);
}

std::tuple<double, double, double, double, double> trackerModule::computeCOM(roiq qROI) {

    double m00 = qROI.q.size();
    int m10 = 0, m01 = 0;
    double m11 = 0, m20 = 0, m02 = 0;
    for (auto i = 0; i < m00; i++) {

        // compute raw moments of first order
        m10 += qROI.q[i].x; // sum of coord x of events within the ROI
        m01 += qROI.q[i].y; // sum of coord y of events within the ROI

        // compute raw moments of second order
        m11 += (qROI.q[i].x) * (qROI.q[i].y);
        m20 += pow(qROI.q[i].x, 2.0);
        m02 += pow(qROI.q[i].y, 2.0);

    }

    // compute the center of mass
    m10 = m10 / m00;
    m01 = m01 / m00;

    return std::make_tuple(m10, m01, m11, m20, m02);
}

std::tuple<double, double, double>
trackerModule::ellipseParam(double m00, double m10, double m01, double m11, double m20, double m02) {

    double a = 0, b = 0, c = 0;
    double thetaRad = 0, l = 0, w = 0;

    // compute central moments
    a = m20 / m00 - pow(m10, 2.0);
    b = 2*(m11 / m00 - m10 * m01);
    c = m02 / m00 - pow(m01, 2.0);

    cout << "central moments:"<<a<<", "<<b<<", "<<c<<endl;

    // compute orientation in radians
    thetaRad = 0.5 * atan2(b, a - c);

    // compute major and minor ellipse axes
    l = sqrt(6 * (a + c + sqrt(pow(b, 2.0) + pow((a - c), 2.0)))) / 2; //semi-major
    double calcolo = a + c - sqrt(pow(b, 2.0) + pow((a - c), 2.0));
    if (calcolo<0)
        w=0;
    else
        w = sqrt(6 * (a + c - sqrt(pow(b, 2.0) + pow((a - c), 2.0)))) / 2; //semi-minor

    return std::make_tuple(l, w, thetaRad);
}

void compute_vel (cv::Point2d actual_point, cv::Point2d previous_point, double actual_t, double previous_t){

    double dist = sqrt(pow((actual_point.x - previous_point.x),2) + pow((actual_point.y - previous_point.y),2));
    double vel = dist / (actual_t - previous_t);

    double slope = (actual_point.y - previous_point.y)/(actual_point.x - previous_point.x);

}

double trackerModule::getPeriod() {
    return 0.03; //30 Hz
}

bool trackerModule::updateModule() {

//    Bottle *bottle_hand_location = hand_location.read();
//    y_hand_position = bottle_hand_location->get(0).asDouble();
//
//    std::cout<<"Y HAND: "<<y_hand_position<<std::endl;

//    std::vector<double> hand_roi(6);
//    for (size_t i = 0; i < hand_roi.size(); i++) {
//        hand_roi[i] = interp[i]->operator()(y_hand_position);
//    }

    return Thread::isRunning();
}

bool trackerModule::interruptModule() {
    return Thread::stop();
}

void trackerModule::onStop() {
    input_port.close();
    output_port.close();
    posObj_port.close();
    image_out.close();
}

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
    rf.setDefault("table-file", "lut.tsv");
    rf.configure(argc, argv);

    /* create the module */
    trackerModule tracker;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return tracker.runModule(rf);
}
