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

    yarp::sig::Matrix frequencyEvents;
    int th;
    Stamp ystamp;

    std::string table_file;
    typedef std::tuple<int, int> Entry;
    std::vector<Entry> table;
    bool first_run;

    vReadPort<vector<AE> > input_port;

    bool configure(yarp::os::ResourceFinder &rf) {

        // options and parameters
        th = rf.check("threshold", Value(100)).asInt();
        table_file = rf.getHomeContextPath() + "/" +
                     rf.check("table-file", yarp::os::Value("egoMotion.tsv")).asString();

        // module name and control
        setName((rf.check("name", Value("/egoMotion")).asString()).c_str());

        if (!input_port.open(getName() + "/AE:i"))
            return false;

        frequencyEvents = yarp::math::zeros(304, 240);
        first_run = false;

        return Thread::start();
    }

    auto writeTable() {

        std::ofstream fout(table_file);
        if (fout.is_open()) {
            for (auto entry = table.begin(); entry != table.end(); entry++) {
                const auto& x = std::get<0>(*entry);
                const auto& y = std::get<1>(*entry);
                fout << x << "\t"
                     << y;
                if (entry != table.end() - 1) {
                    fout << std::endl;
                }
            }
            fout.close();
            yInfo() << "Table written to" << table_file;
            return true;
        } else {
            yError() << "Unable to write to file" << table_file;
            return false;
        }
    }

//************************************************ RUN *****************************************************

    void run() {

        while (Thread::isRunning()) {

            const vector<AE> *q = input_port.read(ystamp);
            if (!q || Thread::isStopping()) return;

            int qs = input_port.queryunprocessed() - 1; // how many packets we have processed yet
            if (qs < 1) qs = 1; // if we do not have any queues
            if (qs > 4) yWarning() << qs;

            for (int i = 0; i < qs; i++) {
                for (auto &v : *q) {
                    frequencyEvents(v.x, v.y) += 1;
                }
            }

            first_run = true;

        }

    }

    double getPeriod() {
        return 1.0;
    }

    bool updateModule() {

        if (first_run){
            table.clear();
            for (auto r=0; r<frequencyEvents.rows(); r++){
                for (auto c=0; c<frequencyEvents.cols(); c++){
//                    std::cout<<frequencyEvents(r,c)<<" ";
                    if (frequencyEvents(r,c)>th && c<185) { //store the most fired events positions
//                    std::cout<<"position "<<r<<" "<<c<<std::endl;
                        table.push_back(std::make_tuple(r,c));
                    }
                }
//                std::cout<<std::endl;
            }
        }

        return Thread::isRunning();
    }

    bool close() {

        writeTable();

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