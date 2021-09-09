#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <iostream>
#include <event-driven/all.h>
#include <opencv2/opencv.hpp>
#include <vector>
//#include <dirent.h>
//#include <sstream>
//#include <numeric>
//#include <iterator>


#define f 177.702
#define x0 155.757
//#define x0 152.228
#define y0 113.539

using namespace ev;
using namespace yarp::os;
using namespace yarp::sig;
using std::vector;
using std::string;


class flowPrediction : public RFModule, public Thread {

private:

    yarp::dev::PolyDriver drv_gaze;
    yarp::dev::IGazeControl* gaze;

    vReadPort< vector<AE> > input_port;
    yarp::os::BufferedPort<yarp::os::Bottle> eye_port, velPort, time_scope_port;

    double x_eye, y_eye, z_eye, xa_eye, ya_eye, za_eye, theta_eye;
    double x_eye_prev, y_eye_prev, z_eye_prev, t_eye_prev;
    double diff_x, diff_y, diff_z, vx, vy, vz, wx, wy, wz;
    yarp::sig::Matrix T_eye, T_eye_prev, R_eye, R_eye_prev, relative_orientation, ang_velocity_skew;
    double time_eye;
    double dT;
    double diff_roll, diff_pitch, diff_yaw;
    double t_prev;
    yarp::sig::Vector angles_eye, angles_eye_prev;
    double last_ts, last_ts_prev;
    double flow_x, flow_y;
    yarp::sig::Vector eye_pos, eye_orient;
    yarp::sig::Vector event_pix, event_root, event_camera;
    yarp::sig::Vector plane{0., 0., 1.0, 0.09};
    double whichImagePlane;

//    BufferedPort< Vector > scope_port;

//    madgwickFilter orientation_filter;
//    velocityEstimator vel_estimate;
    double period;
    bool dof6;

    resolution res;


public:

    flowPrediction()
    {

    }

    virtual bool configure(yarp::os::ResourceFinder& rf)
    {
        //set the module name used to name ports
        setName((rf.check("name", Value("/motion-prediction")).asString()).c_str());

        res.height = rf.check("height", Value(240)).asInt();
        res.width  = rf.check("width",  Value(304)).asInt();
        period = rf.check("period", Value(0.01)).asDouble();
        dof6 = rf.check("dof6", Value(true)).asBool();

        Network yarp;
        if(!yarp.checkNetwork(2.0)) {
            std::cout << "Could not connect to YARP" << std::endl;
            return false;
        }

        //open io ports
        if(!input_port.open(getName() + "/AE:i")) {
            yError() << "Could not open input port";
            return false;
        }

        if(!eye_port.open(getName() + "/eye-frame:i")) {
            yError() << "Could not open input port";
            return false;
        }

        if(!velPort.open(getName() + "/velocities:o")) {
            yError() << "Could not open scope out port";
            return false;
        }

        if(!time_scope_port.open(getName() + "/times:o")) {
            yError() << "Could not open scope out port";
            return false;
        }

//        yarp.connect("/study-air-hockey/eye",
//                     "/motion-prediction/eye-frame:i",
//                     "fast_tcp");

        yarp.connect("/vPreProcess/right:o",
                     "/motion-prediction/AE:i",
                     "fast_tcp");


        yarp::os::Property options_gaze;
        options_gaze.put("device", "gazecontrollerclient");
        options_gaze.put("remote", "/iKinGazeCtrl");
        options_gaze.put("local", getName()+"/gaze");
        if (!drv_gaze.open(options_gaze))
        {
            yInfo()<<"Unable to open the Gaze Controller";
            return false;
        }
        drv_gaze.view(gaze);

        T_eye = yarp::math::zeros(4,4);
        T_eye_prev = yarp::math::zeros(4,4);
        R_eye = yarp::math::zeros(3,3);
        R_eye_prev = yarp::math::zeros(3,3);
        relative_orientation = yarp::math::zeros(3,3);
        ang_velocity_skew = yarp::math::zeros(3,3);
        angles_eye.resize(3);
        angles_eye_prev.resize(3);
        eye_pos.resize(3);
        eye_orient.resize(4);
        event_pix.resize(2);
        event_root.resize(3); event_camera.resize(3);

        whichImagePlane=1;

        t_prev = yarp::os::Time::now();
        gaze->getRightEyePose(eye_pos, eye_orient);
        T_eye_prev = yarp::math::axis2dcm(eye_orient);
        R_eye_prev(0,0)=T_eye_prev(0,0); R_eye_prev(0,1)=T_eye_prev(0,1); R_eye_prev(0,2)=T_eye_prev(0,2);
        R_eye_prev(1,0)=T_eye_prev(1,0); R_eye_prev(1,1)=T_eye_prev(1,1); R_eye_prev(1,2)=T_eye_prev(1,2);
        R_eye_prev(2,0)=T_eye_prev(2,0); R_eye_prev(2,1)=T_eye_prev(2,1); R_eye_prev(2,2)=T_eye_prev(2,2);


        //start the asynchronous and synchronous threads
        return true;
    }

    virtual double getPeriod()
    {
        return period; //period of synchrnous thread
    }

    bool interruptModule()
    {
        //if the module is asked to stop ask the asynchrnous thread to stop
        return Thread::stop();
    }

    void onStop()
    {
        //when the asynchrnous thread is asked to stop, close ports and do
        //other clean up
        input_port.close();
        eye_port.close();
        velPort.close();
        time_scope_port.close();
        drv_gaze.close();
    }

    void getOpticalFlow(double x, double y, double d, vector<double> vel, double& flow_x, double& flow_y)
    {
        double f_inv = 1.0/f;
        flow_x = ((x - x0) * (y - y0) * f_inv)*vel[3] + -(f + (pow(x - x0, 2) * f_inv))*vel[4] + (y - y0)*vel[5];
        flow_y = (f + (pow(y - y0, 2) * f_inv))*vel[3] + (-(x - x0) * (y - y0) * f_inv)*vel[4] + -(x - x0)*vel[5];

        if (dof6){
            flow_x = flow_x + (-f/d)*vel[0] + (x-x0)/d*vel[2];
            flow_y = flow_y + (-f/d)*vel[1] + (y-y0)/d*vel[2];
        }

    }

    void visualizeColor(int x, int y, double stamp, double flow_x, double flow_y, int fstamp, cv::Mat debug_image){

        //                cv::Mat magnitude, angle, magn_norm;
        //                cv::cartToPolar(flow_x, flow_y, magnitude, angle, true);
        double magnitude = sqrt(flow_x*flow_x+flow_y*flow_y)/200.0 * 255.0;
        //yInfo()<<v.x<<v.y<<magnitude/255.0;
        if(magnitude > 255.0) magnitude = 255.0f;

        double angle = atan2(flow_y, flow_x);
        //                cv::normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
        angle = 180.0*(angle + M_PI)/(2.0*M_PI);

        int y_hat = y - flow_y*vtsHelper::deltaS(stamp, fstamp) + 0.5;
        int x_hat = x - flow_x*vtsHelper::deltaS(stamp, fstamp) + 0.5;
        if(y_hat > 0 && y_hat < debug_image.rows-1 && x_hat > 0 && x_hat < debug_image.cols)
            debug_image.at<cv::Vec3f>(y_hat, x_hat) = cv::Vec3f(angle, 255.0f, magnitude);
    }

    void visualizeLines(int x, int y, double flow_x, double flow_y, int ms, cv::Mat masker, cv::Mat arrow_image, double elapsed_time){

        int xl = std::max(x-ms, 0); int xh = std::min(x+ms, masker.cols);
        int yl = std::max(y-ms, 0); int yh = std::min(y+ms, masker.rows);

        masker(cv::Rect(cv::Point(xl, yl), cv::Point(xh, yh))).setTo(0);

        cv::line(arrow_image, cv::Point(x, y), cv::Point(x+flow_x*elapsed_time, y+flow_y*elapsed_time), CV_RGB(255,255,255));
        //                debug_image.at<cv::Vec3f>(v.y, v.x) = cv::Vec3f(180.0f, 255.0f, 255.0f);
    }

    void visualizeCompensatedEdges(int x, int y, double stamp, double first_ts, double flow_x, double flow_y, cv::Mat compensated_image, cv::Mat not_compensated_image, cv::Mat heat_map){//, cv::Mat time_map){

        double deltaT = ev::vtsHelper::deltaS(stamp, first_ts);
        //double deltaT = (stamp - stamp)*vtsHelper::tstosecs();

        double x_hat = x - flow_x*deltaT;
        double y_hat = y - flow_y*deltaT;

//        std::cout<<"x projection: "<<x<<" -> "<<x_hat<<", y projection: "<<y<<" -> "<<y_hat<<std::endl;
//        std::cout<<"  Flow_x = "<<flow_x<<",   Flow y = "<<flow_y<<"     Flow x * deltaT = "<<flow_x*deltaT<<"    Flow y * deltaT"<<flow_y*deltaT<<std::endl;
//        std::cout<<std::endl;

        if(y_hat > 0 && y_hat < compensated_image.rows-1 && x_hat > 0 && x_hat < compensated_image.cols-1){
            compensated_image.at<unsigned char>(y_hat, x_hat) = 255;
            heat_map.at<unsigned char>(y_hat, x_hat) += 1;
//            time_map.at<unsigned  char>(y_hat, x_hat) = stamp;
        }

        if(y > 0 && y < not_compensated_image.rows-1 && x > 0 && x < not_compensated_image.cols-1)
            not_compensated_image.at<unsigned char>(y, x) = 255;

//        std::cout<<"Reprojection: x="<<x_hat<<", y="<<y_hat<<", flow_x="<<flow_x<<", flow_y="<<flow_y<<" deltaT="<<deltaT<<std::endl;

    }

    //synchronous thread
    virtual bool updateModule()
    {

//        yarp::os::Bottle *eye_status = eye_port.read();
//        // position
//        x_eye = eye_status->get(0).asDouble();
//        y_eye = eye_status->get(1).asDouble();
//        z_eye = eye_status->get(2).asDouble();
//        // orientation in axis-angle representation
//        xa_eye = eye_status->get(3).asDouble();
//        ya_eye = eye_status->get(4).asDouble();
//        za_eye = eye_status->get(5).asDouble();
//        theta_eye = eye_status->get(6).asDouble();
//        time_eye = eye_status->get(7).asDouble();

            gaze->getRightEyePose(eye_pos, eye_orient);
        // position
            x_eye = eye_pos[0];
            y_eye = eye_pos[1];
            z_eye = eye_pos[2];
//        // orientation in axis-angle representation
            xa_eye = eye_orient[0];
            ya_eye = eye_orient[1];
            za_eye = eye_orient[2];
            theta_eye = eye_orient[3];
            time_eye = yarp::os::Time::now();

//        angles_eye = yarp::math::dcm2rpy(T_eye);

        diff_x = x_eye - x_eye_prev;
        diff_y = y_eye - y_eye_prev;
        diff_z = z_eye - z_eye_prev;
//        diff_yaw = angles_eye[0]-angles_eye_prev[0]; //z
//        diff_pitch = angles_eye[1]-angles_eye_prev[1]; //y
//        diff_roll = angles_eye[2]-angles_eye_prev[2]; //x
        dT = time_eye - t_eye_prev;

        T_eye = yarp::math::axis2dcm(eye_orient);
        R_eye(0,0)=T_eye(0,0); R_eye(0,1)=T_eye(0,1); R_eye(0,2)=T_eye(0,2);
        R_eye(1,0)=T_eye(1,0); R_eye(1,1)=T_eye(1,1); R_eye(1,2)=T_eye(1,2);
        R_eye(2,0)=T_eye(2,0); R_eye(2,1)=T_eye(2,1); R_eye(2,2)=T_eye(2,2);

        relative_orientation = R_eye*R_eye_prev.transposed();
        yarp::sig::Vector orient, axis;
        double angle;
        orient.resize(4);
        axis.resize(3);
        orient = yarp::math::dcm2axis(relative_orientation);
        axis[0]=orient[0]; axis[1]=orient[1]; axis[2]=orient[2];
        angle=orient[3];

        yarp::sig::Vector omega_root;
        omega_root.resize(3);
        omega_root[0] = (axis[0]*angle)/dT;
        omega_root[1] = (axis[1]*angle)/dT;
        omega_root[2] = (axis[2]*angle)/dT;

        yarp::sig::Vector omega_camera;
        omega_camera.resize(3);
        omega_camera = R_eye_prev.transposed()*omega_root;

//        double acos_argument = (relative_orientation(0,0)+relative_orientation(1,1)+relative_orientation(2,2)-1)/2;
//        if (acos_argument < -1.0)
//            acos_argument = -1.0;
//        else if (acos_argument > 1.0)
//            acos_argument = 1.0;

//        double theta = acos(acos_argument);
//        ang_velocity_skew = 1.0/ (2.0 * dT) * theta / sin(theta) * (relative_orientation*relative_orientation.transposed());

        vx = diff_x/dT;
        vy = diff_y/dT;
        vz = diff_z/dT;
        wx = omega_camera[0];
        wy = omega_camera[1];
        wz = omega_camera[2];

//        std::cout<<"vx="<<vx<<", vy="<<vy<<",vz="<<vz<<",wx="<<wx<<",wy="<<wy<<",wz="<<wz<<std::endl;

        x_eye_prev = x_eye;
        y_eye_prev = y_eye;
        z_eye_prev = z_eye;
        R_eye_prev = R_eye;
        t_eye_prev = time_eye;
//        angles_eye_prev[0]=angles_eye[0];
//        angles_eye_prev[1]=angles_eye[1];
//        angles_eye_prev[2]=angles_eye[2];

        yarp::os::Bottle &vel_bottle = velPort.prepare();
        vel_bottle.clear();
        vel_bottle.addDouble(vz);
        vel_bottle.addDouble(-vx);
        vel_bottle.addDouble(vy);
        vel_bottle.addDouble(wx);
        vel_bottle.addDouble(wy);
        vel_bottle.addDouble(wz);

        velPort.write();
//
//        std::cout<<std::endl;

        Stamp yarpstamp_flow;
        vector<double> vel_eye_frame;
        vel_eye_frame.push_back(vz);
        vel_eye_frame.push_back(-vx);
        vel_eye_frame.push_back(vy);
        vel_eye_frame.push_back(wx); //wz
        vel_eye_frame.push_back(wy); //-wx
        vel_eye_frame.push_back(wz); //wy

        double last_ts;
        double flow_x, flow_y;
        std::vector<cv::Point> velocity;
        static const int ms = 10;
        cv::Mat masker(res.height, res.width, CV_8U);
        masker.setTo(1);
        static cv::Mat debug_image(res.height, res.width, CV_32FC3);
        debug_image.setTo(cv::Scalar(0, 0, 0));
        cv::Mat arrow_image(res.height, res.width, CV_8UC3);
        arrow_image.setTo(0);
        cv::Mat compensated_image(res.height, res.width, CV_8U);
        compensated_image.setTo(0);
        cv::Mat not_compensated_image(res.height, res.width, CV_8U);
        not_compensated_image.setTo(0);
        cv::Mat heat_map(res.height, res.width, CV_8U);
        heat_map.setTo(0);
        cv::Mat time_map(res.height, res.width, CV_64F);
        time_map.setTo(0);
        //add any synchronous operations here, visualisation, debug out prints
        double elapsed_time = yarp::os::Time::now() - t_prev;
        t_prev = yarp::os::Time::now();
        int fstamp = -1;
        last_ts_prev = last_ts;
        int n_packets = input_port.queryunprocessed();
        for(auto i = 0; i < n_packets; i++) {

            const vector<AE> * q_flow = input_port.read(yarpstamp_flow);
            if(!q_flow || Thread::isStopping()) return false;

            for(auto &v : *q_flow) {

                if(fstamp < 0)fstamp = v.stamp;

//                event_pix[0]=v.x;
//                event_pix[1]=v.y;
//                gaze->get3DPointOnPlane(whichImagePlane, event_pix, plane, event_root);
//                std::cout<<event_root[0]<<" "<<event_root[1]<<" "<<event_root[2]<<std::endl;
                // useful to know the position wrt robot reference frame without knowing the plane on which the puck is moving
//                event_camera = R_eye*event_root;
                getOpticalFlow(v.x, v.y, 0.5, vel_eye_frame, flow_x, flow_y);
//                std::cout<<"flow x="<<flow_x<<", flow y="<<flow_y<<std::endl;
                visualizeCompensatedEdges(v.x, v.y, v.stamp, fstamp, flow_x, flow_y, compensated_image, not_compensated_image, heat_map);

                visualizeColor(v.x,v.y,v.stamp,flow_x,flow_y,fstamp,debug_image);
                if(masker.at<unsigned char>(v.y, v.x) == 0)
                    continue;

                visualizeLines(v.x,v.y,flow_x,flow_y,ms,masker,arrow_image, elapsed_time);

                last_ts = v.stamp;

            }
        }

//        heat_map=heat_map*20;
        const static int count_thresh = 1;
        cv::Mat temp1, temp2, temp3;
        cv::threshold(heat_map, temp1, count_thresh, 1, cv::THRESH_BINARY);
//        cv::threshold(heat_map, temp2, 0, 1, cv::THRESH_BINARY);
//        temp3 = 255*(temp2 - temp1);
//        heat_map.setTo(127);
//        heat_map -= temp1*127;
//        heat_map += temp3;
        heat_map = temp1 * 255;

         cv::Mat temp4;
//        int erosion_size = 0;
//        cv::Mat element = getStructuringElement( cv::MORPH_RECT,
//                                             cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
//                                             cv::Point( erosion_size, erosion_size ) );
//        cv::erode(temp3, temp4, cv::Mat());
//        heat_map = temp3;

//        cv::bitwise_and(compensated_image,heat_map, compensated_image);

        //make an image visualising output
        //build hsv image
        cv::Mat bgr, hsv8;
        debug_image.convertTo(hsv8, CV_8UC3);
        cv::cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR);

        arrow_image.copyTo(bgr, arrow_image);

        cv::namedWindow("imu prediction", cv::WINDOW_NORMAL);
        cv::imshow("imu prediction", bgr);

        cv::namedWindow("compensated edges", cv::WINDOW_NORMAL);
        cv::imshow("compensated edges", compensated_image);

        cv::namedWindow("not compensated edges", cv::WINDOW_NORMAL);
        cv::imshow("not compensated edges", not_compensated_image);

        cv::namedWindow("heat map", cv::WINDOW_NORMAL);
        cv::imshow("heat map", heat_map);

        cv::waitKey(3);

//        cv::Mat event_list;
//        cv::findNonZero(compensated_image, event_list);
//        static ev::AddressEvent v;
//        std::deque<ev::AddressEvent> events;
//        for (size_t i = 0; i < event_list.total(); i++) {
//            v.x = event_list.at<cv::Point>(i).x;
//            v.y = event_list.at<cv::Point>(i).y;
//            v.stamp = time_map.at<float>(v.x, v.y) * vtsHelper::vtsscaler; // To comply with ATIS timestamp
////            v.polarity = (ref_values.at<double>(v.y, v.x) > 0.0) ? 0 : 1;
//            events.push_back(v);
//
//            out_port
//
//        }

        return true;
    }

    //asynchronous thread run forever
    void run()
    {
        while (Thread::isRunning()) {


        }
    }

};

int main(int argc, char * argv[])
{

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( false );
    rf.setDefaultContext( "event-driven" );
//    rf.setDefaultConfigFile( "motion-prediction.ini" );
    rf.configure( argc, argv );

    /* create the module */
    flowPrediction instance;
    return instance.runModule(rf);
}
