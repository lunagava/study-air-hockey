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
    n_mass = rf.check("events", Value(100)).asInt();
    n_mass_max = rf.check("max_events", Value(400)).asInt();
    n_mass_min = rf.check("min_events", Value(20)).asInt();
    update_rate= rf.check("update_rate", Value(0.2)).asDouble();
    reset_time = rf.check("reset_time", Value(10)).asDouble();
    min_widthROI = rf.check("min_roi_width", Value(10)).asInt();
    min_heightROI = rf.check("min_roi_height", Value(10)).asInt();
    max_widthROI = rf.check("max_roi_width", Value(40)).asInt();
    max_heightROI = rf.check("max_roi_height", Value(40)).asInt();
    visualization = rf.check("visualization", Value(false)).asBool();
    table_file = rf.findFile("table-file");
    activation_thresh = rf.check("activation_thresh", Value(70)).asInt();

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

    handROI_width = 100;
    handROI_height = 150;

    ROI.resize(4);

    resetTracker();

    widthROI = min_widthROI;
    heightROI = min_heightROI;

    myfile.open("/code/luna/study-air-hockey/tracker/COM.csv");
    if (!myfile.is_open())
    {
        yError()<<"Could not open file for printing COMs";
        return -1;
    }

    return Thread::start();
}

//************************************************ RUN *****************************************************

void trackerModule::run() {

    double k = SeriesInverseError20thOrder(activation_thresh/100.0) * sqrt(2);
    yInfo() << "For activation threshold " << activation_thresh << " k is equal to " << k;

    while (Thread::isRunning()) {

//        m.lock();

        const vector<AE> *q = input_port.read(ystamp);
        if (!q || Thread::isStopping()) return;

//        mut.lock();

        unsigned int addEvents = 0;

//        std::cout<<"Events TO ACQUIRE: "<<int(update_rate*n_mass)<<std::endl;

        while(addEvents < int(update_rate*n_mass)) {

            // if we ran out of events -> get a new queue
            if(i >= q->size()) {
                i = 0;
                q = input_port.read(ystamp);
//                    std::cout<< q->size() <<std::endl;
                if(!q || Thread::isStopping()){
//                        mut.unlock();
                    return;
                }
            }
            yarp::sig::PixelBgr &ePix = trackMap.pixel((*q)[i].x,(*q)[i].y);
            ePix.b = ePix.g = ePix.r = 255;

            addEvents += qROI.add((*q)[i++]);

//            std::cout<<"EVENTS ADDED TO ROI: "<<addEvents<<std::endl;

        }

//        mut.unlock();

        qROI.setSize(n_mass);

        // initialization
//        m00 = qROI.q.size();

        // --------------------------------- GLOBAL ROI -----------------------------------
        std::tie(m10, m01, m11, m20, m02, COM_timestamp) = computeCOM(
                qROI); // function to compute center of mass and firs-order moments of events inside the global ROI
//        std::tie(l, w, thetaRad) = ellipseParam(m00, m10, m01, m11, m20,
//                                                m02); // function to compute "global" ellipse parameters

        COM_prec.x = COM.x;
        COM_prec.y = COM.y;

        COM.x = m10;
        COM.y = m01;

        Bottle &pos_Obj = posObj_port.prepare();
        pos_Obj.clear();

        if (!tracking) {

            std = compute_std(qROI, COM);

            if (k * std < 80) {
                tracking = true;
                prev_t = yarp::os::Time::now();
            } else{
                pos_Obj.addInt(0);
                pos_Obj.addInt(0);
                pos_Obj.addInt(0);
                pos_Obj.addDouble(0);
                pos_Obj.addDouble(yarp::os::Time::now());
//            cout << "NOT tracked" << endl;
            }
        }

        if (tracking){

            std::tie(x_dev, y_dev) = compute_stdev(qROI, COM);

            if (COM_prec.x != COM.x && COM_prec.y != COM.y){
                double factor = 1.5*1.8;
                widthROI = factor * x_dev;
                heightROI = factor * y_dev;
            }

            if (widthROI>max_widthROI)
                widthROI=max_widthROI;
            if (heightROI>max_heightROI)
                heightROI=max_heightROI;
            if (widthROI<min_widthROI)
                widthROI=min_widthROI;
            if (heightROI<min_heightROI)
                heightROI=min_heightROI;

            leftRect_next = COM.x - widthROI;
            rightRect_next = COM.x + widthROI;
            bottomRect_next = COM.y - heightROI;
            topRect_next = COM.y + heightROI;

            ROI = qROI.setROI(leftRect_next, rightRect_next, bottomRect_next, topRect_next);

            double coeff = res.height/(n_mass_max-n_mass_min);
            if (COM.y!=0)
                n_mass = int((1/coeff)*COM.y + coeff*n_mass_min);

//            std::cout<<"recalculated queue size: "<<n_mass<<std::endl;

            if (COM_history.size()>3){
                COM_history.push_front(COM);
                COM_history.pop_back();

                velocity = compute_vel(COM_history);
                nextROI_COM = cv::Point(COM.x + velocity.x, COM.y + velocity.y);

                left_ROI_predicted = nextROI_COM.x - widthROI;
                right_ROI_predicted = nextROI_COM.x + widthROI;
                bottom_ROI_predicted = nextROI_COM.y - heightROI;
                top_ROI_predicted = nextROI_COM.y + heightROI;

//                ROI = qROI.setROI(left_ROI_predicted, right_ROI_predicted, bottom_ROI_predicted, top_ROI_predicted);
            }
            else{
                COM_history.push_front(COM);
            }

            std::cout<<"m00: "<<qROI.q.size()<<std::endl;
            std::cout<< "actual COM: ("<<COM.x<<", "<<COM.y<<", "<<COM_timestamp<<")"<<std::endl;
            std::cout<< "VELOCITY: ("<<velocity.x<<", "<<velocity.y<<")"<<std::endl;
            std::cout<< "next COM: ("<<nextROI_COM.x<<", "<<nextROI_COM.y<<")"<<std::endl;

            prev_t = yarp::os::Time::now();

            pos_Obj.addInt(COM.x);
            pos_Obj.addInt(COM.y);
            pos_Obj.addInt(1);
            pos_Obj.addDouble(velocity.y);
//            cout << "tracked" << "(" << COM.x << " , " << COM.y << ")" << endl;

        }

        posObj_port.write();
//        m.unlock();
    }
}

void trackerModule::resetTracker() {

    n_mass = n_mass_max;
    qROI.setSize(0);
    ROI = qROI.setROI(0, res.width, 0, res.height);

    COM_history.clear();

    tracking = false;
}

double trackerModule::SeriesInverseError20thOrder(const double x){
    return 0.8862269254527579 * x + 0.23201366653465444 * pow(x,3) + 0.12755617530559793 * pow(x,5) + 0.08655212924154752 * pow(x,7) + 0.0649596177453854 * pow(x,9) + 0.051731281984616365 * pow(x,11) + 0.04283672065179733 * pow(x,13) + 0.03646592930853161 * pow(x,15) + 0.03168900502160544 * pow(x,17) + 0.027980632964995214 * pow(x,19) + 0.025022275841198347 * pow(x,21) + 0.02260986331889757 * pow(x,23) + 0.020606780379058987 * pow(x,25) + 0.01891821725077884 * pow(x,27) + 0.017476370562856534 * pow(x,29) + 0.01623150098768524 * pow(x,31) + 0.015146315063247798 * pow(x,33) + 0.014192316002509954 * pow(x,35) + 0.013347364197421288 * pow(x,37) + 0.012594004871332063 * pow(x,39) + 0.011918295936392034 * pow(x,41) + 0.011308970105922531 * pow(x,43) + 0.010756825303317953 * pow(x,45) + 0.010254274081853464 * pow(x,47) + 0.009795005770071162 * pow(x,49) + 0.00937372981918207 * pow(x,51) + 0.008985978502843365 * pow(x,53) + 0.008627953580709422 * pow(x,55) + 0.00829640592773923 * pow(x,57) + 0.007988540162603343 * pow(x,59) + 0.007701938432259749 * pow(x,61) + 0.00743449901783152 * pow(x,63) + 0.007184386511268305 * pow(x,65) + 0.006949991101064708 * pow(x,67) + 0.006729895085341522 * pow(x,69) + 0.006522845161450053 * pow(x,71) + 0.006327729364343136 * pow(x,73) + 0.00614355777038415 * pow(x,75) + 0.00596944626973445 * pow(x,77) + 0.005804602853834295 * pow(x,79) + 0.005648315975551554 * pow(x,81) + 0.0054999446262039555 * pow(x,83) + 0.00535890984168644 * pow(x,85) + 0.005224687403687484 * pow(x,87) + 0.005096801544706202 * pow(x,89) + 0.004974819499739037 * pow(x,91) + 0.004858346774958481 * pow(x,93) + 0.0047470230258849725 * pow(x,95) + 0.004640518455559021 * pow(x,97) + 0.004538530657907072 * pow(x,99) + 0.004440781843527172 * pow(x,101) + 0.004347016395021499 * pow(x,103) + 0.0042569987071827175 * pow(x,105) + 0.00417051127412616 * pow(x,107) + 0.004087352991109011 * pow(x,109) + 0.00400733764349813 * pow(x,111) + 0.003930292559306468 * pow(x,113) + 0.0038560574050482055 * pow(x,115) + 0.003784483107473815 * pow(x,117) + 0.003715430886126037 * pow(x,119) + 0.0036487713836790823 * pow(x,121) + 0.003584383882744558 * pow(x,123) + 0.003522155599297858 * pow(x,125) + 0.0034619810441376495 * pow(x,127) + 0.0034037614448720645 * pow(x,129) + 0.003347404221855558 * pow(x,131) + 0.0032928225123033104 * pow(x,133) + 0.0032399347375042247 * pow(x,135) + 0.0031886642086556213 * pow(x,137) + 0.0031389387673655678 * pow(x,139) + 0.0030906904573240333 * pow(x,141) + 0.00304385522404135 * pow(x,143) + 0.0029983726398996073 * pow(x,145) + 0.002954185652066844 * pow(x,147) + 0.002911240351090823 * pow(x,149) + 0.0028694857582238307 * pow(x,151) + 0.0028288736297367306 * pow(x,153) + 0.0027893582766628493 * pow(x,155) + 0.0027508963985735193 * pow(x,157) + 0.0027134469301298334 * pow(x,159) + 0.002676970899281695 * pow(x,161) + 0.0026414312960976946 * pow(x,163) + 0.0026067929513093027 * pow(x,165) + 0.0025730224237419342 * pow(x,167) + 0.002540087895884912 * pow(x,169) + 0.0025079590769233246 * pow(x,171) + 0.0024766071126183014 * pow(x,173) + 0.0024460045014790908 * pow(x,175) + 0.002416125016721371 * pow(x,177) + 0.0023869436335520523 * pow(x,179) + 0.0023584364613620555 * pow(x,181) + 0.0023305806804456365 * pow(x,183) + 0.0023033544828982926 * pow(x,185) + 0.0022767370173754973 * pow(x,187) + 0.002250708337421751 * pow(x,189) + 0.0022252493531041426 * pow(x,191) + 0.00220034178570695 * pow(x,193) + 0.0021759681252640946 * pow(x,195) + 0.0021521115907245244 * pow(x,197) + 0.002128756092562532 * pow(x,199) + 0.002105886197659956 * pow(x,201) + 0.002083487096301174 * pow(x,203) + 0.0020615445711343583 * pow(x,205) + 0.0020400449679639355 * pow(x,207) + 0.00201897516824968 * pow(x,209) + 0.00199832256319746 * pow(x,211) + 0.001978075029335388 * pow(x,213) + 0.0019582209054771716 * pow(x,215) + 0.0019387489709817671 * pow(x,217) + 0.0019196484252252159 * pow(x,219) + 0.0019009088682066624 * pow(x,221) + 0.0018825202822162794 * pow(x,223) + 0.0018644730144979976 * pow(x,225) + 0.001846757760844751 * pow(x,227) + 0.0018293655500683551 * pow(x,229) + 0.0018122877292901976 * pow(x,231) + 0.0017955159500026674 * pow(x,233) + 0.0017790421548547106 * pow(x,235) + 0.0017628585651180849 * pow(x,237) + 0.0017469576687938258 * pow(x,239) + 0.001731332209321178 * pow(x,241) + 0.0017159751748537467 * pow(x,243) + 0.001700879788069975 * pow(x,245) + 0.001686039496487191 * pow(x,247) + 0.001671447963250492 * pow(x,249) + 0.001657099058369582 * pow(x,251) + 0.001642986850378387 * pow(x,253) + 0.0016291055983939063 * pow(x,255) + 0.001615449744552216 * pow(x,257) + 0.0016020139068009065 * pow(x,259) + 0.0015887928720286174 * pow(x,261) + 0.0015757815895133644 * pow(x,263) + 0.0015629751646726348 * pow(x,265) + 0.0015503688530991604 * pow(x,267) + 0.0015379580548672853 * pow(x,269) + 0.0015257383090957375 * pow(x,271) + 0.0015137052887534616 * pow(x,273) + 0.0015018547956959605 * pow(x,275) + 0.0014901827559203006 * pow(x,277) + 0.001478685215027678 * pow(x,279) + 0.0014673583338830156 * pow(x,281) + 0.0014561983844617347 * pow(x,283) + 0.0014452017458743357 * pow(x,285) + 0.0014343649005600138 * pow(x,287) + 0.001423684430640985 * pow(x,289) + 0.001413157014429672 * pow(x,291) + 0.001402779423081366 * pow(x,293) + 0.0013925485173853214 * pow(x,295) + 0.0013824612446876837 * pow(x,297) + 0.0013725146359399852 * pow(x,299) + 0.0013627058028672803 * pow(x,301) + 0.0013530319352503075 * pow(x,303) + 0.0013434902983163707 * pow(x,305) + 0.0013340782302339149 * pow(x,307) + 0.001324793139706013 * pow(x,309) + 0.0013156325036582665 * pow(x,311) + 0.001306593865016819 * pow(x,313) + 0.001297674830572429 * pow(x,315) + 0.001288873068926738 * pow(x,317) + 0.0012801863085170834 * pow(x,319) + 0.001271612335716371 * pow(x,321) + 0.0012631489930047108 * pow(x,323) + 0.001254794177209685 * pow(x,325) + 0.001246545837812256 * pow(x,327) + 0.0012384019753154882 * pow(x,329) + 0.00123036063967339 * pow(x,331) + 0.0012224199287773018 * pow(x,333) + 0.0012145779869974047 * pow(x,335) + 0.001206833003777016 * pow(x,337) + 0.0011991832122774557 * pow(x,339) + 0.0011916268880714032 * pow(x,341) + 0.0011841623478826993 * pow(x,343) + 0.0011767879483707134 * pow(x,345) + 0.0011695020849574347 * pow(x,347) + 0.001162303190695553 * pow(x,349) + 0.001155189735175876 * pow(x,351) + 0.0011481602234724893 * pow(x,353) + 0.0011412131951241582 * pow(x,355) + 0.0011343472231505362 * pow(x,357) + 0.001127560913101779 * pow(x,359) + 0.0011208529021402775 * pow(x,361) + 0.0011142218581532365 * pow(x,363) + 0.0011076664788949055 * pow(x,365) + 0.0011011854911573118 * pow(x,367) + 0.0010947776499684008 * pow(x,369) + 0.001088441737816534 * pow(x,371) + 0.0010821765639003388 * pow(x,373) + 0.0010759809634029577 * pow(x,375) + 0.0010698537967897648 * pow(x,377) + 0.0010637939491286862 * pow(x,379) + 0.0010578003294322703 * pow(x,381) + 0.001051871870020703 * pow(x,383) + 0.0010460075259049976 * pow(x,385) + 0.0010402062741896442 * pow(x,387) + 0.001034467113493927 * pow(x,389) + 0.0010287890633913458 * pow(x,391) + 0.0010231711638664 * pow(x,393) + 0.0010176124747881504 * pow(x,395) + 0.0010121120753999447 * pow(x,397) + 0.0010066690638247397 * pow(x,399) + 0.001001282556585464 * pow(x,401) + 0.0009959516881398885 * pow(x,403) + 0.000990675610429507 * pow(x,405) + 0.000985453492441922 * pow(x,407) + 0.000980284519786282 * pow(x,409) + 0.0009751678942813104 * pow(x,411) + 0.000970102833555497 * pow(x,413) + 0.000965088570659034 * pow(x,415) + 0.0009601243536870979 * pow(x,417) + 0.0009552094454140942 * pow(x,419) + 0.0009503431229384911 * pow(x,421) + 0.0009455246773378917 * pow(x,423) + 0.0009407534133339981 * pow(x,425) + 0.0009360286489671448 * pow(x,427) + 0.0009313497152800757 * pow(x,429) + 0.0009267159560106687 * pow(x,431) + 0.0009221267272933102 * pow(x,433) + 0.000917581397368642 * pow(x,435) + 0.0009130793463013963 * pow(x,437) + 0.0009086199657060749 * pow(x,439) + 0.0009042026584802059 * pow(x,441) + 0.0008998268385449422 * pow(x,443) + 0.0008954919305927667 * pow(x,445) + 0.0008911973698420738 * pow(x,447) + 0.0008869426017984185 * pow(x,449) + 0.0008827270820222149 * pow(x,451) + 0.0008785502759026872 * pow(x,453) + 0.0008744116584378746 * pow(x,455) + 0.0008703107140205068 * pow(x,457) + 0.0008662469362295634 * pow(x,459) + 0.0008622198276273445 * pow(x,461) + 0.0008582288995618849 * pow(x,463) + 0.0008542736719745464 * pow(x,465) + 0.0008503536732126325 * pow(x,467) + 0.0008464684398468723 * pow(x,469) + 0.0008426175164936275 * pow(x,471) + 0.0008388004556416812 * pow(x,473) + 0.0008350168174834716 * pow(x,475) + 0.0008312661697506358 * pow(x,477) + 0.0008275480875537402 * pow(x,479) + 0.0008238621532260711 * pow(x,481) + 0.0008202079561713666 * pow(x,483) + 0.0008165850927153757 * pow(x,485) + 0.0008129931659611316 * pow(x,487) + 0.0008094317856478317 * pow(x,489) + 0.0008059005680132207 * pow(x,491) + 0.000802399135659376 * pow(x,493) + 0.0007989271174217976 * pow(x,495) + 0.0007954841482417054 * pow(x,497) + 0.0007920698690414611 * pow(x,499) + 0.0007886839266030129 * pow(x,501) + 0.0007853259734492912 * pow(x,503) + 0.0007819956677284643 * pow(x,505) + 0.0007786926731009741 * pow(x,507) + 0.0007754166586292808 * pow(x,509) + 0.0007721672986702337 * pow(x,511) + 0.0007689442727700021 * pow(x,513) + 0.0007657472655614756 * pow(x,515) + 0.000762575966664118 * pow(x,517) + 0.0007594300705861389 * pow(x,519) + 0.000756309276628966 * pow(x,521) + 0.0007532132887939458 * pow(x,523) + 0.000750141815691205 * pow(x,525) + 0.0007470945704506218 * pow(x,527) + 0.0007440712706348473 * pow(x,529) + 0.0007410716381543222 * pow(x,531) + 0.0007380953991842377 * pow(x,533) + 0.0007351422840833848 * pow(x,535) + 0.0007322120273148497 * pow(x,537) + 0.000729304367368497 * pow(x,539) + 0.0007264190466852019 * pow(x,541) + 0.0007235558115827809 * pow(x,543) + 0.0007207144121835784 * pow(x,545) + 0.000717894602343673 * pow(x,547) + 0.0007150961395836398 * pow(x,549) + 0.0007123187850208579 * pow(x,551) + 0.0007095623033033046 * pow(x,553) + 0.0007068264625448016 * pow(x,555) + 0.0007041110342616792 * pow(x,557) + 0.0007014157933108204 * pow(x,559) + 0.0006987405178290501 * pow(x,561) + 0.0006960849891738388 * pow(x,563) + 0.0006934489918652792 * pow(x,565) + 0.0006908323135293152 * pow(x,567) + 0.0006882347448421842 * pow(x,569) + 0.000685656079476044 * pow(x,571) + 0.0006830961140457571 * pow(x,573) + 0.0006805546480568029 * pow(x,575) + 0.0006780314838542896 * pow(x,577) + 0.0006755264265730298 * pow(x,579) + 0.0006730392840886854 * pow(x,581) + 0.0006705698669699095 * pow(x,583) + 0.000668117988431498 * pow(x,585) + 0.0006656834642885094 * pow(x,587) + 0.0006632661129113348 * pow(x,589) + 0.000660865755181693 * pow(x,591) + 0.0006584822144495324 * pow(x,593) + 0.0006561153164908089 * pow(x,595) + 0.0006537648894661344 * pow(x,597) + 0.0006514307638802585 * pow(x,599) + 0.0006491127725423755 * pow(x,601) + 0.0006468107505272315 * pow(x,603) + 0.0006445245351370161 * pow(x,605) + 0.000642253965864018 * pow(x,607) + 0.0006399988843540261 * pow(x,609) + 0.0006377591343704657 * pow(x,611) + 0.0006355345617592385 * pow(x,613) + 0.0006333250144142678 * pow(x,615) + 0.0006311303422437181 * pow(x,617) + 0.000628950397136883 * pow(x,619) + 0.0006267850329317199 * pow(x,621) + 0.0006246341053830207 * pow(x,623) + 0.0006224974721312038 * pow(x,625) + 0.0006203749926717106 * pow(x,627) + 0.0006182665283249953 * pow(x,629) + 0.0006161719422070947 * pow(x,631) + 0.0006140910992007621 * pow(x,633) + 0.0006120238659271596 * pow(x,635) + 0.0006099701107180854 * pow(x,637) + 0.0006079297035887355 * pow(x,639) + 0.0006059025162109815 * pow(x,641) + 0.0006038884218871508 * pow(x,643) + 0.0006018872955243106 * pow(x,645) + 0.0005998990136090273 * pow(x,647) + 0.0005979234541826043 * pow(x,649) + 0.000595960496816781 * pow(x,651) + 0.0005940100225898867 * pow(x,653) + 0.0005920719140634371 * pow(x,655) + 0.000590146055259164 * pow(x,657) + 0.0005882323316364715 * pow(x,659) + 0.0005863306300703043 * pow(x,661) + 0.000584440838829427 * pow(x,663) + 0.0005825628475550968 * pow(x,665) + 0.0005806965472401283 * pow(x,667) + 0.0005788418302083395 * pow(x,669) + 0.0005769985900943688 * pow(x,671) + 0.0005751667218238599 * pow(x,673) + 0.0005733461215939995 * pow(x,675) + 0.0005715366868544139 * pow(x,677) + 0.0005697383162883983 * pow(x,679) + 0.0005679509097944887 * pow(x,681) + 0.0005661743684683589 * pow(x,683) + 0.0005644085945850407 * pow(x,685) + 0.0005626534915814589 * pow(x,687) + 0.0005609089640392747 * pow(x,689) + 0.0005591749176680341 * pow(x,691) + 0.0005574512592886066 * pow(x,693) + 0.0005557378968169206 * pow(x,695) + 0.0005540347392479788 * pow(x,697) + 0.0005523416966401541 * pow(x,699) + 0.0005506586800997567 * pow(x,701) + 0.0005489856017658699 * pow(x,703) + 0.0005473223747954491 * pow(x,705) + 0.0005456689133486695 * pow(x,707) + 0.0005440251325745404 * pow(x,709) + 0.0005423909485967517 * pow(x,711) + 0.0005407662784997686 * pow(x,713) + 0.0005391510403151626 * pow(x,715) + 0.0005375451530081738 * pow(x,717) + 0.0005359485364645014 * pow(x,719) + 0.0005343611114773172 * pow(x,721) + 0.0005327827997344965 * pow(x,723) + 0.0005312135238060668 * pow(x,725) + 0.0005296532071318631 * pow(x,727) + 0.0005281017740093911 * pow(x,729) + 0.0005265591495818922 * pow(x,731) + 0.0005250252598266081 * pow(x,733) + 0.0005235000315432365 * pow(x,735) + 0.0005219833923425821 * pow(x,737) + 0.0005204752706353906 * pow(x,739) + 0.0005189755956213683 * pow(x,741) + 0.0005174842972783808 * pow(x,743) + 0.0005160013063518285 * pow(x,745) + 0.000514526554344197 * pow(x,747) + 0.0005130599735047725 * pow(x,749) + 0.0005116014968195304 * pow(x,751) + 0.000510151058001182 * pow(x,753) + 0.0005087085914793848 * pow(x,755) + 0.0005072740323911094 * pow(x,757) + 0.0005058473165711607 * pow(x,759) + 0.0005044283805428505 * pow(x,761) + 0.0005030171615088202 * pow(x,763) + 0.000501613597342008 * pow(x,765) + 0.0005002176265767609 * pow(x,767) + 0.0004988291884000804 * pow(x,769) + 0.0004974482226430367 * pow(x,771) + 0.0004960746697722584 * pow(x,773) + 0.0004947084708816159 * pow(x,775) + 0.0004933495676840046 * pow(x,777) + 0.0004919979025032637 * pow(x,779) + 0.0004906534182662187 * pow(x,781) + 0.0004893160584948506 * pow(x,783) + 0.0004879857672985841 * pow(x,785) + 0.0004866624893666964 * pow(x,787) + 0.00048534616996084287 * pow(x,789) + 0.00048403675490769747 * pow(x,791) + 0.00048273419059170677 * pow(x,793) + 0.0004814384239479536 * pow(x,795) + 0.00048014940245513224 * pow(x,797) + 0.00047886707412862724 * pow(x,799) + 0.0004775913875137012 * pow(x,801) + 0.0004763222916787815 * pow(x,803) + 0.0004750597362088536 * pow(x,805) + 0.00047380367119894946 * pow(x,807) + 0.00047255404724773646 * pow(x,809) + 0.00047131081545120245 * pow(x,811) + 0.00047007392739643384 * pow(x,813) + 0.0004688433351554884 * pow(x,815) + 0.0004676189912793587 * pow(x,817) + 0.0004664008487920247 * pow(x,819) + 0.00046518886118459483 * pow(x,821) + 0.00046398298240953506 * pow(x,823) + 0.000462783166874981 * pow(x,825) + 0.0004615893694391351 * pow(x,827) + 0.0004604015454047457 * pow(x,829) + 0.0004592196505136668 * pow(x,831) + 0.00045804364094149806 * pow(x,833) + 0.00045687347329230173 * pow(x,835) + 0.00045570910459339807 * pow(x,837) + 0.00045455049229023415 * pow(x,839) + 0.0004533975942413297 * pow(x,841) + 0.0004522503687132923 * pow(x,843) + 0.00045110877437590826 * pow(x,845) + 0.00044997277029730157 * pow(x,847) + 0.00044884231593916206 * pow(x,849) + 0.00044771737115204495 * pow(x,851) + 0.0004465978961707327 * pow(x,853) + 0.00044548385160966656 * pow(x,855) + 0.00044437519845844244 * pow(x,857) + 0.0004432718980773684 * pow(x,859) + 0.00044217391219308845 * pow(x,861) + 0.0004410812028942648 * pow(x,863) + 0.00043999373262732227 * pow(x,865) + 0.0004389114641922551 * pow(x,867) + 0.00043783436073848575 * pow(x,869) + 0.00043676238576078816 * pow(x,871) + 0.0004356955030952647 * pow(x,873) + 0.0004346336769153805 * pow(x,875) + 0.00043357687172805104 * pow(x,877) + 0.00043252505236978566 * pow(x,879) + 0.00043147818400288325 * pow(x,881) + 0.00043043623211168087 * pow(x,883) + 0.0004293991624988544 * pow(x,885) + 0.00042836694128176836 * pow(x,887) + 0.0004273395348888782 * pow(x,889) + 0.0004263169100561787 * pow(x,891) + 0.0004252990338237038 * pow(x,893) + 0.000424285873532072 * pow(x,895) + 0.00042327739681907923 * pow(x,897) + 0.00042227357161633784 * pow(x,899) + 0.0004212743661459616 * pow(x,901) + 0.0004202797489172942 * pow(x,903) + 0.0004192896887236829 * pow(x,905) + 0.00041830415463929425 * pow(x,907) + 0.0004173231160159738 * pow(x,909) + 0.00041634654248014674 * pow(x,911) + 0.0004153744039297606 * pow(x,913) + 0.0004144066705312683 * pow(x,915) + 0.0004134433127166504 * pow(x,917) + 0.00041248430118047836 * pow(x,919) + 0.0004115296068770157 * pow(x,921) + 0.00041057920101735717 * pow(x,923) + 0.00040963305506660667 * pow(x,925) + 0.00040869114074109053 * pow(x,927) + 0.000407753430005609 * pow(x,929) + 0.0004068198950707221 * pow(x,931) + 0.0004058905083900728 * pow(x,933) + 0.000404965242657743 * pow(x,935) + 0.000404044070805645 * pow(x,937) + 0.0004031269660009469 * pow(x,939) + 0.0004022139016435305 * pow(x,941) + 0.00040130485136348276 * pow(x,943) + 0.0004003997890186193 * pow(x,945) + 0.00039949868869203973 * pow(x,947) + 0.00039860152468971377 * pow(x,949) + 0.0003977082715380997 * pow(x,951) + 0.00039681890398179167 * pow(x,953) + 0.0003959333969811975 * pow(x,955) + 0.0003950517257102469 * pow(x,957) + 0.00039417386555412744 * pow(x,959) + 0.00039329979210705026 * pow(x,961) + 0.0003924294811700424 * pow(x,963) + 0.00039156290874877006 * pow(x,965) + 0.00039070005105138535 * pow(x,967) + 0.00038984088448640336 * pow(x,969) + 0.00038898538566060404 * pow(x,971) + 0.0003881335313769605 * pow(x,973) + 0.00038728529863259393 * pow(x,975) + 0.0003864406646167538 * pow(x,977) + 0.0003855996067088216 * pow(x,979) + 0.0003847621024763423 * pow(x,981) + 0.00038392812967307774 * pow(x,983) + 0.0003830976662370843 * pow(x,985) + 0.00038227069028881605 * pow(x,987) + 0.0003814471801292491 * pow(x,989) + 0.0003806271142380302 * pow(x,991) + 0.00037981047127164726 * pow(x,993) + 0.0003789972300616248 * pow(x,995) + 0.00037818736961273613 * pow(x,997) + 0.00037738086910124255 * pow(x,999);
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
    double x_stdev=0, y_stdev=0;

    for (auto i = 0; i < qROI.q.size(); i++) {
        x_sum += (qROI.q[i].x - avg.x) * (qROI.q[i].x - avg.x);
        y_sum += (qROI.q[i].y - avg.y) * (qROI.q[i].y - avg.y);
    }

    x_stdev=sqrt(x_sum/(qROI.q.size()-1));
    y_stdev=sqrt(y_sum/(qROI.q.size()-1));

    return std::make_tuple(x_stdev,y_stdev);
}

std::tuple<double, double, double, double, double, double> trackerModule::computeCOM(roiq qROI) {

    double m00 = qROI.q.size();
    int m10 = 0, m01 = 0;
    double m11 = 0, m20 = 0, m02 = 0;
    double time_sum = 0, mean_time = 0;
    for (auto i = 0; i < m00; i++) {

//        yarp::sig::PixelBgr &ePix = trackMap.pixel(qROI.q[i].x,qROI.q[i].y);
//        ePix.b = ePix.g = ePix.r = 255;

        // compute raw moments of first order
        m10 += qROI.q[i].x; // sum of coord x of events within the ROI
        m01 += qROI.q[i].y; // sum of coord y of events within the ROI

        time_sum += qROI.q[i].stamp*vtsHelper::tsscaler*1000; //ms

        // compute raw moments of second order
        m11 += (qROI.q[i].x) * (qROI.q[i].y);
        m20 += pow(qROI.q[i].x, 2.0);
        m02 += pow(qROI.q[i].y, 2.0);

    }

    std::cout<<"Time sum: "<<time_sum<<std::endl;

    mean_time = time_sum/m00;

    // compute the center of mass
    m10 = m10 / m00;
    m01 = m01 / m00;

    return std::make_tuple(m10, m01, m11, m20, m02, mean_time);
}

std::tuple<double, double, double>
trackerModule::ellipseParam(double m00, double m10, double m01, double m11, double m20, double m02) {

    double a = 0, b = 0, c = 0;
    double thetaRad = 0, l = 0, w = 0;

    // compute central moments
    a = m20 / m00 - pow(m10, 2.0);
    b = 2*(m11 / m00 - m10 * m01);
    c = m02 / m00 - pow(m01, 2.0);

//    cout << "central moments:"<<a<<", "<<b<<", "<<c<<endl;

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

std::tuple<double, double> trackerModule::leastSquare(std::deque<cv::Point2d> points){

    std::vector<double> xy;
    std::vector<double> x_sqrd;
    std::vector<double> y_sqrd;

    double n = points.size();
    double sum_x = 0;
    double sum_y = 0;
    double sum_xy = 0;
    double sum_x_sqrd = 0;
    double sum_y_sqrd = 0;

    for (auto point : points)
    {
        sum_x += point.x;
        sum_y += point.y;
        xy.push_back( point.x * point.y );
        x_sqrd.push_back( pow(point.x, 2) );
        y_sqrd.push_back( pow(point.y, 2) );
    }

    for (auto val : xy)
    {
        sum_xy += val;
    }

    for (auto val : x_sqrd)
    {
        sum_x_sqrd += val;
    }

    for (auto val : y_sqrd)
    {
        sum_y_sqrd += val;
    }

    double m;
    double q;

    m = ( (n * sum_xy) - (sum_x * sum_y) ) / ( (n * sum_x_sqrd) - pow(sum_x, 2) );
    q = ( (sum_y * sum_x_sqrd) - (sum_x * sum_xy) ) / ( (n * sum_x_sqrd) - pow(sum_x, 2) );

    return std::make_tuple(m, q);
}

cv::Point2d trackerModule::compute_vel(std::deque<cv::Point2d> points)
{
    double m, q;
    cv::Point2d vel;

    std::tie(m, q) = leastSquare(points);

    cv::Point2d initPos, finalPos;

    initPos.y = points.back().y;
    initPos.x = ( initPos.y - q ) / m;

    //std::cout << initPos.x << " " << initPos.y << std::endl;

    finalPos.y = points.front().y;
    finalPos.x = ( finalPos.y - q) / m;

    //std::cout << finalPos.x << " " << finalPos.y << std::endl;

    vel.x = (finalPos.x - initPos.x) / double(points.size()-1);
    vel.y = (finalPos.y - initPos.y) / double(points.size()-1);

    return vel;
}

double trackerModule::getPeriod() {
    return 0.033; //30 Hz
}

bool trackerModule::updateModule() {

//    std::cout << "--------------------- UPDATE ------------------------"<<std::endl;

    posObj_port.write();

    if (tracking)
    {
        double curr_time = yarp::os::Time::now();
//            cout << "Time passed: "<< curr_time - prev_t <<std::endl;

        if ((curr_time - prev_t) > reset_time) {
            yInfo() << "Tracker resetted after " << (curr_time - prev_t)
                    << " seconds.--------------------------------------------------";
            resetTracker();
        }

        myfile << COM.x << ","<< COM.y << ","<< COM_timestamp <<","<<std::endl;
    }

    Bottle *bottle_hand_location = hand_location.read();
    y_hand_position = bottle_hand_location->get(0).asDouble();
    x_hand_pixel = bottle_hand_location->get(1).asDouble();
    y_hand_pixel = bottle_hand_location->get(2).asDouble();

    std::cout<<"u HAND: "<<x_hand_pixel<<std::endl;
    std::cout<<"v HAND: "<<y_hand_pixel<<std::endl;
    std::cout<<"Y CARTESIAN HAND: "<<y_hand_position<<std::endl;

    std::vector<double> hand_roi(6);
    for (size_t i = 0; i < hand_roi.size(); i++) {
        hand_roi[i] = interp[i]->operator()(y_hand_position);
    }

    qROI.setROI_hand(hand_roi[2], hand_roi[3],hand_roi[4], hand_roi[5]);
    qROI.remove_handEvents();

    if (visualization) {
//        m.lock();
        yarp::sig::ImageOf<yarp::sig::PixelBgr> &display = image_out.prepare();
        display = trackMap;

        trackMap.zero();
        trackImg = cv::cvarrToMat((IplImage *) display.getIplImage());

        cv::applyColorMap(trackImg, trackImg, cv::COLORMAP_BONE);

        if (tracking){
            // puck
            cv::rectangle(trackImg, cv::Point(leftRect_next, bottomRect_next), cv::Point(rightRect_next, topRect_next),
                          cv::Scalar(0, 255, 0), 1, 8, 0); // print ROI box
            cv::rectangle(trackImg, cv::Point(left_ROI_predicted, bottom_ROI_predicted), cv::Point(right_ROI_predicted, top_ROI_predicted),
                          cv::Scalar(229, 132, 249), 1, 8, 0); // print ROI box
            cv::circle(trackImg, nextROI_COM, 1, cv::Scalar(229, 132, 249), -1, 8, 0);
//            cout << "ELLIPSE: "<< l << " "<<w<<endl;
            cv::ellipse(trackImg, COM, cv::Size(1.8*x_dev, 1.8*y_dev), 0, 0, 360, cv::Scalar(0, 255,
                                                                                             255), 1, 8, 0); // print ROI ellipse
            cv::circle(trackImg, COM, 1, cv::Scalar(0, 255, 0), -1, 8, 0);
        }

        // hand
        cv::circle(trackImg, cv::Point2d(hand_roi[0], hand_roi[1]), 3, cv::Scalar(255, 255, 0), -1, 8,
                   0);
        cv::rectangle(trackImg,
                      cv::Point(hand_roi[2], hand_roi[4]),
                      cv::Point(hand_roi[3], hand_roi[5]), cv::Scalar(255, 255, 0),
                      1, 8, 0); // print ROI box

        image_out.write();
//        m.unlock();
    }

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

    myfile.close();
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