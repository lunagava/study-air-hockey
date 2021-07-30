//
// Created by root on 4/13/21.
//

#ifndef STUDY_AIR_HOCKEY_LINEARTRAJ_H
#define STUDY_AIR_HOCKEY_LINEARTRAJ_H


class linearTraj {

protected:
    double pos;          // current position
    double vel;
    double Ts; // must be at least equal to or lower than 10 ms
    double epsilon = 0.005; // 5 mm (1/2 cm)
    bool reached;

public:

    void init(double velocity, double samplingTime){
        vel=velocity;
        Ts=samplingTime;
        pos=0;
    }


    void computeCoeff(double yd){

        if(std::abs(pos-yd) > epsilon){
            if (pos<yd)
                pos = pos + vel*Ts;
            else
                pos = pos - vel*Ts;
        }

    }

    double getPos(){ return pos; }
};


#endif //STUDY_AIR_HOCKEY_LINEARTRAJ_H