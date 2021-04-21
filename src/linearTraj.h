//
// Created by root on 4/13/21.
//

#ifndef STUDY_AIR_HOCKEY_LINEARTRAJ_H
#define STUDY_AIR_HOCKEY_LINEARTRAJ_H


class linearTraj {

protected:
    double pos;          // current position
    double vel;
    double Ts;
    double epsilon=0.001;

public:

    void init(double velocity, double samplingTime){
        vel=velocity;
        Ts=samplingTime;
        pos=0;
    }


    virtual void computeCoeff(double yd){

        if(abs(pos-yd) > epsilon){
            if (pos<yd)
                pos = pos + vel*Ts;
            else
                pos = pos - vel*Ts;
        }

    }

    double getPos(){ return pos; }
};


#endif //STUDY_AIR_HOCKEY_LINEARTRAJ_H
