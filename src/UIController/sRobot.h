#ifndef SROBOT_H
#define SROBOT_H
#include "../CommonLibs/rtControllerModules/rtController_Common_Parameters.h"
#include <QObject>




class sRobot
{

public:
    sRobot();
    ~sRobot();

    //modifieurs
    void setCalStatus(bool new_state){bIsCal = new_state;}
    void setPowerStatus(bool new_state){bIsPowerOn = new_state;}
    void setSelectedJoint(int new_joint){selected_joint = new_joint;}
    void setJPos(double *);
    void setJPos(int id, double value){JPos[id] = value;}
    void setJVel(double *);
    void setForces(double *);
    void setForces(int id, double value){Forces[id] = value;}

    // accesseurs
    bool getCalStatus(void){return bIsCal;}
    bool getPowerStatus(void){return bIsPowerOn;}
    unsigned int getSelectedJoint(void){return selected_joint;}
    double *getJPos(void){return JPos;}
    double getJPos(int id){return JPos[id];}

    double *getJVel(void){return JVel;}
    double *getForces(void){return Forces;}


protected :
    int nb_joints;
    int nb_dof;
    int nb_forces;


    double *JPos;
    double *JVel;
    double *Forces;

    bool   bIsCal;             /// flag indiquant que le robot est étalonné
    bool   bIsPowerOn;         /// flag indiquant que le robot est sous tension


    // Fonctions membres protégées
    unsigned int selected_joint;        // permet de définir l'actionneur que l'on souhaite controler

};

#endif // SROBOT_H
