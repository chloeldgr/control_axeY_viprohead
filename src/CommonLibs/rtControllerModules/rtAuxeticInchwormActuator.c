#include "rtAuxeticInchwormActuator.h"


AUXETIC_INCHWORM_ACTUATOR_STRUCT actuator;

unsigned int rtAuxeticActuator_initParam(unsigned int sampling_period,
                                         unsigned long long init_phase_time,
                                         unsigned long long sequence_period,
                                         unsigned long long gripper_period)
{
    actuator.sampling_period    = sampling_period;
    actuator.direction          = 1;
    actuator.init_phase_time    = init_phase_time;
    actuator.phase              = INCHWORM_PHASE_INIT;
    actuator.period             = sequence_period;
    actuator.gripper_time       = gripper_period;//FG.close_time;
    actuator.actuation_time     = (actuator.period -(4*actuator.gripper_time))/2;
    actuator.control            = 0;
    // Code en dur les temps des différentes phases
    actuator.phase_time[0]  = actuator.gripper_time;//init_phase_time;
    actuator.phase_time[1]  = actuator.gripper_time;//FG.open_time;
    actuator.phase_time[2]  = actuator.actuation_time;//AA.actuation_time;
    actuator.phase_time[3]  = actuator.gripper_time;//FG.close_time;
    actuator.phase_time[4]  = actuator.gripper_time;//MG.open_time;
    actuator.phase_time[5]  = actuator.actuation_time;//AA.actuation_time;
    actuator.phase_time[6]  = actuator.gripper_time;//MG.close_time;
    actuator.current_time = 0;
    actuator.digital_output = 7;

    return actuator.digital_output;

}



///
/// \brief rtAuxeticActuator_updateParam
/// \param sequence_period
/// \param gripper_period
/// \return digital output  bit 0 : S1(EV1
///                         bit 1 : S2(EV3
///                         bit 2 : S3(EV2
///                         bit 3 : S4(EV4
///
unsigned int rtAuxeticActuator_updateParam( int direction,
                                            unsigned long long sequence_period,
                                            unsigned long long gripper_period)
{

    actuator.direction          = direction;
    actuator.period             = sequence_period;
    actuator.gripper_time       = gripper_period;//FG.close_time;
    actuator.actuation_time     = (actuator.period -(4*actuator.gripper_time))/2;
    // Code en dur les temps des différentes phases
    actuator.phase_time[INCHWORM_PHASE_INIT]            = actuator.gripper_time;//init_phase_time;
    actuator.phase_time[INCHWORM_PHASE_OPEN_FG]         = actuator.gripper_time;//FG.open_time;
    actuator.phase_time[INCHWORM_PHASE_AA_ELONGATION]   = actuator.actuation_time;//AA.actuation_time;
    actuator.phase_time[INCHWORM_PHASE_CLOSE_FG]        = actuator.gripper_time;//FG.close_time;
    actuator.phase_time[INCHWORM_PHASE_OPEN_MG]         = actuator.gripper_time;//MG.open_time;
    actuator.phase_time[INCHWORM_PHASE_AA_RETRACTION]   = actuator.actuation_time;//AA.actuation_time;
    actuator.phase_time[INCHWORM_PHASE_CLOSE_MG]        = actuator.gripper_time;//MG.close_time;

    actuator.current_time += actuator.sampling_period;
    if ( direction == 0)
    {
        actuator.digital_output=0;
    }
    else
    {
        switch(actuator.phase)
        {
        case INCHWORM_PHASE_INIT: //E0
            if ( actuator.direction > 0)
            {
                if (actuator.current_time >= (actuator.phase_time[INCHWORM_PHASE_INIT]))
                {
                    actuator.digital_output = 2;//3;

                    actuator.phase = INCHWORM_PHASE_OPEN_FG; //E1
                    actuator.current_time = 0;
                }
            }else
            {
                if (actuator.current_time >= (actuator.phase_time[INCHWORM_PHASE_INIT]))
                {
                    actuator.digital_output = 1;//5;

                    actuator.phase = INCHWORM_PHASE_AA_RETRACTION; //E5
                    actuator.current_time = 0;
                }
            }

            break;
        case INCHWORM_PHASE_OPEN_FG: //E1
            if (direction >0 )
            {
                if (actuator.current_time >= actuator.gripper_time)//phase_time[INCHWORM_PHASE_OPEN_FG])
                {

                    // si insertion alors vers E2
                    actuator.digital_output = 6;//11;
                    actuator.phase = INCHWORM_PHASE_AA_ELONGATION;
                    actuator.current_time = 0;
                }
            }
            else
            {
                if (actuator.current_time >= actuator.actuation_time)//phase_time[INCHWORM_PHASE_OPEN_FG])
                {

                    // si extraction alors vers E6
                    actuator.digital_output = 3;//7;
                    actuator.phase = INCHWORM_PHASE_CLOSE_MG;
                    actuator.current_time = 0;

                }
            }

            break;
        case INCHWORM_PHASE_AA_ELONGATION: //E2
            if (direction >0 )
            {
                if (actuator.current_time >= actuator.actuation_time)//phase_time[INCHWORM_PHASE_AA_ELONGATION])
                {

                    // si insertion alors vers E3
                    actuator.digital_output = 7;//15;
                    actuator.phase = INCHWORM_PHASE_CLOSE_FG;
                    actuator.current_time = 0;
                }
            }
            else
            {
                if (actuator.current_time >= actuator.gripper_time)//phase_time[INCHWORM_PHASE_AA_ELONGATION])
                {
                    //si extraction vers E1
                    actuator.digital_output = 1;//3;
                    actuator.phase = INCHWORM_PHASE_OPEN_FG;
                    actuator.current_time = 0;
                }

            }
            break;
        case INCHWORM_PHASE_CLOSE_FG: //E3
            if (direction >0 )
            {
                if (actuator.current_time >= actuator.gripper_time)//phase_time[INCHWORM_PHASE_CLOSE_FG])
                {

                    // si insertion alors vers E4
                    actuator.digital_output = 5;//13;
                    actuator.phase = INCHWORM_PHASE_OPEN_MG;
                    actuator.current_time = 0;
                }
            }
            else
            {
                if (actuator.current_time >= actuator.gripper_time)//phase_time[INCHWORM_PHASE_CLOSE_FG])
                {
                    // si extraction alors vers E2
                    actuator.digital_output = 5;//11;
                    actuator.phase = INCHWORM_PHASE_AA_ELONGATION;
                    actuator.current_time = 0;
                }
            }
            break;
        case INCHWORM_PHASE_OPEN_MG: //E4
            if (direction >0 )
            {
                if (actuator.current_time >= actuator.gripper_time) //phase_time[INCHWORM_PHASE_OPEN_MG])
                {
                    // si insertion alors prochaine étape  E5
                    actuator.digital_output = 1;//5;

                    actuator.phase = INCHWORM_PHASE_AA_RETRACTION;
                    actuator.current_time = 0;
                }
            }
            else
                {
                if (actuator.current_time >= actuator.actuation_time)//phase_time[INCHWORM_PHASE_OPEN_MG])
                {
                    //si extraction vers étape E3
                    actuator.digital_output = 7;//2;//15;

                    actuator.phase = INCHWORM_PHASE_CLOSE_FG;//E3
                    actuator.current_time = 0;
                }

            }
            break;
        case INCHWORM_PHASE_AA_RETRACTION: //E5
            if (direction >0 )
            {
                if (actuator.current_time >= actuator.actuation_time)//actuator.phase_time[INCHWORM_PHASE_AA_RETRACTION])
                {

                    // si insertion alors prochaine étape  E6
                    actuator.digital_output = 3;//7;
                    actuator.phase = INCHWORM_PHASE_CLOSE_MG;
                    actuator.current_time = 0;
                }
            }
            else
            {
                if (actuator.current_time >= actuator.gripper_time)//actuator.phase_time[INCHWORM_PHASE_AA_RETRACTION])
                {
                    // si extraction alors prochaine étape  E4
                    actuator.digital_output = 6;//3;//13;
                    actuator.phase = INCHWORM_PHASE_OPEN_MG;
                    actuator.current_time = 0;
                }
            }
            break;
        case INCHWORM_PHASE_CLOSE_MG: //E6
            if (direction >0 )
            {
                if (actuator.current_time >= actuator.gripper_time)//phase_time[INCHWORM_PHASE_CLOSE_MG])
                {
                    // si insertion alors prochaine étape  E1
                    actuator.digital_output = 2;//3;
                    actuator.phase = INCHWORM_PHASE_OPEN_FG;
                    actuator.current_time = 0;
                }
            }
            else
            {
                if (actuator.current_time >= actuator.gripper_time)//phase_time[INCHWORM_PHASE_CLOSE_MG])
                {
                    //si extraction alors E5

                    // si insertion alors prochaine étape  E5
                    actuator.digital_output = 2;///5;

                    actuator.phase = INCHWORM_PHASE_AA_RETRACTION;
                    actuator.current_time = 0;
                }
            }
            break;
        default:
            break;
        }
    }
    return actuator.digital_output;
}
