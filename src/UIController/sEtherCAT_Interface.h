#ifndef SETHERCAT_INTERFACE_H
#define SETHERCAT_INTERFACE_H

#include <QObject>
 #include <QProcess>
#include <QTimer>
#include <QDebug>

//#include <QtWidgets>
//#include <ecrt.h>
//typedef struct{

//    u_int16_t   alias;
//    u_int16_t   position;
//    u_int32_t   ProductCode;
//    u_int32_t   VendorID;
//     ec_sdo_request_t *sdo_home_offset;
//     ec_sdo_request_t *sdo_max_following_error;
//     ec_sdo_request_t *sdo_max_profile_velocity;
//     ec_sdo_request_t *sdo_quick_stop_deceleration;
//     ec_sdo_request_t *sdo_speed_switch_search;
//     ec_sdo_request_t *sdo_speed_zero_search;
//     ec_sdo_request_t *sdo_homing_acceleration;
//     ec_sdo_request_t *sdo_current_threshold_homing;
//     ec_sdo_request_t *sdo_home_position;

//    ec_slave_config_t       *sc;

//    // Several offset PDO entries
//    unsigned int off_cur_actual_value;
//    unsigned int off_homing_method; // Refered to homing methode
//    unsigned int off_following_err;
//    unsigned int off_inter_buffer_status;
//    unsigned int off_acc_profile;
//    unsigned int off_vel_profile;
//    unsigned int off_dec_profile;
//    //offset of PDO Entires;
//    unsigned int off_Control_Word;
//    unsigned int off_Target_Position;
//    unsigned int off_Target_Velocity;
//    unsigned int off_Target_Torque;
//    unsigned int off_Position_Offset;
//    unsigned int off_Velocity_Offset;
//    unsigned int off_Torque_Offset;
//    unsigned int off_Operation_Mode;
//    unsigned int off_Digital_Output_funct;
//    unsigned int off_Touch_Probe_funct;

//    unsigned int off_Status_Word;
//    unsigned int off_Position_Actual;
//    unsigned int off_Velocity_Actual;
//    unsigned int off_Torque_Actual;
//    unsigned int off_Operation_Display_Mode;
//    unsigned int off_Digital_Input_funct;
//    unsigned int off_Touch_Probe_Status;
//    unsigned int off_Touch_Probe_Pos_pos;
//    unsigned int off_Touch_Probe_Pos_neg;


//    uint8_t operationmode;
//    uint16_t controlword;
//    //uint16_t statusword;
//    uint16_t Status;
//    uint8_t *pdomain_pd;
//    uint8_t bIsRunning;
//    uint8_t bIsShutdown;
//    uint8_t bIsSwitchon;
//    uint8_t bIsEnable;
//    uint8_t bIsInit;
//    uint8_t bIsCal;
//    int homing_submode;
//    int homing_method;

//}SLAVE_STRUCT;
//#define NB_SLAVES 3


//class CommandException:
//    public runtime_error
//{
//    friend class Command;

//    protected:
//        /** Constructor with char * parameter. */
//        CommandException(
//                const string &msg /**< Message. */
//                ): runtime_error(msg) {}

//        /** Constructor with stringstream parameter. */
//        CommandException(
//                const stringstream &s /**< Message. */
//                ): runtime_error(s.str()) {}
//};
#define NB_SLAVES 3
class sEtherCAT_Interface : public QObject
{
    Q_OBJECT
    QProcess *pEtherCAT_process;
    QString program;
    QStringList arguments;
    u_int8_t    master_id;
    QTimer  *pTimerECStatus;
    bool bIsMasterRunning;
public:
    explicit sEtherCAT_Interface(QObject *parent = 0);
    ~sEtherCAT_Interface();

    void configure_slaves(void);


public slots:
    void readout() ;
    void readerr() ;

signals:
    void ECSlave_configured(void);

};

#endif // SETHERCAT_INTERFACE_H
