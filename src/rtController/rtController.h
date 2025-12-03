#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <stdatomic.h>
#include <signal.h>
#include <strings.h>
#include <stdlib.h>

#define RT_CONTROLLER_PERIOD_NS 1000000L   // 1 ms = 1 kHz loop
#define RT_CONTROLLER_PRIORITY 80          // 0–99, requires sudo on Linux

// Topologie du bus Ethercat
/* Vendor / Product codes */
#define ED1F               0x0000aaaa, 0x00000005
#define Beckhoff_EK1100    0x00000002, 0x01c33052
#define Beckhoff_EL7031    0x00000002, 0x1b773052

/* EtherCAT positions */
#define AxisX1Pos     0, 0
#define AxisX2Pos     0, 1
#define AxisZPos      0, 2
#define AxisYPos      0, 3
#define BusCouplerPos 0, 4
#define ExtruderPos   0, 5

// Mode of rtCOntroller
typedef enum {
    CYCLE_IDLE = 0,
    CYCLE_HOMING,
    CYCLE_SEGMENTED_TRAJECTORY,
    CYCLE_ERROR,
    CYCLE_SHUTDOWN
} CycleMode;


typedef struct{

    pthread_t thread;
    pthread_attr_t attr;
    struct sched_param param;

    long period_ns;

    // Variable globale atomique pour stopper la boucle avec Ctrl-C
    atomic_int running;
}RT_CONTROLLER_TASK;

typedef struct{
    int mode;
    

}RT_CONTROLLER_STRUCT;


void RT_CONTROLLER_TASK_init(RT_CONTROLLER_TASK *rt_task, int priority, long period_ns);
void RT_CONTROLLER_TASK_start(RT_CONTROLLER_TASK *rt_task, void *(*task_func)(void *), void *arg);
void RT_CONTROLLER_TASK_join(RT_CONTROLLER_TASK *rt_task);
void RT_CONTROLLER_TASK_cleanup(RT_CONTROLLER_TASK *rt_task);


    