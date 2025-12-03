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


    