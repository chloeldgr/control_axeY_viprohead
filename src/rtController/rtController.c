#define _GNU_SOURCE
#include "rtController.h"

// Global pointer to the real-time controller task
RT_CONTROLLER_TASK *rt_controller_task;

/**
 * @brief stop the real-time loop on SIGINT
 * 
 * @param signal 
 */
void handle_sigint(int signal) {
    if (signal == SIGINT)
    {
        rt_controller_task->running = 0;
        printf("\nSIGINT reçu → arrêt de la boucle…\n");
    }
}

/**
 * @brief Set the signal action object
 * 
 */
void set_signal_action(void)
{
    // Declare the sigaction structure
    struct sigaction act;

    // Set all of the structure's bits to 0 to avoid errors
    // relating to uninitialized variables...
    bzero(&act, sizeof(act));
    // Set the signal handler as the default action
    act.sa_handler = &handle_sigint;
    // Apply the action in the structure to the
    // SIGINT signal (ctrl-c)
    sigaction(SIGINT, &act, NULL);
}


/**
 * @brief Add nanoseconds to a timespec structure
 * 
 * @param t 
 * @param ns 
 */
void timespec_add_ns(struct timespec *t, long ns) {
    t->tv_nsec += ns;
    while (t->tv_nsec >= 1000000000L) {
        t->tv_nsec -= 1000000000L;
        t->tv_sec += 1;
    }
}



void *rt_thread(void *arg) {
    struct timespec next;
    int i = 0;
    arg = arg; // unused
    clock_gettime(CLOCK_MONOTONIC, &next);

    printf("Real-time loop starting (1 kHz)...\n");

    while (rt_controller_task->running) {
        // Do the job

        
        // Compute next wake-up
        timespec_add_ns(&next, rt_controller_task->period_ns);

        // Sleep precisely until next time
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }
    printf("Real-time loop stopped.\n");

    return NULL;
}

int main() {
    rt_controller_task = malloc(sizeof(RT_CONTROLLER_TASK));

    // Change SIGINT's associated action
    set_signal_action();

    // Use real-time FIFO scheduling
    RT_CONTROLLER_TASK_init(rt_controller_task, RT_CONTROLLER_PRIORITY, RT_CONTROLLER_PERIOD_NS);

    printf("Creating RT thread...\n");

    RT_CONTROLLER_TASK_start(rt_controller_task, rt_thread, NULL);

    RT_CONTROLLER_TASK_join(rt_controller_task);

    
    return 0;
}

/**
 * @brief initialize the RT_CONTROLLER_TASK structure
 * 
 * @param rt_task 
 * @param priority 
 * @param period_ns 
 */
void RT_CONTROLLER_TASK_init(RT_CONTROLLER_TASK *rt_task, int priority, long period_ns)
{
    pthread_attr_init(&rt_task->attr);


    // Use real-time FIFO scheduling
    pthread_attr_setschedpolicy(&rt_task->attr, SCHED_FIFO);
    rt_task->param.sched_priority = priority;   // 0–99, requires sudo on Linux
    pthread_attr_setschedparam(&rt_task->attr, &rt_task->param);
    pthread_attr_setinheritsched(&rt_task->attr, PTHREAD_EXPLICIT_SCHED);

    rt_task->period_ns = period_ns;   

}

/**
 * @brief start the real-time task
 * 
 * @param rt_task 
 * @param task_func 
 * @param arg 
 */
void RT_CONTROLLER_TASK_start(RT_CONTROLLER_TASK *rt_task, void *(*task_func)(void *), void *arg)
{
    rt_task->running = 1;
    
    if (pthread_create(&rt_task->thread, &rt_task->attr, task_func, arg) != 0) {
        perror("pthread_create");
        return;
    }
    
}

/**
 * @brief join the real-time task
 * 
 * @param rt_task 
 */
void RT_CONTROLLER_TASK_join(RT_CONTROLLER_TASK *rt_task)  
{
    pthread_join(rt_task->thread, NULL);
}   
