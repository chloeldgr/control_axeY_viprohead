#ifndef SRTFIFO_H
#define SRTFIFO_H
#include <stdlib.h>
#include <unistd.h>
#include <QDebug>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


class sRTFifo
{
public:
    sRTFifo(void);
    ~sRTFifo();

    int Open(const char *fifo_path, int flags);
    int Delete(void);
    int getID(void);
    int Write(const void *arg, size_t ss_arg);
    int Read(void *buf, size_t nb_bytes);
    bool getOpenStatus(void){return bIsOpen;}

protected:
    int         id; // fifo id
    bool        bIsOpen;

};
#endif // SRTFIFO_H
