#include "sRTFifo.h"
#define FIFO_ALREADY_OPEN             1000
#define FIFO_ALREADY_CLOSE            1001
#define OPEN_CMD_FIFO_FAILED          1002

#define OPEN_STATUS_FIFO_FAILED       1003
#define OPEN_ACKNO_FIFO_FAILED        1004
#define OPEN_SAVE_DATA_FIFO_FAILED    1005
#define OPEN_PARAM_FIFO_FAILED        1006
#define OPEN_IDENT_PARAM_FIFO_FAILED    1007
#define OPEN_FIFO_FAILED                1100
sRTFifo::sRTFifo()
{

    bIsOpen     = false;
    id          = 0;
}

int sRTFifo::Open(const char *fifo_path, int flags)
{
    id = open(fifo_path, flags );
    if ( id < 0 )
        return id;
    bIsOpen = true;
    return 0;
}
int sRTFifo::Delete(void)
{
    if (!bIsOpen)
    return 0;
    close(id);
    bIsOpen = false;
    return 0;
}
int sRTFifo::getID(void)
{
    if ( bIsOpen)
    return id;
    else
    return -1;
}
int sRTFifo::Write(const void *arg, size_t ss_arg)
{
    if (!bIsOpen)
    return -1;
    return write(id, arg, ss_arg);
}
int sRTFifo::Read(void *buf, size_t nb_bytes)
{
    if (!bIsOpen)
    return -1;
    return read(id, buf, nb_bytes);
}
