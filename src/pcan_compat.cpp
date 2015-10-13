
/***************************************************************/
//      compatibility with libpcan library from Peak, adapt to Linux SocketCAN
//      TODO: runtime switch to choose SocketCAN or libpcan.
/***************************************************************/

#include <ipa_canopen_core/pcan_compat.h>

HANDLE LINUX_CAN_Open(const char *devname, int mode)
{  
  return 0;
}

int CAN_Init(HANDLE h, int baudrate, int type)
{
  return -1;
}

void CAN_Close(HANDLE h)
{
}

int CAN_Write(HANDLE h, TPCANMsg *msg)
{
  return -1;
}

int LINUX_CAN_Read(HANDLE h, TPCANRdMsg *rdmsg)
{
  return -1;
}


