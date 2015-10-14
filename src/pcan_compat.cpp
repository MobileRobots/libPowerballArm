
/***************************************************************/
//      compatibility with libpcan library from Peak, adapt to Linux SocketCAN
//      TODO: runtime switch to choose SocketCAN or libpcan.
/***************************************************************/

#include <ipa_canopen_core/pcan_compat.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/if.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>

HANDLE LINUX_CAN_Open(const char *devname, int mode)
{  
  int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if(s < 0)
    return s;
  struct ifreq ifr;
  strcpy(ifr.ifr_name, devname);
  int r = ioctl(s, SIOCGIFINDEX, &ifr);
  if(r < 0)
  {
    printf("can: error %d finding CAN interface %s\n", errno, devname);
    perror("SIOCGIFINDEX");
    return -1;
  }
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  r = bind(s, (struct sockaddr*)&addr, sizeof(addr));
  if(r < 0)
  {
    printf("can: error %d binding to CAN interface %s\n", errno, devname);
    perror("bind AF_CAN");
    CAN_Close(s);
    return -1;
  }
  printf("can: openend, socket %d\n", s);
  return s;
}

int CAN_Init(HANDLE h, int baudrate, int type)
{
  if(h > 0)
    return 0;
  else
    return -1;
}

void CAN_Close(HANDLE h)
{
  shutdown(h, SHUT_RDWR);
}

int CAN_Write(HANDLE h, TPCANMsg *msg)
{
  assert(msg);
  assert(msg->LEN <= 8);
  struct can_frame frame;
  frame.can_id = msg->ID;
  frame.can_dlc = msg->LEN;
  memcpy(frame.data, msg->DATA, 8);
  printf("write message id %d\n", msg->ID);
  return write(h, &frame, sizeof(struct can_frame));
}

int LINUX_CAN_Read(HANDLE h, TPCANRdMsg *rdmsg)
{
  assert(rdmsg);
  struct can_frame frame;
  puts("reading...\n");
  int n = read(h, &frame, sizeof(struct can_frame));
  printf("read can frame %d bytes ID %d\n", n, frame.can_id);
  if(n < 0)
  {
    perror("can: read");
    return -1;
  }
  if(n < sizeof(struct can_frame))
  {
    printf("can: error: read incomplete CAN frame\n");
    return -2;
  }
  rdmsg->Msg.ID = frame.can_id;
  /// TODO rdmsg->Msg.MSGTYPE = 
  rdmsg->Msg.LEN = frame.can_dlc;
  memcpy(rdmsg->Msg.DATA, frame.data, 8);
  // TODO rdmsg->dwTime
  // TODO rdmsg->wUsec
  return n;
}


