
#include <Arm.h>
#include <utility>
#include "ipa_canopen_core/canopen.h"

#define DEGTORAD(d) (d*0.0174533)
#define RADTODEG(r) (r*57.2958)

Arm::Arm(const char *_canif, size_t numjoints, int firstCANID, unsigned int _syncInterval) :
  canifName(_canif), chainName("arm"), syncInterval(_syncInterval),
  gripCANid(12), gripDevice(gripCANid), 
  emulated(false),
  maxSpeed(72)
{
  canopen::syncInterval = std::chrono::milliseconds(syncInterval);
  devices.resize(numjoints);
  ids.reserve(numjoints+1); // plus gripper
  std::vector <std::string> names;
  names.reserve(numjoints+1); // plus gripper
  int CANid = firstCANID;
  int n = 0;
  for(std::vector<canopen::Device>::iterator i = devices.begin(); i != devices.end(); ++i)
  {
    i->setCANid(CANid);
    canopen::devices[CANid] = (*i);
    canopen::incomingPDOHandlers[ 0x180 + CANid ] = [CANid](const TPCANRdMsg m) { canopen::defaultPDO_incoming_status( CANid, m ); };
    canopen::incomingPDOHandlers[ 0x480 + CANid ] = [CANid](const TPCANRdMsg m) { canopen::defaultPDO_incoming_pos( CANid, m ); };
    ids.push_back(CANid);
    char name[16];
    snprintf(name, 16, "joint_%d", n+1);
    names.push_back(name);
    ++n;
    ++CANid;
  }
  canopen::devices[gripCANid] = gripDevice;
  CANid = gripCANid;
  canopen::incomingPDOHandlers[ 0x180 + CANid ] = [CANid](const TPCANRdMsg m) { canopen::defaultPDO_incoming_status( CANid, m ); };
  canopen::incomingPDOHandlers[ 0x480 + CANid ] = [CANid](const TPCANRdMsg m) { canopen::defaultPDO_incoming_pos( CANid, m ); };
  ids.push_back(gripCANid);
  names.push_back("gripper");
  canopen::deviceGroups[chainName] = canopen::DeviceGroup(ids, names);

  canopen::sendData = canopen::defaultPDOOutgoing_interpolated;

  minPos.resize(6);
  maxPos.resize(6);
  minPos[0] = -170.0; maxPos[0] = 170.0;
  minPos[1] = -50.0; maxPos[1] = 50.0; // about horizontal. could go forward more in some configurations
  minPos[2] = -150.0; maxPos[2] = 150.0;
  minPos[3] = -170.0; maxPos[3] = 170.0;
  minPos[4] = -140.0; maxPos[4] = 140.0;
  minPos[5] = -170.0; maxPos[5] = 170.0;

  gripMinPos = 1;
  gripMaxPos = 65;
  gripMaxSpeed = 400;
}

Arm::~Arm()
{
  haltAll();
}

bool Arm::open()
{
  canopen::init(canifName, chainName, canopen::syncInterval);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  canopen::sendSync();

  for(size_t i = 0; i < devices.size(); ++i)
  {
    uint8_t CANid = ids[i];
    canopen::setMotorState(CANid, canopen::MS_OPERATION_ENABLED);

    canopen::Device& dev = devices[i];

    // start each device current position to current actual position
    //dev.setDesiredPos((double)canopen::devices[CANid].getActualPos());
    //dev.setDesiredVel(0);
    //canopen::sendData((uint16_t)CANid, (double)dev.getDesiredPos());

    // enable movement
    canopen::controlPDO((uint16_t)CANid, canopen::CONTROLWORD_ENABLE_MOVEMENT, 0x00);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    dev.setInitialized(true);
  }

  // gripper
  canopen::setMotorState(gripCANid, canopen::MS_OPERATION_ENABLED);

  // start device current position to current actual position
  //gripDevice.setDesiredPos(canopen::devices[gripCANid].getActualPos());
  //gripDevice.setDesiredVel(0);
  //canopen::sendData(gripCANid, gripDevice.getDesiredPos());

  // enable movement
  canopen::controlPDO(gripCANid, canopen::CONTROLWORD_ENABLE_MOVEMENT, 0x00);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  gripDevice.setInitialized(true);

  return true;
}

void Arm::haltAll()
{
  std::vector<float> emptySpeeds;
  emptySpeeds.resize(devices.size());
  setJointVels(emptySpeeds);
  if(!emulated)
    canopen::halt(chainName);
}



void Arm::setJointVels(const std::vector<float>& speeds)
{
  // Todo switch to velocity mode (ipa_canopen uses "interpoladed position"
  // mode) if in a different mode
//printf("Arm::setJointVels: ");
  for(size_t i = 0; i < devices.size(); ++i)
  {
    float amt = speeds[i];
    if(amt >= 0 && amt > maxSpeed) amt = maxSpeed;
    else if(amt < 0 && amt < -maxSpeed) amt = -maxSpeed;
    canopen::Device& dev = devices[i];
    uint8_t CANid = ids[i];
    // reset device current position to current actual position but set desired
    // amount of position change to start sending
    dev.setDesiredPos(dev.getActualPos());
    dev.setDesiredVel(DEGTORAD(amt));
//printf("%d=%fdeg(%frad) ", i, amt, DEGTORAD(amt));
    //canopen::sendData(CANid, dev.getDesiredPos()); // will be sent by canopen library
  }
//puts("");
}

void Arm::setGripperVel(float amt)
{
  // Todo switch to velocity mode (ipa_canopen uses "interpoladed position"
  // mode) if in a different mode
  if(amt >= 0 && amt > gripMaxSpeed) amt = gripMaxSpeed;
  else if(amt < 0 && amt < -gripMaxSpeed) amt = -gripMaxSpeed;
  // reset device current position to current actual position but set desired
  // amount of position change to start sending
  gripDevice.setDesiredPos(gripDevice.getActualPos());
  gripDevice.setDesiredVel(DEGTORAD(amt));
  //canopen::sendData(gripCANid, gripDevice.getDesiredPos()); will be sent by canopen library
}
  

std::vector<float> Arm::getJointPositions()
{
  std::vector<float> pos;
  pos.resize(devices.size());
  for(size_t i = 0; i < devices.size(); ++i)
  {
    pos[i] = RADTODEG(devices[i].getActualPos());
  }
  return pos;
}

