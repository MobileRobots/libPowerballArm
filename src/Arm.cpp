
#include <Arm.h>
#include <utility>
#include "ipa_canopen_core/canopen.h"

#define DEGTORAD(d) (d*0.0174533)
#define RADTODEG(r) (r*57.2958)

Arm::Arm(const char *_canif, size_t numjoints, int firstCANID, unsigned int _syncInterval) :
canifName(_canif), chainName("arm"), syncInterval(_syncInterval)
{
  canopen::syncInterval = std::chrono::milliseconds(syncInterval);
  devices.resize(numjoints);
  ids.reserve(numjoints+1); // plus gripper
  std::vector <std::string> names;
  names.reserve(numjoints+1); // plus gripper
  int CANid = firstCANID;
  int num = 1;
  for(std::vector<canopen::Device>::iterator i = devices.begin(); i != devices.end(); ++i)
  {
    i->setCANid(CANid);
    canopen::devices[CANid] = (*i);
    canopen::incomingPDOHandlers[ 0x180 + CANid ] = [CANid](const TPCANRdMsg m) { canopen::defaultPDO_incoming_status( CANid, m ); };
    canopen::incomingPDOHandlers[ 0x480 + CANid ] = [CANid](const TPCANRdMsg m) { canopen::defaultPDO_incoming_pos( CANid, m ); };
    ids.push_back(CANid);
    std::string name("joint_");
    name += num++;
    names.push_back(name);
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
}

Arm::~Arm()
{
  halt();
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
    dev.setDesiredPos((double)canopen::devices[CANid].getActualPos());
    dev.setDesiredVel(0);
    canopen::sendData((uint16_t)CANid, (double)dev.getDesiredPos());

    // enable movement
    canopen::controlPDO((uint16_t)CANid, canopen::CONTROLWORD_ENABLE_MOVEMENT, 0x00);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    dev.setInitialized(true);
  }

  // gripper
  canopen::setMotorState(gripCANid, canopen::MS_OPERATION_ENABLED);

  // start device current position to current actual position
  gripDevice.setDesiredPos(canopen::devices[gripCANid].getActualPos());
  gripDevice.setDesiredVel(0);
  canopen::sendData(gripCANid, gripDevice.getDesiredPos());

  // enable movement
  canopen::controlPDO(gripCANid, canopen::CONTROLWORD_ENABLE_MOVEMENT, 0x00);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  gripDevice.setInitialized(true);

  return true;
}

void Arm::halt()
{
  canopen::halt(canifName, chainName, std::chrono::milliseconds(syncInterval));
}

void Arm::moveJointTo(size_t i, float pos)
{
  canopen::Device& dev = devices[i];
  dev.setDesiredPos(DEGTORAD(pos));
  uint8_t CANid = ids[i];
  canopen::sendData(CANid, dev.getDesiredPos());
}

void Arm::moveGripperTo(float pos)
{
  gripDevice.setDesiredPos(DEGTORAD(pos));
  canopen::sendData(gripCANid, gripDevice.getDesiredPos());
}


void Arm::moveJointBy(size_t i, float amt)
{
  if(amt > 0 && amt > maxSpeed) amt = maxSpeed;
  else if(amt < 0 && amt < -maxSpeed) amt = -maxSpeed;
  canopen::Device& dev = devices[i];
  // reset device current position to current actual position but set desired
  // amount of position change to start sending
  uint8_t CANid = ids[i];
  dev.setDesiredPos(dev.getActualPos());
  dev.setDesiredVel(DEGTORAD(amt));
  canopen::sendData(CANid, dev.getDesiredPos());
}

void Arm::moveGripperBy(float amt)
{
  if(amt > 0 && amt > maxSpeed) amt = maxSpeed;
  else if(amt < 0 && amt < -maxSpeed) amt = -maxSpeed;
  // reset device current position to current actual position but set desired
  // amount of position change to start sending
  gripDevice.setDesiredPos(gripDevice.getActualPos());
  gripDevice.setDesiredVel(DEGTORAD(amt));
  canopen::sendData(gripCANid, gripDevice.getDesiredPos());
}
  

std::vector<float> Arm::getJointPositions()
{
  std::vector<float> pos;
  pos.resize(devices.size());
  for(size_t i = 0; i < devices.size(); ++i)
  {
    pos[i] = RADTODEG(devices[i].getActualPos());
  }
}

