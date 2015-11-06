
#ifndef POWERBALLARM_H
#define POWERBALLARM_H

#include <vector>
#include "ipa_canopen_core/canopen.h"


class Arm
{
public:
	Arm(const char* canif = "can0", size_t numjoints = 6, int firstCANID = 3, unsigned int syncInterval = 20);
  virtual ~Arm();
  bool open();

  /// stop all joints in this arm. 
  void haltAll();

  /* Call regularly to cause all joints to update for any new movement data
 * received. Joints will stop if they stop receiving sync messages. */
  void sync() {
    canopen::sendSync();
  }

  // Send new joint position data. Devices will update on next sync(). Positions
  // are in degrees.
  void moveJointsTo(float pos1, float pos2, float pos3, float pos4, float pos5, float pos6) 
  {
    std::vector<float> p;
    p.reserve(6);
    p.push_back(pos1);
    p.push_back(pos2);
    p.push_back(pos3);
    p.push_back(pos4);
    p.push_back(pos5);
    p.push_back(pos6);
    moveJointsTo(p);
  }

  // Send new joint position data. Devices will update on next sync(). Give positions
  // in degrees.
  void moveJointsTo(std::vector<float> pos)
  {
    for(size_t i = 0; i < devices.size() && i < pos.size(); ++i)
    {
      moveJointTo(i, pos[i]);
    }
  }

  // Send new joint position data. Devices will update on next sync(). Give
  // position in degrees.
  void moveJointTo(size_t joint, float pos);

  // Get current joint positions in degrees.
  void getJointPositions(float *pos1, float *pos2, float *pos3, float *pos4, float *pos5, float *pos6)
  {
    std::vector<float> pos = getJointPositions();
    if(pos1) *pos1 = pos[0];
    if(pos2) *pos2 = pos[1];
    if(pos3) *pos3 = pos[2];
    if(pos4) *pos4 = pos[3];
    if(pos5) *pos5 = pos[4];
    if(pos6) *pos6 = pos[5];
  }

  // Get current joint positions in degrees.
  void getJointPositions(std::vector<float>& pos)
  {
    pos = getJointPositions();
  }
  std::vector<float> getJointPositions();

  // Begin sending incremented positions to joint over time according to speed
  // (degrees/sec)
  void moveJointBy(size_t joint, float speed);

  // Begin sending incremented positions to joints over time according to speed
  // (degrees/sec)
  void moveJointsBy(std::vector<float> speeds)
  {
    for(size_t i = 0; i < devices.size(); ++i)
    {
      moveJointBy(i, speeds[i]);
    }
  }

  void moveGripperTo(float pos);

  void moveGripperBy(float speed);

  float getGripperPosition()
  {
    // todo
    return 0;
  }

protected:
  std::vector<canopen::Device> devices;
  std::vector <uint8_t> ids;
  std::string canifName;
  std::string chainName;
  unsigned int syncInterval;
  uint8_t gripCANid;
  canopen::Device gripDevice;

public:
  float maxSpeed;
  std::vector<float> minPos;
  std::vector<float> maxPos;
  float gripMinPos;
  float gripMaxPos;
  float gripMaxSpeed;
};


#endif
