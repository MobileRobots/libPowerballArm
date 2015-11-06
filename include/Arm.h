
#ifndef POWERBALLARM_H
#define POWERBALLARM_H

#include <vector>
#include "ipa_canopen_core/canopen.h"


class Arm
{
public:
	Arm(const char* canif = "can0", size_t numjoints = 5, int firstCANID = 3, unsigned int syncInterval = 20);
  virtual ~Arm();
  bool open();
  void halt();

  /* Call regularly to cause all joints to update for any new movement data received */
  void sync() {
    canopen::sendSync();
  }

  // Send new joint position data. Devices will update on next sync().
  void moveJointsTo(float pos1, float pos2, float pos3, float pos4, float pos5) 
  {
    std::vector<float> p;
    p.push_back(pos1);
    p.push_back(pos2);
    p.push_back(pos3);
    p.push_back(pos4);
    p.push_back(pos5);
    moveJointsTo(p);
  }

  // Send new joint position data. Devices will update on next sync().
  void moveJointsTo(std::vector<float> pos)
  {
    for(size_t i = 0; i < devices.size() && i < pos.size(); ++i)
    {
      moveJointTo(i, pos[i]);
    }
  }

  // Send new joint position data. Devices will update on next sync().
  void moveJointTo(size_t joint, float pos);

  void getJointPositions(float *pos1, float *pos2, float *pos3, float *pos4, float *pos5)
  {
    std::vector<float> pos = getJointPositions();
    if(pos1) *pos1 = pos[0];
    if(pos2) *pos2 = pos[1];
    if(pos3) *pos3 = pos[2];
    if(pos4) *pos4 = pos[3];
    if(pos5) *pos5 = pos[4];
  }

  void getJointPositions(std::vector<float>& pos)
  {
    pos = getJointPositions();
  }
  std::vector<float> getJointPositions();

  void moveJointBy(size_t joint, float speed);

  void moveJointsBy(std::vector<float> speeds)
  {
    for(size_t i = 0; i < devices.size(); ++i)
    {
      moveJointBy(i, speeds[i]);
    }
  }

protected:
  std::vector<canopen::Device> devices;
  std::vector <uint8_t> ids;
  std::string canifName;
  std::string chainName;
  unsigned int syncInterval;
};


#endif
