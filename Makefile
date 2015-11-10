

#INCLUDE:=\
#  include/Arm.h\
#  include/canopen_master/layer.h\
#  include/canopen_master/objdict.h\
#  include/canopen_master/can_layer.h\
#  include/canopen_master/master.h\
#  include/canopen_master/exceptions.h\
#  include/canopen_master/canopen.h\
#  include/canopen_master/timer.h\
#  include/canopen_402/motor.h\
#  include/canopen_402/base.h\
#  include/socketcan_interface/threading.h\
#  include/socketcan_interface/interface.h\
#  include/socketcan_interface/dummy.h\
#  include/socketcan_interface/dispatcher.h\
#  include/socketcan_interface/string.h\
#  include/socketcan_interface/asio_base.h\
#  include/socketcan_interface/reader.h\
#  include/socketcan_interface/FastDelegate.h\
#  include/socketcan_interface/socketcan.h
#
#SRC:=\
#  src/emcy.cpp \
#  src/master.cpp \
#  src/motor.cpp \
#  src/node.cpp \
#  src/objdict.cpp \
#  src/pdo.cpp \
#  src/sdo.cpp

INCLUDE:=\
  include/FTSLWA.h \
  include/ipa_canopen_core/schunkErrors.h \
  include/ipa_canopen_core/canopen.h  \
  include/ipa_canopen_core/pcan_compat.h
#  include/Arm.h \

SRC:=\
  src/ipa_canopen_core.cpp \
  src/pcan_compat.cpp \
  src/FTSLWA.cpp 
 # src/Arm.cpp \

OBJ:=$(patsubst src/%.cpp,obj/%.o,$(SRC))

#CFLAGS:=-std=c++11 -fPIC -g -Iinclude
CFLAGS:=-std=c++0x -fPIC -g -Iinclude

ifndef CXX
CXX:=c++
endif

all: lib/libPowerballArm.so 

clean: 
	-rm lib/libPowerballArm.so
	-rm $(OBJ)

lib/libPowerballArm.so: $(OBJ)
	@mkdir -p lib
	$(CXX) -shared -o $@ $^ -lboost_thread -lboost_system -lpthread


obj/%.o: src/%.cpp
	@mkdir -p obj
	$(CXX) -c $(CFLAGS) -o $@ $^


include Makefile.dep

Makefile.dep:
	$(info building Makefile.dep)
	$(CXX) $(CFLAGS) -MM $(SRC) >Makefile.dep
	

info:
	$(info SRC=$(SRC))
	$(info OBJ=$(OBJ))

.PHONY: info all clean
