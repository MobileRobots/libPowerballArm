
#CFLAGS:=-std=c++11 -fPIC -g -I../include
CFLAGS:=-std=c++0x -fPIC -g -I../include

ifndef ARIA
ARIA=/usr/local/Aria
endif

ARIACFLAGS:=-I$(ARIA)/include

ifndef CXX
CXX:=c++
endif

SRC:=$(shell find . -name \*.cpp)
ALL:=$(patsubst %.cpp,%,$(SRC))

all: $(ALL)

clean: 
	-rm $(ALL)

%: %.cpp ../lib/libPowerballArm.so
	$(CXX) $(CFLAGS) -o $@ $< -L../lib -lPowerballArm   -lboost_system

armdemo: armdemo.cpp ../lib/libPowerballArm.so 
	$(CXX) $(CFLAGS) $(ARIACFLAGS) -o $@ $< -L../lib -L$(ARIA)/lib -lPowerballArm -lAria -lboost_system -pthread -ldl -lrt

../lib/libPowerballArm.so: FORCE
	$(MAKE) -C .. lib/libPowerballArm.so


info:
	$(info ALL=$(ALL))

include Makefile.dep

Makefile.dep:
	$(info building Makefile.dep)
	$(CXX) $(CFLAGS) $(ARIACFLAGS) -MM $(SRC) >Makefile.dep
	

FORCE:

.PHONY: info all clean
