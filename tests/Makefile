
#CFLAGS:=-std=c++11 -fPIC -g -I../include
CFLAGS:=-std=c++0x -fPIC -g -I../include

ifndef CXX
CXX:=c++
endif

TESTS:=$(patsubst %.cpp,%,$(shell find . -name \*.cpp))

all: $(TESTS)

clean: 
	-rm $(TESTS)

%: %.cpp ../lib/libPowerballArm.so
	$(CXX) $(CFLAGS) -o $@ $^ -L../lib -lPowerballArm   -lboost_system

../lib/libPowerballArm.so: FORCE
	$(MAKE) -C .. lib/libPowerballArm.so


info:
	$(info TESTS=$(TESTS))

FORCE:

.PHONY: info all clean
