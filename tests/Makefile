
CFLAGS:=-std=c++11 -fPIC -g -I../include

ifndef CXX
CXX:=c++
endif

TESTS:=$(patsubst %.cpp,%,$(shell find . -name \*.cpp))

all: ../lib/libPowerballArm.so $(TESTS)

clean: 
	-rm $(TESTS)

%: %.cpp 
	$(CXX) -c $(CFLAGS) -o $@ $^ -L../lib -lPowerballArm

../lib/libPowerballArm.so: FORCE
	$(MAKE) -C .. lib/libPowerballArm.so


info:
	$(info TESTS=$(TESTS))

FORCE:

.PHONY: info all clean
