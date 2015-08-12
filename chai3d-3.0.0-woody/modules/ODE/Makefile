#  Software License Agreement (BSD License)
#  Copyright (c) 2003-2014, CHAI3D.
#  (www.chai3d.org)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of CHAI3D nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: conti $
#  $Date: 2014-01-02 11:48:36 +1300 (Thu, 02 Jan 2014) $
#  $Rev: 1242 $


# global path
TOP_DIR = .

# commong settings
include $(TOP_DIR)/Makefile.common

# directories
OBJ_ROOT  = $(TOP_DIR)/obj
OBJ_DIR   = $(TOP_DIR)/obj/$(CFG)/$(OS)-$(ARCH)-$(COMPILER)
SRC_DIR   = $(TOP_DIR)/src
VPATH     = $(SRC_DIR)

# main sources
SOURCES   = $(wildcard $(SRC_DIR)/*.cpp)

# include dependencies
INCLUDES  = $(wildcard $(SRC_DIR)/*.h)

# required dependencies
VPATH    += $(dir $(wildcard $(ODE_DIR)/ode/src/* $(ODE_DIR)/ode/src/*/* $(ODE_DIR)/ode/src/*/*/*))
CSOURCES  = $(wildcard $(ODE_DIR)/ode/src/*.c)
SOURCES  += $(wildcard $(ODE_DIR)/ode/src/*.cpp)
SOURCES  += $(wildcard $(ODE_DIR)/ode/src/joints/*.cpp)
VPATH    += $(dir $(wildcard $(ODE_DIR)/OPCODE/* $(ODE_DIR)/OPCODE/*/*))
SOURCES  += $(wildcard $(ODE_DIR)/OPCODE/*.cpp)
SOURCES  += $(wildcard $(ODE_DIR)/OPCODE/Ice/*.cpp)

# include path
CXXFLAGS += -I$(ODE_DIR)/ode/src -I$(ODE_DIR)/ode/src/joints -I$(ODE_DIR)/OPCODE -I$(ODE_DIR)/OPCODE/Ice
CFLAGS   += -I$(ODE_DIR)/include -I$(ODE_DIR)/ode/src

# objects
OBJECTS   = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(notdir $(SOURCES)))
OBJECTS  += $(patsubst %.c,   $(OBJ_DIR)/%.o, $(notdir $(CSOURCES)))

# ODE specific defines
CFLAGS   += -DdDOUBLE
CXXFLAGS += -DdDOUBLE

# examples
SUBDIRS = examples

# target rules
all: $(SUBDIRS)

$(SUBDIRS): chai3d lib | $(BIN_DIR)
	$(MAKE) -C $@

lib: $(LIB_TARGET)

$(LIB_TARGET): $(OBJECTS) | $(LIB_DIR)
	$(AR) $(ARFLAGS) $@ $?

$(OBJECTS): $(INCLUDES) | $(OBJ_DIR)

$(BIN_DIR) $(LIB_DIR) $(OBJ_DIR):
	mkdir -p $@

chai3d:
	$(MAKE) -C $(CHAI3D) lib

.PHONY: chai3d $(SUBDIRS)
	
# object file compilation
$(OBJ_DIR)/%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<
$(OBJ_DIR)/%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $<	

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done
	-rm -f $(OBJECTS) $(LIB_TARGET) *~
	-rm -rf $(LIB_DIR) $(OBJ_DIR)
