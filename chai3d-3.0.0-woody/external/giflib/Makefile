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


# top level directory
TOP_DIR = .

# determine build configuration
ifneq ($(CFG),debug)
CFG = release
endif

# determine platform
ARCH   ?= $(shell echo `uname -m`)
SYSTEM ?= $(shell echo `uname -s`)
ifeq ($(SYSTEM), Linux)
OS       = lin
CC      ?= gcc
COMPILER = $(CC)
endif
ifeq ($(SYSTEM), Darwin)
OS       = mac
CC      ?= clang
COMPILER = $(CC)
endif

# paths
LIB_DIR = $(TOP_DIR)/lib/$(CFG)/$(OS)-$(ARCH)-$(COMPILER)
INC_DIR = $(TOP_DIR)/include
SRC_DIR = $(TOP_DIR)/src
OBJ_DIR = $(TOP_DIR)/obj/$(CFG)/$(OS)-$(ARCH)-$(COMPILER)

# compiler flags
CFLAGS  = -I$(INC_DIR) -I$(SRC_DIR) -fsigned-char

# librarian flags
ARFLAGS  = rs

# build configuration specific flags
ifneq ($(CFG),debug)
CFLAGS += -O3 -w
else
CFLAGS += -O0 -g -DDEBUG
endif

# set target architecture
ifeq ($(OS), lin)
ifeq ($(ARCH), x86_64)
CFLAGS += -m64
endif
ifeq ($(ARCH), i686)
CFLAGS += -m32 -march=i686 -msse
endif
endif
ifeq ($(OS), mac)
CFLAGS += -arch $(ARCH)
endif

# target
LIB_TARGET = $(LIB_DIR)/libgif.a

# source files
EXCLUDE  = $(wildcard $(SRC_DIR)/*_core.c)
SOURCES  = $(filter-out $(EXCLUDE), $(wildcard $(SRC_DIR)/*.c))
OBJECTS  = $(patsubst %.c, $(OBJ_DIR)/%.o, $(notdir $(SOURCES)))
INCLUDES = $(wildcard $(INC_DIR)/*.h)


# target rules

all: lib

lib: $(LIB_TARGET)

$(LIB_TARGET): $(OBJECTS) | $(LIB_DIR)
	$(AR) $(ARFLAGS) $(LIB_TARGET) $?

$(OBJECTS): $(INCLUDES) | $(OBJ_DIR)

$(LIB_DIR) $(OBJ_DIR):
	mkdir -p $@

$(OBJ_DIR)/%.o : $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

tags:
	find ../.. -name \*.cpp -o -name \*h | xargs etags -o TAGS

clean:
	-rm -f $(OBJECTS) $(LIB_TARGET) *~
	-rm -rf $(OBJ_DIR)
