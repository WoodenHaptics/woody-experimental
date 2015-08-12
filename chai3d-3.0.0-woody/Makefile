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
#  $Author: seb $
#  $Date: 2014-04-03 15:07:15 +1300 (Thu, 03 Apr 2014) $
#  $Rev: 1297 $


# global path
TOP_DIR = .

# commong settings
include $(TOP_DIR)/Makefile.common

# directories
OBJ_ROOT  = $(TOP_DIR)/obj
OBJ_DIR   = $(TOP_DIR)/obj/$(CFG)/$(OS)-$(ARCH)-$(COMPILER)
SRC_DIR   = $(TOP_DIR)/src
VPATH     = $(dir $(wildcard $(SRC_DIR)/*/*))

# main sources
SOURCES   = $(shell find $(SRC_DIR) -name '*.cpp')

# include dependencies
INCLUDES  = $(shell find $(SRC_DIR) -name '*.h')

# external dependencies
EXTERNAL  = external

# required dependencies
GLEW_DIR     = $(TOP_DIR)/external/glew
CXXFLAGS    += -I$(GLEW_DIR)/include
CFLAGS      += -I$(GLEW_DIR)/include
VPATH       += $(GLEW_DIR)/src
CSOURCES    += $(wildcard $(GLEW_DIR)/src/*.c)
CSOURCES    += $(shell find $(SRC_DIR) -name '*.c')

# optional dependencies to include in static library (see Makefile.config)
ifeq ($(USE_EXTERNAL_LIBPNG), no)
PNG_DIR      = $(EXTERNAL)/libpng
CXXFLAGS    += -I$(PNG_DIR)/include
CFLAGS      += -I$(PNG_DIR)/include
VPATH       += $(PNG_DIR)/src
EXCLUDE      = $(wildcard $(PNG_DIR)/src/*_core.c)
CSOURCES    += $(filter-out $(EXCLUDE), $(wildcard $(PNG_DIR)/src/*.c))
endif
ifeq ($(USE_EXTERNAL_LIBJPEG), no)
JPG_DIR      = $(EXTERNAL)/libjpeg
CXXFLAGS    += -I$(JPG_DIR)/include
CFLAGS      += -I$(JPG_DIR)/include
VPATH       += $(JPG_DIR)/src
CSOURCES    += $(wildcard $(JPG_DIR)/src/*.c)
endif
ifeq ($(USE_EXTERNAL_GIFLIB), no)
GIF_DIR      = $(EXTERNAL)/giflib
CXXFLAGS    += -I$(GIF_DIR)/include
CFLAGS      += -I$(GIF_DIR)/include
VPATH       += $(GIF_DIR)/src
CSOURCES    += $(wildcard $(GIF_DIR)/src/*.c)
endif
ifeq ($(USE_EXTERNAL_LIB3DS), no)
3DS_DIR      = $(EXTERNAL)/lib3ds
CXXFLAGS    += -I$(3DS_DIR)/include
CFLAGS      += -I$(3DS_DIR)/include
VPATH       += $(3DS_DIR)/src
CSOURCES    += $(wildcard $(3DS_DIR)/src/*.c)
endif
ifeq ($(USE_EXTERNAL_OPENAL), no)
CFLAGS    += -DAL_ALEXT_PROTOTYPES -DAL_BUILD_LIBRARY
OPENAL_DIR = $(EXTERNAL)/openal
CFLAGS    += -I$(OPENAL_DIR)/include -I$(OPENAL_DIR)/OpenAL32/Include -I$(OPENAL_DIR)/Alc
CXXFLAGS  += -I$(OPENAL_DIR)/include -I$(OPENAL_DIR)/OpenAL32/Include -I$(OPENAL_DIR)/Alc
CSOURCES  += $(wildcard $(OPENAL_DIR)/OpenAL32/*.c)
CSOURCES  += $(wildcard $(OPENAL_DIR)/Alc/backends/*.c)
CSOURCES  += $(wildcard $(OPENAL_DIR)/Alc/*.c)
VPATH     += $(OPENAL_DIR)/OpenAL32
VPATH     += $(OPENAL_DIR)/Alc
VPATH     += $(OPENAL_DIR)/Alc/backends
VPATH     += $(OPENAL_DIR)/Alc/effects
endif
ifeq ($(USE_EXTERNAL_THEORAPLAYER), no)
CXXFLAGS  += -D__THEORA
LTP_DIR    = $(EXTERNAL)/theoraplayer
CFLAGS    += -I$(LTP_DIR)/include/theoraplayer
CXXFLAGS  += -I$(LTP_DIR)/include/theoraplayer
SOURCES   += $(wildcard $(LTP_DIR)/src/*.cpp)
VPATH     += $(LTP_DIR)/src
YUV_DIR    = $(LTP_DIR)/src/YUV/C
CFLAGS    += -I$(YUV_DIR)
CXXFLAGS  += -I$(YUV_DIR)
CSOURCES  += $(wildcard $(YUV_DIR)/*.c)
VPATH     += $(YUV_DIR)
WRAPPER_DIR = $(LTP_DIR)/src/Theora
CXXFLAGS  += -I$(WRAPPER_DIR)
SOURCES   += $(wildcard $(WRAPPER_DIR)/*.cpp)
VPATH     += $(WRAPPER_DIR)
OGG_DIR    = $(LTP_DIR)/external/ogg
CFLAGS    += -I$(OGG_DIR)/include
CXXFLAGS  += -I$(OGG_DIR)/include
CSOURCES  += $(wildcard $(OGG_DIR)/src/*.c)
VPATH     += $(OGG_DIR)/src
VORBIS_DIR = $(LTP_DIR)/external/vorbis
CFLAGS    += -I$(VORBIS_DIR)/include -I$(VORBIS_DIR)/lib
CXXFLAGS  += -I$(VORBIS_DIR)/include
CSOURCES  += $(wildcard $(VORBIS_DIR)/lib/*.c)
VPATH     += $(VORBIS_DIR)/lib
THEORA_DIR = $(LTP_DIR)/external/theora
CFLAGS    += -I$(THEORA_DIR)/include  -I$(THEORA_DIR)/lib
CXXFLAGS  += -I$(THEORA_DIR)/include
CSOURCES  += $(wildcard $(THEORA_DIR)/lib/*.c)
VPATH     += $(THEORA_DIR)/lib
CSOURCES  += $(wildcard $(THEORA_DIR)/lib/x86/*.c)
VPATH     += $(THEORA_DIR)/lib/x86
endif

# optional external binary libraries
ifeq ($(USE_DHD),yes)
CXXFLAGS += -I$(DHD_EXT)/include
endif

# objects
OBJECTS   = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(notdir $(SOURCES)))
OBJECTS  += $(patsubst %.c,   $(OBJ_DIR)/%.o, $(notdir $(CSOURCES)))

# examples subdirectories
SUBDIRS  = examples utils templates

# build rules

all: $(SUBDIRS)

$(SUBDIRS): lib | $(BIN_DIR)
	$(MAKE) -C $@

lib: $(LIB_TARGET)

$(LIB_TARGET): $(OBJECTS) | $(LIB_DIR)
	$(AR) $(ARFLAGS) $@ $?

$(OBJECTS):  $(INCLUDES) | $(OBJ_DIR)

$(BIN_DIR) $(LIB_DIR) $(OBJ_DIR):
	mkdir -p $@

$(OBJ_DIR)/%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<
	
$(OBJ_DIR)/%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $<

.PHONY: $(SUBDIRS)

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done
	-rm -f $(OBJECTS) $(LIB_TARGET) *~
	-rm -rf $(LIB_DIR) $(OBJ_DIR)
