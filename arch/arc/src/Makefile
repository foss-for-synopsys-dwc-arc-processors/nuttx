############################################################################
# arch/arc/src/Makefile
#
#   Copyright (C) 2020 Synopsys. All
#     rights reserved.
#   Author: Wayne Ren <wei.ren@synopsys.com>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/Make.defs
-include chip$(DELIM)Make.defs

# User space related src files
CMN_UASRCS =
CMN_UCSRCS =


# files need to be linked differently
# These files contain symbols overriding symbols in libraries
HEAD_ASRCS = up_exception.S
HEAD_CSRCS = nuttx_embarc.c

CMN_ASRCS = up_switchcontext.S up_fullcontextrestore.S
CMN_ASRCS += vfork.S

CMN_CSRCS = up_assert.c up_vfork.c \
	    up_initialize.c up_initialstate.c up_interruptcontext.c \
	    up_allocateheap.c up_idle.c up_blocktask.c up_unblocktask.c \
	    up_createstack.c up_stackframe.c up_releasestack.c \
	    up_releasepending.c up_schedulesigaction.c up_puts.c up_exit.c \
	    up_usestack.c up_reprioritizertr.c up_doirq.c up_irq.c up_timerisr.c


## Nuttx configuration to embarc bsp configuration
EMBARC_BUILD_DIR = $(ARCH_SRCDIR)$(DELIM)common$(DELIM)embarc_bsp$(DELIM)obj_$(EMBARC_BOARD)_$(EMBARC_BD_VER)$(DELIM)gnu_$(EMBARC_CORE)
EMBARC_LIB = $(EMBARC_BUILD_DIR)$(DELIM)libembarc.a
EMBARC_CORE_CONFIG_H = $(EMBARC_BUILD_DIR)$(DELIM)embARC_generated$(DELIM)core_config.h
NUTTX_CORE_CONFIG_H = $(ARCH_SRCDIR)$(DELIM)common$(DELIM)core_config.h

# use embarc bsp's start objs
EMBARC_OBJS += $(EMBARC_BUILD_DIR)$(DELIM)arc$(DELIM)startup$(DELIM)arc_startup.o \
	      $(EMBARC_BUILD_DIR)$(DELIM)arc$(DELIM)startup$(DELIM)arc_cxx_support.o

CPPFLAGS += $(EXTRADEFINES)
CFLAGS += $(EXTRADEFINES)
CXXFLAGS += $(EXTRADEFINES)

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  ARCH_SRCDIR = $(TOPDIR)\arch\$(CONFIG_ARCH)\src
  NUTTX = "$(TOPDIR)\nuttx$(EXEEXT)"
  CFLAGS += -I$(ARCH_SRCDIR)\chip
  CFLAGS += -I$(ARCH_SRCDIR)\common
  CFLAGS += -I$(ARCH_SRCDIR)\$(ARCH_SUBDIR)
  CFLAGS += -I$(TOPDIR)\sched
  CFLAGS += -I$(ARCH_SRCDIR)\common\embarc_bsp\include
  CFLAGS += -I$(ARCH_SRCDIR)\common\embarc_bsp\board
  CFLAGS += -I$(ARCH_SRCDIR)\common\embarc_bsp\library
else
  ARCH_SRCDIR = $(TOPDIR)/arch/$(CONFIG_ARCH)/src
ifeq ($(WINTOOL),y)
  NUTTX = "${shell cygpath -w $(TOPDIR)/nuttx$(EXEEXT)}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/chip}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/common}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/$(ARCH_SUBDIR)}"
  CFLAGS += -I "${shell cygpath -w $(TOPDIR)/sched}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/common/embarc_bsp/include}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/common/embarc_bsp/board}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/common/embarc_bsp/library}"
else
  NUTTX = "$(TOPDIR)/nuttx$(EXEEXT)"
  CFLAGS += -I$(ARCH_SRCDIR)/chip
  CFLAGS += -I$(ARCH_SRCDIR)/common
  CFLAGS += -I$(ARCH_SRCDIR)/$(ARCH_SUBDIR)
  CFLAGS += -I$(TOPDIR)/sched
  CFLAGS += -I$(ARCH_SRCDIR)/common/embarc_bsp/include
  CFLAGS += -I$(ARCH_SRCDIR)/common/embarc_bsp/board
  CFLAGS += -I$(ARCH_SRCDIR)/common/embarc_bsp/library
endif
endif

# The "head" objects
HEAD_AOBJS = $(HEAD_ASRCS:.S=$(OBJEXT))
HEAD_COBJS = $(HEAD_CSRCS:.c=$(OBJEXT))
HEAD_OBJS = $(HEAD_AOBJS) $(HEAD_COBJS)
STARTUP_OBJS ?= $(HEAD_OBJS)

# Flat build or kernel-mode objects

ASRCS = $(CHIP_ASRCS) $(CMN_ASRCS)
AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS = $(CHIP_CSRCS) $(CMN_CSRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

# User-mode objects

UASRCS = $(CHIP_UASRCS) $(CMN_UASRCS)
UAOBJS = $(UASRCS:.S=$(OBJEXT))

UCSRCS = $(CHIP_UCSRCS) $(CMN_UCSRCS)
UCOBJS = $(UCSRCS:.c=$(OBJEXT))

USRCS = $(UASRCS) $(UCSRCS)
UOBJS = $(UAOBJS) $(UCOBJS)

KBIN = libkarch$(LIBEXT)
UBIN = libuarch$(LIBEXT)
BIN  = libarch$(LIBEXT)

# here use the linkscript generated by embARC BSP
LDFLAGS += -T $(EMBARC_BUILD_DIR)$(DELIM)linker_gnu.ldf

# Override in Make.defs if linker is not 'ld'

LDSTARTGROUP ?= --start-group
LDENDGROUP ?= --end-group

EXTRA_LIBS ?=
EXTRA_LIBPATHS ?=
LINKLIBS ?=

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BOARDMAKE = $(if $(wildcard .\board\Makefile),y,)
  LIBPATHS += -L"$(TOPDIR)\staging"
ifeq ($(BOARDMAKE),y)
  LIBPATHS += -L"$(TOPDIR)\arch\$(CONFIG_ARCH)\src\board"
endif

else
  BOARDMAKE = $(if $(wildcard ./board/Makefile),y,)

ifeq ($(WINTOOL),y)
  LIBPATHS += -L"${shell cygpath -w "$(TOPDIR)/staging"}"
ifeq ($(BOARDMAKE),y)
  LIBPATHS += -L"${shell cygpath -w "$(TOPDIR)/arch/$(CONFIG_ARCH)/src/board"}"
endif

else
  LIBPATHS += -L"$(TOPDIR)/staging"
ifeq ($(BOARDMAKE),y)
  LIBPATHS += -L"$(TOPDIR)/arch/$(CONFIG_ARCH)/src/board"
endif
endif
endif

LDLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(LINKLIBS)))
ifeq ($(BOARDMAKE),y)
  LDLIBS += -lboard
endif

LIBGCC = "${shell "$(CC)" $(ARCHCPUFLAGS) -print-libgcc-file-name}"
GCC_LIBDIR := ${shell dirname $(LIBGCC)}

VPATH += chip
VPATH += common
VPATH += $(ARCH_SUBDIR)

EXTRA_LIBS += $(EMBARC_LIB)


all: $(HEAD_OBJS) $(BIN)

.PHONY: board$(DELIM)libboard$(LIBEXT)

$(AOBJS) $(UAOBJS) $(HEAD_AOBJS): %$(OBJEXT): %.S $(EMBARC_LIB)
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(UCOBJS) $(HEAD_COBJS): %$(OBJEXT): %.c $(EMBARC_LIB)
	$(call COMPILE, $<, $@)

$(BIN) $(KBIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

$(UBIN): $(UOBJS)
	$(call ARCHIVE, $@, $(UOBJS))

$(EMBARC_LIB):
	$(Q) $(MAKE) -C common/embarc_bsp BOARD=$(EMBARC_BOARD) BD_VER=$(EMBARC_BD_VER) \
	CUR_CORE=$(EMBARC_CORE) embarc_lib
	$(Q) $(call INSTALL_LIB, $(EMBARC_CORE_CONFIG_H), $(NUTTX_CORE_CONFIG_H))


board$(DELIM)libboard$(LIBEXT):
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" libboard$(LIBEXT) EXTRADEFINES=$(EXTRADEFINES)

nuttx$(EXEEXT): $(HEAD_OBJS) board$(DELIM)libboard$(LIBEXT)
	$(Q) echo "LD: nuttx"
	$(LD) $(LDFLAGS) $(LIBPATHS) $(EXTRA_LIBPATHS) \
		-o $(NUTTX) $(HEAD_OBJS) $(EXTRA_OBJS) $(EMBARC_OBJS) \
		$(LDSTARTGROUP) $(LDLIBS) $(EXTRA_LIBS) $(LIBGCC) $(LDENDGROUP)
ifneq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) $(NM) $(NUTTX) | \
	grep -v '\(compiled\)\|\(\$(OBJEXT)$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
	sort > $(TOPDIR)$(DELIM)System.map
endif
	$(Q) echo "Disassemble nuttx"
	$(Q) $(OBJDUMP) -S $(NUTTX) > $(NUTTX).lst

# This is part of the top-level export target
# Note that there may not be a head object if layout is handled
# by the linker configuration.

export_startup: board$(DELIM)libboard$(LIBEXT) $(STARTUP_OBJS)
ifneq ($(STARTUP_OBJS),)
	$(Q) if [ -d "$(EXPORT_DIR)$(DELIM)startup" ]; then \
		cp -f $(STARTUP_OBJS) "$(EXPORT_DIR)$(DELIM)startup$(DELIM)."; \
	 else \
		echo "$(EXPORT_DIR)$(DELIM)startup does not exist"; \
	exit 1; \
	fi
endif

# Dependencies

.depend: Makefile chip$(DELIM)Make.defs $(SRCS)
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" depend
endif
	$(Q) $(MKDEP) $(patsubst %,--dep-path %,$(subst :, ,$(VPATH))) \
	 "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" clean
endif
	$(Q) $(MAKE) -C common/embarc_bsp BOARD=$(EMBARC_BOARD) BD_VER=$(EMBARC_BD_VER) \
	CUR_CORE=$(EMBARC_CORE) clean
	$(call DELFILE, $(NUTTX_CORE_CONFIG_H))
	$(call DELFILE, $(KBIN))
	$(call DELFILE, $(UBIN))
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" distclean
endif
	$(Q) $(MAKE) -C common/embarc_bsp distclean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
