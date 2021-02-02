CC = $(CROSS_COMPILE)gcc
DEFINES = -D_GNU_SOURCE
CFLAGS = -g -O2 -Wall -Werror $(DEFINES)
YACC := bison
LEX  := flex

# dpkg-parsechangelog is significantly slow
CHANGELOG = $(shell head -n 1 ./debian/changelog)
VER_STR = $(shell echo "$(CHANGELOG)" | sed -n -e 's/.*(\(.*[0-9]\+\.[0-9]\+\.[0-9]\+.*\)).*/\1/p')
VER = $(shell echo "$(VER_STR)" | sed -n -e 's/.*\([0-9]\+\.[0-9]\+\.[0-9]\+\).*/\1/p')
MAJ = $(shell echo "$(VER)" | cut -d . -f 1)
MIN = $(shell echo "$(VER)" | cut -d . -f 2)
REV = $(shell echo "$(VER)" | cut -d . -f 3)

SRCS := $(wildcard *.c)
DEPS := $(SRCS:.c=.d)
BINS := skybroker skysensed skybmsd skypsu skysense-cli

LIBSKYCHARGE-SRCS := libskysense.o libskysense-local.o libskysense-remote.o \
		     libskybms.o libskydp.o bms-btle.o libskypsu.o libi2c/i2c.o \
		     gpio.o
LIBSKYCHARGE-LIBS := -lpthread -lserialport -lczmq -lzmq -luuid -lgps -lz

# Put there "LIBSKYCHARGE-SRCS += libskysense-dummy.o" for testing.
# Do not include the file if dpkg-buildpackage build is performed.
ifndef DEB_BUILD_ARCH
-include Makefile.dummy
endif

ifeq ("$(origin V)", "command line")
  VERBOSE = $(V)
endif
ifndef VERBOSE
  VERBOSE = 0
endif

ifeq ($(VERBOSE),1)
  Q =
else
  Q = @
endif

all: $(BINS)

##
## docopt-gen
##

docopt-gen:
ifneq ($(VERBOSE),1)
	@echo " LD $@"
endif
	$(Q)$(MAKE) -C docopt
	$(Q)cp docopt/docopt docopt-gen
	$(Q)$(MAKE) -C docopt clean

##
## version.h
##

version.h: debian/changelog
ifneq ($(VERBOSE),1)
	@echo " GEN $@"
endif
	@printf "/* File is automatically generated */\n" >  version.h
	@printf "#ifndef VERSION_H\n"                     >> version.h
	@printf "#define VERSION_H\n\n"                   >> version.h
	@printf "#define SKY_VERSION     0x%08x\n" \
		$$(($(MAJ) << 16 | $(MIN) << 8 | $(REV))) >> version.h
	@printf "#define SKY_VERSION_STR \"%s\"\n\n" $(VER_STR) \
							  >> version.h
	@printf "#endif /* VERSION_H */\n"                >> version.h


%-cmd.h %-cmd.y %-cmd.l: | docopt-gen %-cmd.docopt
ifneq ($(VERBOSE),1)
	@echo " GEN $@"
endif
	$(Q)$(RM) $*-cmd.h $*-cmd.l $*-cmd.y
	$(Q)./docopt-gen $*-cmd.docopt

%-cmd.tab.c: %-cmd.y
ifneq ($(VERBOSE),1)
	@echo "YACC $@"
endif
	$(Q)$(YACC) -o $@ --defines $*-cmd.y

%-cmd.lex.c: %-cmd.l %-cmd.tab.c
ifneq ($(VERBOSE),1)
	@echo " LEX $@"
endif
	$(Q)$(LEX) -o $@ $*-cmd.l

# Generate automatic prerequisites, i.e. dependencies
# http://make.mad-scientist.net/papers/advanced-auto-dependency-generation/
%.o : %.c
	@$(MAKEDEPEND)
ifneq ($(VERBOSE),1)
	@echo "  CC $@"
endif
	$(Q)$(COMPILE.c) -o $@ $<

ifneq ($(MAKECMDGOALS),clean)
-include $(DEPS)
endif

##
## skybroker
##
skybroker.o: skybroker-cmd.h version.h

skybroker: skybroker.o skybroker-cmd.tab.o skybroker-cmd.lex.o
ifneq ($(VERBOSE),1)
	@echo "  LD $@"
endif
	$(Q)$(CC) $(LFLAGS) -o $@ $^ -lczmq -lzmq -lpthread

##
## skyserver (skysensed)
##
skyserver.o: skyserver-cmd.h version.h

skysensed: skyserver.o avahi.o $(LIBSKYCHARGE-SRCS) \
	   skyserver-cmd.tab.o skyserver-cmd.lex.o
ifneq ($(VERBOSE),1)
	@echo "  LD $@"
endif
	$(Q)$(CC) $(LFLAGS) -o $@ $^ $(LIBSKYCHARGE-LIBS) -lavahi-client -lavahi-common

##
## skybmsd (skysensed)
##
skybms.o: skybms-cmd.h version.h

skybmsd: skybms.o libskybms.o bms-btle.o skybms-cmd.tab.o skybms-cmd.lex.o
ifneq ($(VERBOSE),1)
	@echo "  LD $@"
endif
	$(Q)$(CC) $(LFLAGS) -o $@ $^ -lserialport

##
## skypsu (skysensed)
##
skypsu.o: libskypsu.h

skypsu: skypsu.o libskypsu.o libi2c/i2c.o
ifneq ($(VERBOSE),1)
	@echo "  LD $@"
endif
	$(Q)$(CC) $(LFLAGS) -o $@ $^


##
## skyclient (skysense-cli)
##
skyclient.o: skyclient-cmd.h version.h

skysense-cli: skyclient.o $(LIBSKYCHARGE-SRCS) \
	      skyclient-cmd.tab.o skyclient-cmd.lex.o
ifneq ($(VERBOSE),1)
	@echo "  LD $@"
endif
	$(Q)$(CC) $(CFLAGS) $(LFLAGS) -o $@ $^ $(LIBSKYCHARGE-LIBS)

package:
	dpkg-buildpackage -us -uc

clean:
	$(Q)$(RM) $(DEPS) $(BINS) *.o libi2c/*.o *~ \
		skyclient-cmd.lex.c skyclient-cmd.tab.* \
		skyclient-cmd.h skyclient-cmd.l skyclient-cmd.y \
		skyserver-cmd.lex.c skyserver-cmd.tab.* \
		skyserver-cmd.h skyserver-cmd.l skyserver-cmd.y \
		skybroker-cmd.lex.c skybroker-cmd.tab.* \
		skybroker-cmd.h skybroker-cmd.l skybroker-cmd.y \
		skybms-cmd.lex.c skybms-cmd.tab.* \
		skybms-cmd.h skybms-cmd.l skybms-cmd.y

distclean: clean
	$(Q)$(RM) docopt-gen

# Don't delete intermediate files like *-cmd.lex.c or *-cmd.tab.c
# We don't need them, but I don't know how to silent 'rm' command
.SECONDARY:

.PHONY: all clean distclean package
