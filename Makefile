CC = $(CROSS_COMPILE)gcc
DEFINES = -D_GNU_SOURCE
CFLAGS = -g -O2 -Wall -Werror $(DEFINES)
YACC := bison
LEX  := flex

# dpkg-parsechangelog is significantly slow
CHANGELOG = $(shell head -n 1 ./debian/changelog)
VERS = $(shell echo "$(CHANGELOG)" | sed -n -e 's/.*(\([0-9]\+\.[0-9]\+\.[0-9]\+\).*).*/\1/p')
MAJ = $(shell echo "$(VERS)" | cut -d . -f 1)
MIN = $(shell echo "$(VERS)" | cut -d . -f 2)
REV = $(shell echo "$(VERS)" | cut -d . -f 3)

SRCS := $(wildcard *.c)
DEPS := $(SRCS:.c=.d)
BINS := skybroker skysensed skysense-cli
LIBS := -lczmq -lzmq -lserialport -lgps -lpthread -luuid -ldl

LIBSKYSENSE-SRCS := libskysense.o libskysense-local.o \
	            libskysense-remote.o

# Put there "LIBSKYSENSE-SRCS += libskysense-dummy.o" for testing
-include Makefile.dummy

all: $(BINS)

##
## docopt-gen
##

docopt-gen:
	$(MAKE) -C docopt
	cp docopt/docopt docopt-gen
	$(MAKE) -C docopt clean

##
## version.h
##

version.h: debian/changelog
	@printf "/* File is automatically generated */\n" >  version.h
	@printf "#ifndef VERSION_H\n"                     >> version.h
	@printf "#define VERSION_H\n\n"                   >> version.h
	@printf "#define SKY_VERSION 0x%08x\n\n" \
		$$(($(MAJ) << 16 | $(MIN) << 8 | $(REV))) >>  version.h
	@printf "#endif /* VERSION_H */\n"                >> version.h


%-cmd.h %-cmd.y %-cmd.l: | docopt-gen %-cmd.docopt
	$(RM) $*-cmd.h $*-cmd.l $*-cmd.y
	./docopt-gen $*-cmd.docopt

%-cmd.tab.c: %-cmd.y
	$(YACC) -o $@ --defines $*-cmd.y

%-cmd.lex.c: %-cmd.l %-cmd.tab.c
	$(LEX) -o $@ $*-cmd.l

# Generate automatic prerequisites, i.e. dependencies
# http://make.mad-scientist.net/papers/advanced-auto-dependency-generation/
%.o : %.c
	@$(MAKEDEPEND)
	$(COMPILE.c) -o $@ $<

ifneq ($(MAKECMDGOALS),clean)
-include $(DEPS)
endif

##
## skybroker
##
skybroker.o: skybroker-cmd.h version.h

skybroker: skybroker.o skybroker-cmd.tab.o skybroker-cmd.lex.o
	$(CC) $(LFLAGS) -o $@ $^ $(LIBS)

##
## skyserver (skysensed)
##
skyserver.o: skyserver-cmd.h version.h

skysensed: skyserver.o $(LIBSKYSENSE-SRCS) \
	   skyserver-cmd.tab.o skyserver-cmd.lex.o
	$(CC) $(LFLAGS) -o $@ $^ $(LIBS)

##
## skyclient (skysense-cli)
##
skyclient.o: skyclient-cmd.h version.h

skysense-cli: skyclient.o $(LIBSKYSENSE-SRCS) \
	      skyclient-cmd.tab.o skyclient-cmd.lex.o
	$(CC) $(CFLAGS) $(LFLAGS) -o $@ $^ $(LIBS) -lz

clean:
	$(RM) $(DEPS) $(BINS) *.o *~ \
		skyclient-cmd.lex.c skyclient-cmd.tab.* \
		skyclient-cmd.h skyclient-cmd.l skyclient-cmd.y \
		skyserver-cmd.lex.c skyserver-cmd.tab.* \
		skyserver-cmd.h skyserver-cmd.l skyserver-cmd.y \
		skybroker-cmd.lex.c skybroker-cmd.tab.* \
		skybroker-cmd.h skybroker-cmd.l skybroker-cmd.y

.PHONY: all clean
