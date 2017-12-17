CC = $(CROSS_COMPILE)gcc
DEFINES = -D_GNU_SOURCE
CFLAGS = -g -O2 -Wall -Werror $(DEFINES)
YACC := bison
LEX  := flex

# dpkg-parsechangelog is significantly slow
CHANGELOG = $(shell head -n 1 ./debian/changelog)
VERS = $(shell echo "$(CHANGELOG)" | sed -n -e 's/.*(\([^)]*\).*/\1/p')
MAJ = $(shell echo "$(VERS)" | cut -d . -f 1)
MIN = $(shell echo "$(VERS)" | cut -d . -f 2)
REV = $(shell echo "$(VERS)" | cut -d . -f 3)

SRCS := $(wildcard *.c)
BINS := skysensed skysense-cli
LIBS := -lzmq -lserialport -lpthread -ldl

LIBSKYSENSE-SRCS := libskysense.o libskysense-local.o \
	            libskysense-remote.o libskysense-dummy.o

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


##
## skyserver (skysensed)
##

skyserver-cmd.h skyserver-cmd.y skyserver-cmd.l: docopt-gen skyserver-cmd.docopt
	$(RM) skyserver-cmd.h skyserver-cmd.l skyserver-cmd.y
	./docopt-gen skyserver-cmd.docopt

skyserver-cmd.tab.c: skyserver-cmd.y
	$(YACC) -o $@ --defines skyserver-cmd.y

skyserver-cmd.lex.c: skyserver-cmd.l
	$(LEX) -o $@ skyserver-cmd.l

skyserver.o: skyserver-cmd.h version.h

skysensed: skyserver.o $(LIBSKYSENSE-SRCS) \
	   skyserver-cmd.tab.o skyserver-cmd.lex.o
	$(CC) $(LFLAGS) -o $@ $^ $(LIBS)

##
## skyclient (skysense-cli)
##

skyclient-cmd.h skyclient-cmd.y skyclient-cmd.l: docopt-gen skyclient-cmd.docopt
	$(RM) skyclient-cmd.h skyclient-cmd.l skyclient-cmd.y
	./docopt-gen skyclient-cmd.docopt

skyclient-cmd.tab.c: skyclient-cmd.y
	$(YACC) -o $@ --defines skyclient-cmd.y

skyclient-cmd.lex.c: skyclient-cmd.l
	$(LEX) -o $@ skyclient-cmd.l

skyclient.o: skyclient-cmd.h version.h

skysense-cli: skyclient.o $(LIBSKYSENSE-SRCS) \
	      skyclient-cmd.tab.o skyclient-cmd.lex.o
	$(CC) $(CFLAGS) $(LFLAGS) -o $@ $^ $(LIBS)

clean:
	$(RM) $(BINS) *.o *~ \
		skyclient-cmd.lex.c skyclient-cmd.tab.* \
		skyclient-cmd.h skyclient-cmd.l skyclient-cmd.y \
		skyserver-cmd.lex.c skyserver-cmd.tab.* \
		skyserver-cmd.h skyserver-cmd.l skyserver-cmd.y
