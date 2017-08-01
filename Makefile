CC = $(CROSS_COMPILE)gcc
DEFINES = -D_GNU_SOURCE
CFLAGS = -g -O2 -Wall -Werror $(DEFINES)
YACC := bison
LEX  := flex

SRCS := $(wildcard *.c)
BINS := skyserver skyclient

all: $(BINS)

skyserver: skyserver.o libskysense.o libskysense-loc.o libskysense-rem.o
	$(CC) $(LFLAGS) -o $@ $^ -lzmq -lpthread

docopt-gen:
	$(MAKE) -C docopt
	cp docopt/docopt docopt-gen
	$(MAKE) -C docopt clean

skyclient-cmd.h skyclient-cmd.y skyclient-cmd.l: docopt-gen skyclient-cmd.docopt
	$(RM) skyclient-cmd.h skyclient-cmd.l skyclient-cmd.y
	./docopt-gen skyclient-cmd.docopt

skyclient-cmd.tab.c: skyclient-cmd.y
	$(YACC) -o $@ --defines skyclient-cmd.y

skyclient-cmd.lex.c: skyclient-cmd.l
	$(LEX) -o $@ skyclient-cmd.l

skyclient.o: skyclient-cmd.h

skyclient: skyclient.o libskysense.o libskysense-loc.o libskysense-rem.o \
	   skyclient-cmd.tab.o skyclient-cmd.lex.o
	$(CC) $(CFLAGS) $(LFLAGS) -o $@ $^ -lzmq -lpthread

clean:
	$(RM) $(BINS) *.o *~ \
		skyclient-cmd.lex.c skyclient-cmd.tab.* \
		skyclient-cmd.h skyclient-cmd.l skyclient-cmd.y
