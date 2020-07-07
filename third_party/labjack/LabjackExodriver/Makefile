#
# Makefile for liblabjackusb
#
#

UNAME = $(shell uname -s)

VERSION = 2.5.3
PREFIX ?= /usr/local
DESTINATION = $(PREFIX)/lib
HEADER = labjackusb.h
HEADER_DESTINATION = $(PREFIX)/include
LIBFLAGS = -lusb-1.0 -lc
ADD_LDCONFIG_PATH = ./add_ldconfig_path.sh

ifeq ($(UNAME),Darwin)
	#Mac OS X operating system macros
	TARGET = liblabjackusb.dylib

	# Build for only the host architecture
	#ARCHFLAGS =

	# Build for 32- and 64-bit Intel architectures
	#ARCHFLAGS = -arch i386 -arch x86_64

	# Build for multiple architectures
	#ARCHFLAGS = -arch i386 -arch x86_64 -arch ppc

	COMPILE = libtool -dynamic -o $(TARGET) -install_name $(TARGET) -current_version $(VERSION) -compatibility_version $(VERSION) labjackusb.o $(LIBFLAGS)
	RUN_LDCONFIG ?= 0
else
	#Linux operating system macros
	TARGET = liblabjackusb.so.$(VERSION)

	# Build for only the host architecture
	#ARCHFLAGS =

	COMPILE = $(CC) -shared -Wl,-soname,liblabjackusb.so -o $(TARGET) labjackusb.o $(LIBFLAGS)
	RUN_LDCONFIG ?= 1
endif

CFLAGS += -fPIC -g -Wall $(ARCHFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $<

all: $(TARGET)

$(TARGET): labjackusb.o $(HEADER)
	$(COMPILE)

install: $(TARGET)
	test -z $(DESTINATION) || mkdir -p $(DESTINATION)
	install $(TARGET) $(DESTINATION)
	test -z $(HEADER_DESTINATION) || mkdir -p $(HEADER_DESTINATION)
	install $(HEADER) $(HEADER_DESTINATION)
ifeq ($(RUN_LDCONFIG),1)
	@$(ADD_LDCONFIG_PATH)
	ldconfig
endif

clean:
	rm -f $(TARGET) *.o *~
