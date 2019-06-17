CURDIR=$(PWD)
CARLADIR=$(CURDIR)/../../
BINDIR=$(CURDIR)/bin
INSTALLDIR=$(CARLADIR)

CC=/usr/bin/gcc
CXX=/usr/bin/g++
CXXFLAGS=-std=c++14 -pthread -fPIC -O3 -DNDEBUG

define log
	@echo "\033[1;35m$(1)\033[0m"
endef

default: build


run: build
	$(call log,Running C++ Client...)
	@$(BINDIR)/cpp_client $(ARGS)

build: $(BINDIR)/cpp_client

$(BINDIR)/cpp_client:
	$(call log,Compiling C++ Client...)
	@mkdir -p $(BINDIR)
	@$(CXX) $(CXXFLAGS) -I$(INSTALLDIR)/include/lib -L$(INSTALLDIR)/lib \
		-o $(BINDIR)/cpp_client main.cpp \
		-Wl,-Bstatic -lcarla_client -lrpc -lboost_filesystem -Wl,-Bdynamic \
		-lpng -ltiff -ljpeg
