##
##  Mac OSX build for kdtree
##
##  Note: When creating the shared libraries, errors like
##
##    Undefined symbols for architecture i386:
##      "__Znwm", referenced from:
##      __ZN6KDTree11unserializeEPv in kdtree.o
##
##  may occur. This is the result of the libraries -lstdc++.6 -lgcc_s_x86_64.1
##  being inappropriate for your system. If errors occur, trawl though 
##  /usr/lib and look for similar libraries. I know this sounds vague
##  but there will be something like libstdc++ and libgcc on your mac.
##
##    ~ Mayhem
##

# This may have to be changed according to your MATLAB directory
MATDIR = /Applications/MATLAB_R2009a.app

# These may have to be changed depeing on the MATLAB version
# Older versions of matlab may have:
MEX_INCDIR = -Isrc -I$(MATDIR)/extern/include
MEX_LIBDIR = -L$(MATDIR)/bin/maci
MEX_LIBS = -lmat -lmex -lmx -lm

MEX_SUFFIX = mexmaci


CXX = g++
CXXFLAGS = -Wall -g -fomit-frame-pointer \
						-arch i386 -mtune=i386 -msse -msse2 -fPIC

CPP_LIBS = -lstdc++.6 -lgcc_s_x86_64.1 -lc

################################################################
# No changes should need to be made below this line


SRC = kdtree.cpp kdtree_create.cpp kdtree_closestpoint.cpp kdtree_range.cpp
OBJ = ${SRC:.cpp=.o}
TARGETS = kdtree kdtree_closestpoint kdtree_range


all : purge all_source kdtree kdtree_closestpoint kdtree_range

purge:
	rm -rf @kdtree
	mkdir @kdtree

kdtree: kdtree_create.o kdtree.o
	libtool -dynamic -lSystemStubs -o @kdtree/kdtree.${MEX_SUFFIX} kdtree.o kdtree_create.o ${MEX_LIBDIR} ${MEX_LIBS} ${CPP_LIBS}

kdtree_closestpoint: kdtree_closestpoint.o kdtree.o
	libtool -dynamic -lSystemStubs -o @kdtree/kdtree_closestpoint.${MEX_SUFFIX} kdtree.o kdtree_closestpoint.o ${MEX_LIBDIR} ${MEX_LIBS} ${CPP_LIBS}

kdtree_range: kdtree_range.o kdtree.o
	libtool -dynamic -lSystemStubs -o @kdtree/kdtree_range.${MEX_SUFFIX} kdtree.o kdtree_range.o ${MEX_LIBDIR} ${MEX_LIBS} ${CPP_LIBS}
	rm ${OBJ}

all_source:
	for a in ${SRC} ; do \
		$(CXX) $(CXXFLAGS) $(MEX_INCDIR) -c src/$$a; \
	done

clean:
	rm -r @kdtree
