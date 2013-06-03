default: installed

TARBALL = build/srl-people-tracker-0.2.tgz
TARBALL_URL = http://othello.informatik.uni-freiburg.de/~luber/html/people_tracker/code/srl-people-tracker-0.2.tgz
SOURCE_DIR=build/srl_source
UNPACK_CMD =  tar xzf
TARBALL_PATCH= CMakeLists.patch 
INITIAL_DIR=build/srl-people-tracker-0.2
include $(shell rospack find mk)/download_unpack_build.mk

EXTRA_CMAKE_FLAGS=-DCMAKE_INSTALL_PREFIX=`rospack find srl_people_tracker_0.2`/install -DCMAKE_PREFIX_PATH=`rospack find eigen`

include $(shell rospack find mk)/cmake.mk

downloaded: ${SOURCE_DIR}/unpacked

installed: downloaded all
	cd build;make 
	touch installed

wipe: clean
	rm -r install lib bin
	rm installed
	rm ${TARBALL}

