SHELL       := /bin/bash
MAKEFLAGS   += --no-print-directory

.SILENT     : init build clean test gendoc
.PHONY      : test

# Define some directory paths
CATKIN_WS   := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))/../..
PACKAGE     := ${CATKIN_WS}/src/mosaic_gnss_driver

# All these will be deleted when clean is invoked
TRASH_DIRS  := ${CATKIN_WS}/build ${CATKIN_WS}/devel ${PACKAGE}/docs/html ${PACKAGE}/docs/latex
TRASH_FILES :=

define banner
	@echo -e "************************************************************\n"
	@echo -e "\tMosaic GNSS ROS Driver : $(1)\n"
	@echo -e "************************************************************\n"
endef

init:
	$(call banner,Help)
	echo -e "Available commands: \n"
	echo "make build  : Build workspace"
	echo "make test   : Run all unit tests"
	echo "make clean  : Cleanup workspace"
	echo "make gendoc : Generate documentation (Doxygen)"
	echo ""

# Build catkin workspace
build:
	$(call banner,Build)
	source /opt/ros/melodic/setup.bash && \
	cd ${CATKIN_WS} && \
	catkin_make

# Run unit tests (only if rosmaster in online)
test:
	$(call banner,Run unit tests)
	source /opt/ros/melodic/setup.bash && \
	cd ${CATKIN_WS} && \
	catkin_make && \
	if rostopic list > /dev/null 2>&1; then \
		cd ${CATKIN_WS} && catkin_make run_tests; \
	else \
		echo -e "\nOops! Master is offline\n"; \
	fi

# Clean workspace
clean:
	$(call banner,Cleanup)
	# Check if directory exists, if yes, delete
	for dir in ${TRASH_DIRS}; do \
		if [[ -d $$dir ]]; then \
			rm -rf $$dir; \
			echo "Deleted : $$dir"; \
		fi \
	done
	# Check if file exists, if yes, delete
	for file in ${TRASH_FILES}; do \
		if [[ -f $$file ]]; then \
			rm -rf $$file; \
			echo "Deleted : $$dir"; \
		fi \
	done

# Generate documentation using Doxygen
gendoc:
	$(call banner,Generate documentation)
	cd ${PACKAGE}/docs && \
	doxygen mosaic.dconfig && \
	cd latex && \
	$(MAKE)
