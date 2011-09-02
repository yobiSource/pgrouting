SET(LIBRARY_OUTPUT_PATH ../../../lib/)
ADD_LIBRARY(routing_dd SHARED boost_drivedist.cpp drivedist.c drivedist.h)
INSTALL(TARGETS routing_dd DESTINATION ${LIBRARY_INSTALL_PATH})
