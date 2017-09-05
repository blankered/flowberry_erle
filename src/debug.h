#ifndef DEBUG_H
#define DEBUG_H

#include "common.h"

#ifdef __cplusplus

#include <iostream>

#ifdef DEBUG

#define DBG(msg)	do {		\
	std::cout << "[" << __FILE__ << "] " << msg << std::endl;	\
} while (0);

#else

#define DBG(...)

#endif

#define ERR(msg)	do {		\
	std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] " << msg << std::endl;	\
} while (0);

#else

#include <stdio.h>

#ifdef DEBUG

#define DBG 		printf

#else

#define DBG(...)

#endif

#define ERR(...)	fprintf(stderr, __VA_ARGS__)


#endif

#endif
