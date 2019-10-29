// This is a couple of macros used to write output and error to console i a threadsafe manner.

#ifndef _THREAD_SAFE_PRINT_H_
#define _THREAD_SAFE_PRINT_H_

#include <windows.h>
#include <iostream>

extern HANDLE ghConsoleMutex;

#define safeCout(stream) { \
    WaitForSingleObject(ghConsoleMutex, INFINITE); \
    std::cout << stream << std::endl; \
    ReleaseMutex(ghConsoleMutex); }

#define safeCerr(stream) { \
    WaitForSingleObject(ghConsoleMutex, INFINITE); \
    std::cerr << stream; \
    ReleaseMutex(ghConsoleMutex); }

int createConsoleMutex();

#endif
