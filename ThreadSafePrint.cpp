#include <windows.h>
#include "stdafx.h"

#include "ThreadSafePrint.h"

HANDLE ghConsoleMutex;

int createConsoleMutex()
{
    ghConsoleMutex = CreateMutex( NULL,         // default security attributes
                                  FALSE,        // initially not owned
                                  NULL);        // unnamed mutex

    if (ghConsoleMutex == NULL) {
        printf("CreateMutex error: %d\n", GetLastError());
        return 1;
    }

    return 0;
}
