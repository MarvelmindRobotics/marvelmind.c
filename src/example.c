#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#if defined(WIN32) || defined(_WIN64)
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <semaphore.h>
#include <time.h>
#endif
#include "marvelmind.h"

bool terminateProgram=false;

#if defined(WIN32) || defined(_WIN64)
BOOL CtrlHandler( DWORD fdwCtrlType )
{
    if ((fdwCtrlType==CTRL_C_EVENT ||
            fdwCtrlType==CTRL_BREAK_EVENT ||
            fdwCtrlType==CTRL_CLOSE_EVENT ||
            fdwCtrlType==CTRL_LOGOFF_EVENT ||
            fdwCtrlType==CTRL_SHUTDOWN_EVENT) &&
            (terminateProgram==false))
    {
        terminateProgram=true;
        return true;
    }
    else return false;
}
#else
void CtrlHandler(int signum)
{
    terminateProgram=true;
}
#endif

#if defined(WIN32) || defined(_WIN64)
void sleep(unsigned int seconds)
{
    Sleep (seconds*1000);
}
#endif


#if defined(WIN32) || defined(_WIN64)
HANDLE ghSemaphore;
DWORD dwSemWaitResult;
void semCallback()
{
    ReleaseSemaphore(
        ghSemaphore,  // handle to semaphore
        1,            // increase count by one
        NULL);
}
#else
// Linux
static sem_t *sem;
struct timespec ts;
void semCallback()
{
	sem_post(sem);
}
#endif // WIN32

#ifdef _WIN64
const wchar_t* GetWC(const char* c)
{
    const size_t cSize = strlen(c) + 1;
    wchar_t* wc =  malloc(sizeof(wchar_t)*cSize);
    mbstowcs(wc, c, cSize);

    return wc;
}
#endif

int main (int argc, char *argv[])
{
    // get port name from command line arguments (if specified)
    #ifdef _WIN64
    const wchar_t * ttyFileName;
    if (argc == 2) ttyFileName = GetWC(argv[1]);
    else ttyFileName = TEXT(DEFAULT_TTY_FILENAME);
    #else
    const char * ttyFileName;
    if (argc == 2) ttyFileName = argv[1];
    else ttyFileName = DEFAULT_TTY_FILENAME;
    #endif

    // Init
    struct MarvelmindHedge * hedge=createMarvelmindHedge ();
    if (hedge==NULL)
    {
        puts ("Error: Unable to create MarvelmindHedge");
        return -1;
    }
    hedge->ttyFileName=ttyFileName;
    hedge->verbose=true; // show errors and warnings
    hedge->anyInputPacketCallback= semCallback;
    startMarvelmindHedge (hedge);

    // Set Ctrl-C handler
#if defined(WIN32) || defined(_WIN64)
    SetConsoleCtrlHandler( (PHANDLER_ROUTINE) CtrlHandler, TRUE );
#else
    signal (SIGINT, CtrlHandler);
    signal (SIGQUIT, CtrlHandler);
#endif

#if defined(WIN32) || defined(_WIN64)
    ghSemaphore = CreateSemaphore(
        NULL, // default security attributes
        10,  // initial count
        10,  // maximum count
        NULL);          // unnamed semaphore
    if (ghSemaphore == NULL)
    {
        printf("CreateSemaphore error: %d\n", (int) GetLastError());
        return 1;
    }
#else
	// Linux
	sem = sem_open(DATA_INPUT_SEMAPHORE, O_CREAT, 0777, 0);
#endif

    // Main loop
    while ((!terminateProgram) && (!hedge->terminationRequired))
    {
        //sleep (3);
        #if defined(WIN32) || defined(_WIN64)
        dwSemWaitResult = WaitForSingleObject(
            ghSemaphore,   // handle to semaphore
            1000); // time-out interval
        #else
        // Linux
        if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
		{
			printf("clock_gettime error");
			return -1;
		}
		ts.tv_sec += 2;
		sem_timedwait(sem,&ts);
        #endif


        printPositionFromMarvelmindHedge (hedge, true);

        printStationaryBeaconsPositionsFromMarvelmindHedge (hedge, true);

        printRawDistancesFromMarvelmindHedge(hedge, true);

        printRawIMUFromMarvelmindHedge(hedge, true);

        printFusionIMUFromMarvelmindHedge(hedge, true);

        printTelemetryFromMarvelmindHedge(hedge, true);

        printQualityFromMarvelmindHedge(hedge, true);

        printUserPayloadFromMarvelmindHedge(hedge, true);
    }

    // Exit
    stopMarvelmindHedge (hedge);
    destroyMarvelmindHedge (hedge);
    return 0;
}
