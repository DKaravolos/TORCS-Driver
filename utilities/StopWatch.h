#include <Windows.h>

class CStopWatch
{
public:
    // Constructor.
    CStopWatch()
    {
        // Ticks per second.
        QueryPerformanceFrequency( &liPerfFreq );
    }

    // Start counter.
    inline void Start()
    {
        liStart.QuadPart = 0;
        QueryPerformanceCounter( &liStart );
    }

    // Stop counter.
    inline void Stop()
    {
        liEnd.QuadPart = 0;
        QueryPerformanceCounter( &liEnd );
    }

    // Get duration.
    inline long double GetDuration()
    {
        return ( liEnd.QuadPart - liStart.QuadPart) /
                long double( liPerfFreq.QuadPart );
    }

private:
    LARGE_INTEGER liStart;
    LARGE_INTEGER liEnd;
    LARGE_INTEGER liPerfFreq;
};