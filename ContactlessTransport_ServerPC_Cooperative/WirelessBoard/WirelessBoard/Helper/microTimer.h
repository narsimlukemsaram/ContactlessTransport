#pragma once
#include <Windows.h>

//class microTimer {
//public:
namespace microTimer {
	static DWORD uGetTime(DWORD baseTime = 0) {
		LARGE_INTEGER nFreq, currentTime;
		DWORD dwTime;

		QueryPerformanceFrequency(&nFreq);
		QueryPerformanceCounter(&currentTime);
		dwTime = (DWORD)(currentTime.QuadPart * 1000000 / nFreq.QuadPart);

		return dwTime - baseTime;
	}

	static void uWait(DWORD waitTime) {
		DWORD currentTime = uGetTime();
		while (waitTime > uGetTime(currentTime)) { ; }
	}

	static void keepUpdatePeriod(DWORD updatePeriod) {
		static DWORD prevTime = 0;
		DWORD currentTime = microTimer::uGetTime();
		while (currentTime - prevTime < updatePeriod) {
			currentTime = microTimer::uGetTime();
		}
		prevTime = currentTime;
	}
};