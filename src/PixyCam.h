/*
 * PixyCam.h
 *
 *  Created on: Jan 14, 2016
 *      Author: fmj
 */

#ifndef SRC_PIXYCAM_H_
#define SRC_PIXYCAM_H_

#include <pthread.h>
#include <string>

//Robot
#include <WPILib.h>
#include "RobotParams.h"

const uint16_t PIXICOM_FRAMESYNCWORD = 0xAA55;

typedef enum PIXICOM_STATES {
	PIXYCOM_UNSYNCHED,
	PIXYCOM_LOOK4SECONDSYNCWORD,
	PIXYCOM_STARTOFRAME,
	PIXYCOM_STARTOFBLOCK,
	PIXYCOM_GETBLOCKDATA,
	PIXYCOM_ENDOFBLOCK,
	PIXYCOM_ENDOFFRAME,
	PIXYCOM_LAST
} PIXICOM_STATES;

class PixyCam : public PIDSource{
	struct Block{
		uint16_t checksum;
		uint16_t signature;
		uint16_t x; // Center
		uint16_t y; // Center
		uint16_t width;
		uint16_t height;
	};

public:

	PixyCam();
	~PixyCam();

	static void *StartTask(void *pThis, const char* szComponentName, int iPriority)
	{
		pthread_setname_np(pthread_self(), szComponentName);
		pthread_setschedprio(pthread_self(), iPriority);
		((PixyCam *)pThis)->Run();
		return(NULL);
	}

	void Run(void);
	double PIDGet(void);
	bool GetCentroid(float &fNewCentroid);   // -1.0 to 1.0
private:
 	std::thread* pTask;
    uint16_t uCurrentBlock[12];
    uint16_t uCommands[12];
	bool bBlockFound;
	float fCentroid;
	int iCentroid1;
	int iCentroid2;
	mutable priority_recursive_mutex mutexData;
};


#endif // SRC_PIXYCAM_H_





