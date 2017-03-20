#include "PixyCam.h"

//#define PIXI_SERIAL
#define PIXI_SPI

PixyCam::PixyCam() {

	bBlockFound = false;
	fCentroid = 0.0;

	pTask = new std::thread(&PixyCam::StartTask, this, PIXI_TASKNAME, PIXI_PRIORITY);
}

/*
0, 1     0              sync (0xaa55)
---
0, 1     0              sync (0xaa55)
2, 3     1              checksum (sum of all 16-bit words 2-6)
4, 5     2              signature number
6, 7     3              x center of object
8, 9     4              y center of object
10, 11   5              width of object
12, 13   6              height of object

*/

void PixyCam::Run(void)
{
	 uint8_t uPixiData[2];
	 uint16_t uPixiWord;
	 uint16_t uBlockByteCount = 0;
	 PIXICOM_STATES ePixyComState = PIXYCOM_UNSYNCHED;
#ifdef PIXI_SERIAL
	 SerialPort* pCamera;

	 pCamera = new SerialPort(19200, SerialPort::kOnboard, 8,
			 SerialPort::kParity_None, SerialPort::kStopBits_One);

#endif

#ifdef PIXI_SPI
	SPI* pCamera;

	pCamera = new SPI(SPI::kOnboardCS0);
	pCamera->SetMSBFirst();
	pCamera->SetSampleDataOnRising();
	pCamera->SetClockActiveHigh();
	pCamera->SetClockRate(500000);

#endif
	 while(true){
		// TODO this is a lot of data, do we need it?  fewer max blocks?
     	//TODO do the math, is this fast enough?

		 // wait for there to be 2 bytes in the buffer

#ifdef PIXI_SERIAL
		 while(pCamera->GetBytesReceived() < 2)
		 {
			 Wait(0.005);
		 }

		 pCamera->Read((char *)&uPixiData, 2);
#endif

#ifdef PIXI_SPI
		 pCamera->Read(false, uPixiData, 2);
#endif
		 uPixiWord = (((uint16_t)uPixiData[0] << 8) & 0xFF00) | ((uint16_t)uPixiData[1] & 0x00FF);   // convert to big endian

		 //printf("data = %04X state %d\n", uPixiWord, ePixyComState);

		 switch (ePixyComState)
		 {
			 case  PIXYCOM_UNSYNCHED:
				 // look for synch word

				 if(uPixiWord == PIXICOM_FRAMESYNCWORD)
				 {
					 ePixyComState = PIXYCOM_LOOK4SECONDSYNCWORD;
				 }
				 else if(uPixiWord == 0)
				 {
					 {
						//std::lock_guard<priority_recursive_mutex> sync(pInstance->mutexData);
						bBlockFound = false;
						fCentroid = 0.0;
					 }

					 Wait(0.0005);
				 }
				 break;

			 case  PIXYCOM_LOOK4SECONDSYNCWORD:
				 // look for second consecutive synch word

				 //TODO look for color sync word as well?

				 if(uPixiWord == PIXICOM_FRAMESYNCWORD)
				 {
					 uBlockByteCount = 0;
					 iCentroid1 = 0;
					 iCentroid2 = 0;
					 ePixyComState = PIXYCOM_GETBLOCKDATA;
				 }
				 else if(uPixiWord == 0)
				 {
					 {
						//std::lock_guard<priority_recursive_mutex> sync(pInstance->mutexData);
						bBlockFound = false;
						fCentroid = 0.0;
					 }
					 ePixyComState = PIXYCOM_UNSYNCHED;
				 }
				 else
				 {
					 // keep looking for a new frame

					 ePixyComState = PIXYCOM_UNSYNCHED;
				 }
				 break;

			 case  PIXYCOM_STARTOFRAME:
				 ePixyComState = PIXYCOM_STARTOFBLOCK;
				 uBlockByteCount = 0;
				 break;

			 case  PIXYCOM_STARTOFBLOCK:
				 ePixyComState = PIXYCOM_GETBLOCKDATA;
				 uBlockByteCount = 0;
				 break;

			 case  PIXYCOM_GETBLOCKDATA:
				 uCurrentBlock[uBlockByteCount++] = uPixiWord;  // block data

				 if(uBlockByteCount >= 6)
				 {
					 ePixyComState = PIXYCOM_ENDOFBLOCK;
				 }
				 break;

			 case  PIXYCOM_ENDOFBLOCK:
				 if(uPixiWord == PIXICOM_FRAMESYNCWORD)
				 {
					 // is this the last block?
					 uBlockByteCount = 0;
					 ePixyComState = PIXYCOM_ENDOFFRAME;

					 // calibrate on known image, verify x,y,w,h make sense
					 // TODO verify checksum
					 // TODO should we only follow the largest match?
					 // TODO convert x and y to % of full view? - what would help servo math the best
					 // TODO is the first signature labeled 0 or 1?

					//std::lock_guard<priority_recursive_mutex> sync(pInstance->mutexData);

					if(uCurrentBlock[1] == 1)
					{
						// centroid of the largest block (the first signature)

						iCentroid1 = (int)uCurrentBlock[2] - 160 / 160;
					}
					else if(uCurrentBlock[1] == 2)
					{
						// centroid of the second largest block (the first signature)

						bBlockFound = true;
						iCentroid2 = (int)uCurrentBlock[2] - 160 / 160;
						fCentroid = (float)(iCentroid1 + (iCentroid2 - iCentroid1) / 2.0);
					}
					else
					{
						// ignore other blocks, we are looking for 2 pieces of tape
					}

					 //printf("%d: x = %u, y = %u, w = %u, h = %u\n",
					 //		 pInstance->uCurrentBlock[1],
					 //		 pInstance->uCurrentBlock[2],
					 //		 pInstance->uCurrentBlock[3],
					 //		 pInstance->uCurrentBlock[4],
					 //		 pInstance->uCurrentBlock[5]);
				 }
				 else
				 {
					 ePixyComState = PIXYCOM_UNSYNCHED;
				 }
				 break;

			 case  PIXYCOM_ENDOFFRAME:
				 if(uPixiWord == PIXICOM_FRAMESYNCWORD)
				 {
					 // new frame and new block

					 uBlockByteCount = 0;
					 ePixyComState = PIXYCOM_GETBLOCKDATA;
				 }
				 else if(uPixiWord == 0)
				 {
					 // new frame but there are no queued objects

					 {
						//std::lock_guard<priority_recursive_mutex> sync(pInstance->mutexData);
						bBlockFound = false;
						fCentroid = 0.0;
					 }
					 ePixyComState = PIXYCOM_UNSYNCHED;
				 }
				 else
				 {
					 // new block, eat the first word

					 uCurrentBlock[0] = uPixiWord;
					 uBlockByteCount = 1;
					 ePixyComState = PIXYCOM_GETBLOCKDATA;
				 }
				 break;

			 default:
				 // should never get here!
				 ePixyComState = PIXYCOM_UNSYNCHED;
				 break;
		 }
	 }
}

bool PixyCam::GetCentroid(float &fNewCentroid)
{
	//std::lock_guard<priority_recursive_mutex> sync(mutexData);
	fNewCentroid = fCentroid;
	return(bBlockFound);
}

double PixyCam::PIDGet(){
	//std::lock_guard<priority_recursive_mutex> sync(mutexData);
	SmartDashboard::PutNumber("fCentroid", fCentroid);
	return fCentroid;
}


PixyCam::~PixyCam(){
	delete pTask;
}





