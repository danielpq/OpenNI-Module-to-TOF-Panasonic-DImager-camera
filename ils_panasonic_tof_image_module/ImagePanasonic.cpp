/*****************************************************************************
*                                                                            *
*  OpenNI 1.0 Alpha                                                          *
*  Copyright (C) 2010 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  OpenNI is free software: you can redistribute it and/or modify            *
*  it under the terms of the GNU Lesser General Public License as published  *
*  by the Free Software Foundation, either version 3 of the License, or      *
*  (at your option) any later version.                                       *
*                                                                            *
*  OpenNI is distributed in the hope that it will be useful,                 *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              *
*  GNU Lesser General Public License for more details.                       *
*                                                                            *
*  You should have received a copy of the GNU Lesser General Public License  *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.            *
*                                                                            *
*****************************************************************************/


//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "ImagePanasonic.h"


#define SUPPORTED_X_RES 640
#define SUPPORTED_Y_RES 480
#define SUPPORTED_FPS 30

ImagePanasonic::ImagePanasonic() : 
	m_bGenerating(FALSE),
	m_bDataAvailable(FALSE),
	m_pDepthMap(NULL),
	m_nFrameID(0),
	m_nTimestamp(0),
	m_hScheduler(NULL),
	m_bMirror(FALSE)
{
}

ImagePanasonic::~ImagePanasonic()
{
	delete[] m_pDepthMap;

	// Adormece a câmera
	ChangeSleep (0);
	
	// Libera o driver
	FreeImageDriver();

	free (kdat);
	free (ndat);
}

XnStatus ImagePanasonic::Init()
{
	m_pDepthMap = new XnUInt8[SUPPORTED_X_RES * SUPPORTED_Y_RES];
	if (m_pDepthMap == NULL)
	{
		return XN_STATUS_ALLOC_FAILED;
	}

	// Inicializa a câmera
	if  (InitImageDriver() != 0) {
		return XN_STATUS_DEVICE_NOT_CONNECTED;
	}

	// Define a taxa de captura
	if (Speedmode () != SUPPORTED_FPS){
		ChangeSpeed (SUPPORTED_FPS);
	}

	// Define a frequência da luz (evitar interferênia com outras)
	if (Freqmode () != 1){
		ChangeFreq (1);
	}

	// Acorda a câmera
	if (Sleepmode () == 0){
		ChangeSleep (1);
	}

	// Aloca memória para os vetores retornados pela câmera (160*120*2 bytes = 38400 bytes)
	kdat = (unsigned short*) malloc(38400);
	ndat = (unsigned short*) malloc(38400);

	return (XN_STATUS_OK);
}

XnBool ImagePanasonic::IsCapabilitySupported( const XnChar* strCapabilityName )
{
	// we only support the mirror capability
	return (strcmp(strCapabilityName, XN_CAPABILITY_MIRROR) == 0);
}

XnStatus ImagePanasonic::StartGenerating()
{
	XnStatus nRetVal = XN_STATUS_OK;
	
	m_bGenerating = TRUE;

	// start scheduler thread
	nRetVal = xnOSCreateThread(SchedulerThread, this, &m_hScheduler);
	if (nRetVal != XN_STATUS_OK)
	{
		m_bGenerating = FALSE;
		return (nRetVal);
	}

	m_generatingEvent.Raise();

	// Inicializa a câmera
	if  (InitImageDriver() != 0) {
		return XN_STATUS_DEVICE_NOT_CONNECTED;
	}

		// Define a frequência da luz (evitar interferênia com outras)
	if (Freqmode () != 1){
		ChangeFreq (1);
	}

	// Acorda a câmera
	if (Sleepmode () == 0){
		ChangeSleep (1);
	}

	return (XN_STATUS_OK);
}

XnBool ImagePanasonic::IsGenerating()
{
	return m_bGenerating;
}

void ImagePanasonic::StopGenerating()
{
	m_bGenerating = FALSE;

	// wait for thread to exit
	xnOSWaitForThreadExit(m_hScheduler, 100);

	m_generatingEvent.Raise();

	// Adormece a câmera
	ChangeSleep (0);
	
	// Libera o driver
	FreeImageDriver();
}

XnStatus ImagePanasonic::RegisterToGenerationRunningChange( XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback )
{
	return m_generatingEvent.Register(handler, pCookie, &hCallback);
}

void ImagePanasonic::UnregisterFromGenerationRunningChange( XnCallbackHandle hCallback )
{
	m_generatingEvent.Unregister(hCallback);
}

XnStatus ImagePanasonic::RegisterToNewDataAvailable( XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback )
{
	return m_dataAvailableEvent.Register(handler, pCookie, &hCallback);
}

void ImagePanasonic::UnregisterFromNewDataAvailable( XnCallbackHandle hCallback )
{
	m_dataAvailableEvent.Unregister(hCallback);
}

XnBool ImagePanasonic::IsNewDataAvailable( XnUInt64& nTimestamp )
{
	return m_bDataAvailable;
}

XnStatus ImagePanasonic::UpdateData()
{
	XnStatus nRetVal = XN_STATUS_OK;
	
	XnUInt8* pPixel = m_pDepthMap;

	GetImageKN(kdat, ndat);

	// change our internal data, so that pixels go from frameID incrementally in both axes.
	for (XnUInt y = 0; y < SUPPORTED_Y_RES; ++y)
	{
		for (XnUInt x = 0; x < SUPPORTED_X_RES; ++x, ++pPixel)
		{
			//*pPixel = (m_nFrameID + x + y) % MAX_DEPTH_VALUE;
			*pPixel = (XnUInt8)ndat[y/4*(SUPPORTED_X_RES/4) + x/4]; // Não há perda de dados porque a imagem só utiliza 8 bits dos 16 do short
		}
	}

	// if needed, mirror the map
	if (m_bMirror)
	{
		XnUInt8 temp;

		for (XnUInt y = 0; y < SUPPORTED_Y_RES; ++y)
		{
			XnUInt8* pUp = &m_pDepthMap[y * SUPPORTED_X_RES];
			XnUInt8* pDown = &m_pDepthMap[(y+1) * SUPPORTED_X_RES - 1];

			for (XnUInt x = 0; x < SUPPORTED_X_RES/2; ++x, ++pUp, --pDown)
			{
				temp = *pUp;
				*pUp = *pDown;
				*pDown = temp;
			}
		}
	}

	m_nFrameID++;
	m_nTimestamp += 1000000 / SUPPORTED_FPS;

	// mark that data is old
	m_bDataAvailable = FALSE;
	
	return (XN_STATUS_OK);
}

XnUInt32 ImagePanasonic::GetDataSize()
{
	return (SUPPORTED_X_RES * SUPPORTED_Y_RES * sizeof(XnDepthPixel));
}

XnUInt64 ImagePanasonic::GetTimestamp()
{
	return m_nTimestamp;
}

XnUInt32 ImagePanasonic::GetFrameID()
{
	return m_nFrameID;
}

xn::ModuleMirrorInterface* ImagePanasonic::GetMirrorInterface()
{
	return this;
}

XnStatus ImagePanasonic::SetMirror( XnBool bMirror )
{
	m_bMirror = bMirror;
	m_mirrorEvent.Raise();
	return (XN_STATUS_OK);
}

XnBool ImagePanasonic::IsMirrored()
{
	return m_bMirror;
}

XnStatus ImagePanasonic::RegisterToMirrorChange( XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback )
{
	return m_mirrorEvent.Register(handler, pCookie, &hCallback);
}

void ImagePanasonic::UnregisterFromMirrorChange( XnCallbackHandle hCallback )
{
	m_mirrorEvent.Unregister(hCallback);
}

XnUInt32 ImagePanasonic::GetSupportedMapOutputModesCount()
{
	// we only support a single mode
	return 1;
}

XnStatus ImagePanasonic::GetSupportedMapOutputModes( XnMapOutputMode aModes[], XnUInt32& nCount )
{
	if (nCount < 1)
	{
		return XN_STATUS_OUTPUT_BUFFER_OVERFLOW;
	}

	aModes[0].nXRes = SUPPORTED_X_RES;
	aModes[0].nYRes = SUPPORTED_Y_RES;
	aModes[0].nFPS = SUPPORTED_FPS;

	return (XN_STATUS_OK);
}

XnStatus ImagePanasonic::SetMapOutputMode( const XnMapOutputMode& Mode )
{
	// make sure this is our supported mode
	if (Mode.nXRes != SUPPORTED_X_RES ||
		Mode.nYRes != SUPPORTED_Y_RES ||
		Mode.nFPS != SUPPORTED_FPS)
	{
		return (XN_STATUS_BAD_PARAM);
	}

	return (XN_STATUS_OK);
}

XnStatus ImagePanasonic::GetMapOutputMode( XnMapOutputMode& Mode )
{
	Mode.nXRes = SUPPORTED_X_RES;
	Mode.nYRes = SUPPORTED_Y_RES;
	Mode.nFPS = SUPPORTED_FPS;

	return (XN_STATUS_OK);
}

XnStatus ImagePanasonic::RegisterToMapOutputModeChange( XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback )
{
	// no need. we only allow one mode
	hCallback = this;
	return XN_STATUS_OK;
}

void ImagePanasonic::UnregisterFromMapOutputModeChange( XnCallbackHandle hCallback )
{
	// do nothing (we didn't really register)	
}

XnUInt8* ImagePanasonic::GetImageMap()
{
	return m_pDepthMap;
}

/*XnDepthPixel ImagePanasonic::GetDeviceMaxDepth()
{
	return MAX_DEPTH_VALUE;
}*/

void ImagePanasonic::GetFieldOfView( XnFieldOfView& FOV )
{
	// some numbers
	FOV.fHFOV = 1.35;
	FOV.fVFOV = 1.35;
}

XnStatus ImagePanasonic::RegisterToPixelFormatChange( XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback )
{
	// no need. it never changes
	hCallback = this;
	return XN_STATUS_OK;
}

void ImagePanasonic::UnregisterFromPixelFormatChange( XnCallbackHandle hCallback )
{
	// do nothing (we didn't really register)	
}

XN_THREAD_PROC ImagePanasonic::SchedulerThread( void* pCookie )
{
	ImagePanasonic* pThis = (ImagePanasonic*)pCookie;

	while (pThis->m_bGenerating)
	{
		// wait 33 ms (to produce 30 FPS)
		xnOSSleep(1000000/SUPPORTED_FPS/1000);

		pThis->OnNewFrame();
	}

	XN_THREAD_PROC_RETURN(0);
}

void ImagePanasonic::OnNewFrame()
{
	m_bDataAvailable = TRUE;
	m_dataAvailableEvent.Raise();
}


XnBool ImagePanasonic::IsPixelFormatSupported(XnPixelFormat Format){

	// Não faz nada
	return (true);
}

XnStatus ImagePanasonic::SetPixelFormat(XnPixelFormat Format){
	// Não faz nada
	return XN_STATUS_OK;
}
		
XnPixelFormat ImagePanasonic::GetPixelFormat(){

	// Imagem em Gray Scale
	return (XN_PIXEL_FORMAT_GRAYSCALE_8_BIT);
}