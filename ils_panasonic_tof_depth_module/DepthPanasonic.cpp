/*****************************************************************************
*                                                                            *
*  Copyright (C) 2011 Ilusis Interactive Graphics.                           *
*                                                                            *
*  This is afree software: you can redistribute it and/or modify             *
*  it under the terms of the GNU Lesser General Public License as published  *
*  by the Free Software Foundation, either version 3 of the License, or      *
*  (at your option) any later version.                                       *
*                                                                            *
*  This is distributed in the hope that it will be useful,                   *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              *
*  GNU Lesser General Public License for more details.                       *
*                                                                            *
*  You should see a copy of the GNU Lesser General Public License			 *
*  in <http://www.gnu.org/licenses/>.										 *
*                                                                            *
*****************************************************************************/


//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "DepthPanasonic.h"


#define SUPPORTED_X_RES 640
#define SUPPORTED_Y_RES 480
#define SUPPORTED_FPS 30
#define MAX_DEPTH_VALUE	10000


DepthPanasonic::DepthPanasonic() : 
	m_bGenerating(FALSE),
	m_bDataAvailable(FALSE),
	m_pDepthMap(NULL),
	m_nFrameID(0),
	m_nTimestamp(0),
	m_hScheduler(NULL),
	m_bMirror(FALSE)
{
	//all the m_s2dConfig stuff is copied from the XnDDK
	// copied from Kinect
    // see XnShiftToDepthStreamHelper.cpp from XnDDK
    m_s2dConfig.nZeroPlaneDistance = 120; //120 // PEGA
    m_s2dConfig.fZeroPlanePixelSize = 0.10419999f; // PEGA
    m_s2dConfig.fEmitterDCmosDistance = 7.5; //7.5 // 1.0 PEGA
    m_s2dConfig.nDeviceMaxShiftValue = 2047; //2047 // PEGA
    m_s2dConfig.nDeviceMaxDepthValue = MAX_DEPTH_VALUE;   
    m_s2dConfig.nConstShift = 200; //200// 150 PEGA
    m_s2dConfig.nPixelSizeFactor = 1;
    m_s2dConfig.nParamCoeff = 4; //4
    m_s2dConfig.nShiftScale = 10; //10
    m_s2dConfig.nDepthMinCutOff = 0;
    m_s2dConfig.nDepthMaxCutOff = MAX_DEPTH_VALUE;     

	/*
	Hi,

so the depthmap gives back the Z-value for each pixel in mm
(millimeters). Thus if you want to get the full 3D position (X,Y,Z) of a
pixel (u,v), then use following formula:

XnUInt64 F_;
XnDouble pixel_size_;

// get the focal length in mm (ZPS = zero plane distance)
depth_.GetIntProperty ("ZPD", F_)

// get the pixel size in mm ("ZPPS" = pixel size at zero plane)
depth_.GetRealProperty ("ZPPS", pixel_size_)

X = (u - 320) * depth_md_[k] * pixel_size_ * 0.001 / F_;
Y = (v - 240) * depth_md_[k] * pixel_size_ * 0.001 / F_;
Z = depth_md_[k] * 0.001; // from mm in meters!

I hope this helps you,


One thing i forgot to say:
The actual pixel size is twice the pixel size returned by
"depth_.GetRealProperty ("ZPPS", pixel_size_)". This results from the
fact, that the actual sensor is 1280x1024 and the pixel size is given
for that resolution, so you have to multiply by 2. 
	*/
}

DepthPanasonic::~DepthPanasonic()
{
	delete[] m_pDepthMap;

	// Adormece a câmera
	ChangeSleep (0);
	
	// Libera o driver
	FreeImageDriver();

	free (kdat);
	free (ndat);
}

XnStatus DepthPanasonic::Init()
{
	// Aloca o mapa de profundidade
	m_pDepthMap = new XnDepthPixel[SUPPORTED_X_RES * SUPPORTED_Y_RES];

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

	// Inicialização das matrizes que convertem os valores de profundidade para mm (e vice versa)
	XN_VALIDATE_INPUT_PTR(&m_ShiftToDepthTables);
	XN_VALIDATE_INPUT_PTR(&m_s2dConfig);

	XN_VALIDATE_ALIGNED_CALLOC(m_ShiftToDepthTables.pShiftToDepthTable, XnDepthPixel, m_s2dConfig.nDeviceMaxShiftValue+1, XN_DEFAULT_MEM_ALIGN);
	XN_VALIDATE_ALIGNED_CALLOC(m_ShiftToDepthTables.pDepthToShiftTable, XnUInt16, m_s2dConfig.nDeviceMaxDepthValue+1, XN_DEFAULT_MEM_ALIGN);
	m_ShiftToDepthTables.bIsInitialized = TRUE;

	// store allocation sizes
	m_ShiftToDepthTables.nShiftsCount = m_s2dConfig.nDeviceMaxShiftValue + 1;
	m_ShiftToDepthTables.nDepthsCount = m_s2dConfig.nDeviceMaxDepthValue + 1;

	// Gera a matriz
	XnShiftToDepthUpdate(&m_ShiftToDepthTables, &m_s2dConfig);

	return (XN_STATUS_OK);
}


XnStatus DepthPanasonic::UpdateData()
{
	XnStatus nRetVal = XN_STATUS_OK;
		
	XnDepthPixel* pPixel = m_pDepthMap;
	unsigned short kValue = 0, value = 0;
	unsigned int nNumberOfPoints = 0;
	unsigned int nIndex = 0;

	// Recebe os dados da câmera Panasonic
	GetImageKN(kdat, ndat);

	// Passa os dados da câmera para o mapa de profundidade, fazendo uma reescala da imagem (4x)
	for (XnUInt y = 0; y < SUPPORTED_Y_RES; ++y)
	{
		for (XnUInt x = 0; x < SUPPORTED_X_RES; ++x, ++pPixel)
		{
			kValue = kdat[y/4*(SUPPORTED_X_RES/4) + x/4];

			value = m_ShiftToDepthTables.pShiftToDepthTable[kValue];

			*pPixel = value;
		}
	}
	
	// if needed, mirror the map
	if (m_bMirror)
	{
		XnDepthPixel temp;

		for (XnUInt y = 0; y < SUPPORTED_Y_RES; ++y)
		{
			XnDepthPixel* pUp = &m_pDepthMap[y * SUPPORTED_X_RES];
			XnDepthPixel* pDown = &m_pDepthMap[(y+1) * SUPPORTED_X_RES - 1];

			for (XnUInt x = 0; x < SUPPORTED_X_RES/2; ++x, ++pUp, --pDown)
			{
				temp = *pUp;
				*pUp = *pDown;
				*pDown = temp;
			}
		}
	}

	// Incrementa o número do frame, aplicações utilizam um get para obter este valor
	m_nFrameID++;

	// Incrementa o número do Timestamp, aplicações utilizam um get para obter este valor
	m_nTimestamp += 1000000 / SUPPORTED_FPS;

	// Mark that data is old
	m_bDataAvailable = FALSE;
	
	return (XN_STATUS_OK);
}


XnStatus DepthPanasonic::XnShiftToDepthUpdate(XnShiftToDepthTables* pShiftToDepth, const XnShiftToDepthConfig* pConfig)
{
	XN_VALIDATE_INPUT_PTR(pShiftToDepth);
	XN_VALIDATE_INPUT_PTR(pConfig);

	// check max shift wasn't changed (if so, memory should be re-allocated)
	if (pConfig->nDeviceMaxShiftValue > pShiftToDepth->nShiftsCount){
		printf("DepthPanasonic::XnShiftToDepthUpdate: ERRO \n");
		return 198660;
	}

	// check max depth wasn't changed (if so, memory should be re-allocated)
	if (pConfig->nDeviceMaxDepthValue > pShiftToDepth->nDepthsCount){
		printf("DepthPanasonic::XnShiftToDepthUpdate: ERRO \n");
		return 198661;
	}

	XnUInt32 nIndex = 0;
	XnInt16  nShiftValue = 0;
	XnDouble dFixedRefX = 0;
	XnDouble dMetric = 0;
	XnDouble dDepth = 0;
	XnDouble dPlanePixelSize = pConfig->fZeroPlanePixelSize;
	XnDouble dPlaneDsr = pConfig->nZeroPlaneDistance;
	XnDouble dPlaneDcl = pConfig->fEmitterDCmosDistance;
	XnInt32 nConstShift = pConfig->nParamCoeff * pConfig->nConstShift;

	dPlanePixelSize *= pConfig->nPixelSizeFactor;
	nConstShift /= pConfig->nPixelSizeFactor;

	XnDepthPixel* pShiftToDepthTable = pShiftToDepth->pShiftToDepthTable;
	XnUInt16* pDepthToShiftTable = pShiftToDepth->pDepthToShiftTable;

	xnOSMemSet(pShiftToDepthTable, 0, pShiftToDepth->nShiftsCount * sizeof(XnDepthPixel));
	xnOSMemSet(pDepthToShiftTable, 0, pShiftToDepth->nDepthsCount * sizeof(XnUInt16));

	XnUInt16 nLastDepth = 0;
	XnUInt16 nLastIndex = 0;


	for (nIndex = 1; nIndex < pConfig->nDeviceMaxShiftValue; nIndex++)
	{
		nShiftValue = nIndex;
		
		// Essa é a maneira que o Kinect faz a conversão dos seus valores para mm 
		/*dFixedRefX = (XnDouble)(nShiftValue - nConstShift) / (XnDouble)pConfig->nParamCoeff;
		dFixedRefX -= 0.375;
		dMetric = dFixedRefX * dPlanePixelSize;
		dDepth = pConfig->nShiftScale * ((dMetric * dPlaneDsr / (dPlaneDcl - dMetric)) + dPlaneDsr);
		*/

		// Essa é a maneira que a Panasonic faz a conversão dos seus valores para mm 
		// Está utilizando uma função linear, mas o ideal seria modelar a função realmente empregada
		dDepth = 9.23239486*nIndex + 113.2059079;

		// check cut-offs
		if ((dDepth > pConfig->nDepthMinCutOff) && (dDepth < pConfig->nDepthMaxCutOff))
		{
			pShiftToDepthTable[nIndex] = (XnUInt16)dDepth;

			for (XnUInt16 i = nLastDepth; i < dDepth; i++)
				pDepthToShiftTable[i] = nLastIndex;

			nLastIndex = nIndex;
			nLastDepth = (XnUInt16)dDepth;
		}
	}

	for (XnUInt16 i = nLastDepth; i <= pConfig->nDeviceMaxDepthValue; i++)
		pDepthToShiftTable[i] = nLastIndex;

	return XN_STATUS_OK;
}

// Faz a conversão dos valores da câmera para valores de distância (em mm)
// Trabalha com um intervalo de valores e é utilizada pelas aplicações
XnStatus DepthPanasonic::XnShiftToDepthConvert(XnShiftToDepthTables* pShiftToDepth, XnUInt16* pInput, XnUInt32 nInputSize, XnDepthPixel* pOutput)
{
	XN_VALIDATE_INPUT_PTR(pShiftToDepth);
	XN_VALIDATE_INPUT_PTR(pInput);
	XN_VALIDATE_INPUT_PTR(pOutput);

	XnUInt16* pInputEnd = pInput + nInputSize;
	XnDepthPixel* pShiftToDepthTable = pShiftToDepth->pShiftToDepthTable;

	while (pInput != pInputEnd)
	{
		pOutput[0] = pShiftToDepthTable[pInput[0]];

		pInput++;
		pOutput++;
	}

	return XN_STATUS_OK;
}

XnBool DepthPanasonic::IsCapabilitySupported( const XnChar* strCapabilityName )
{

	XnBool result;

	// we only support the mirror capability
	result =  (strcmp(strCapabilityName, XN_CAPABILITY_MIRROR) == 0);


	/*result = (strcmp(strCapabilityName, XN_CAPABILITY_USER_POSITION) == 0 ||
		strcmp(strCapabilityName, XN_CAPABILITY_ALTERNATIVE_VIEW_POINT) == 0 ||
		strcmp(strCapabilityName, XN_CAPABILITY_FRAME_SYNC) == 0 ||
		strcmp(strCapabilityName, XN_CAPABILITY_CROPPING) == 0 );*/

	//result =  (strcmp(strCapabilityName, XN_CAPABILITY_MIRROR) == 0 || 
		//strcmp(strCapabilityName, XN_CAPABILITY_CROPPING) == 0 ); 

	 return result;
}

XnStatus DepthPanasonic::StartGenerating()
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

XnBool DepthPanasonic::IsGenerating()
{
	return m_bGenerating;
}

void DepthPanasonic::StopGenerating()
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

XnStatus DepthPanasonic::RegisterToGenerationRunningChange( XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback )
{
	return m_generatingEvent.Register(handler, pCookie, &hCallback);
}

void DepthPanasonic::UnregisterFromGenerationRunningChange( XnCallbackHandle hCallback )
{
	m_generatingEvent.Unregister(hCallback);
}

XnStatus DepthPanasonic::RegisterToNewDataAvailable( XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback )
{
	return m_dataAvailableEvent.Register(handler, pCookie, &hCallback);
}

void DepthPanasonic::UnregisterFromNewDataAvailable( XnCallbackHandle hCallback )
{
	m_dataAvailableEvent.Unregister(hCallback);
}

XnBool DepthPanasonic::IsNewDataAvailable( XnUInt64& nTimestamp )
{
	return m_bDataAvailable;
}

XnUInt32 DepthPanasonic::GetDataSize()
{
	return (SUPPORTED_X_RES * SUPPORTED_Y_RES * sizeof(XnDepthPixel));
}

XnUInt64 DepthPanasonic::GetTimestamp()
{
	return m_nTimestamp;
}

XnUInt32 DepthPanasonic::GetFrameID()
{
	return m_nFrameID;
}

xn::ModuleMirrorInterface* DepthPanasonic::GetMirrorInterface()
{
	return this;
}

XnStatus DepthPanasonic::SetMirror( XnBool bMirror )
{
	m_bMirror = bMirror;
	m_mirrorEvent.Raise();
	return (XN_STATUS_OK);
}

XnBool DepthPanasonic::IsMirrored()
{
	return m_bMirror;
}

XnStatus DepthPanasonic::RegisterToMirrorChange( XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback )
{
	return m_mirrorEvent.Register(handler, pCookie, &hCallback);
}

void DepthPanasonic::UnregisterFromMirrorChange( XnCallbackHandle hCallback )
{
	m_mirrorEvent.Unregister(hCallback);
}

XnUInt32 DepthPanasonic::GetSupportedMapOutputModesCount()
{
	// we only support a single mode
	return 1;
}

XnStatus DepthPanasonic::GetSupportedMapOutputModes( XnMapOutputMode aModes[], XnUInt32& nCount )
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

XnStatus DepthPanasonic::SetMapOutputMode( const XnMapOutputMode& Mode )
{

	// make sure this is our supported mode
	if (Mode.nXRes != SUPPORTED_X_RES ||
		Mode.nYRes != SUPPORTED_Y_RES ||
		Mode.nFPS != SUPPORTED_FPS)
	{
		printf("XN_STATUS_BAD_PARAM\n");
		return (XN_STATUS_BAD_PARAM);
	}

	return (XN_STATUS_OK);
}

XnStatus DepthPanasonic::GetMapOutputMode( XnMapOutputMode& Mode )
{
	Mode.nXRes = SUPPORTED_X_RES;
	Mode.nYRes = SUPPORTED_Y_RES;
	Mode.nFPS = SUPPORTED_FPS;

	return (XN_STATUS_OK);
}

XnStatus DepthPanasonic::RegisterToMapOutputModeChange( XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback )
{

	// no need. we only allow one mode
	hCallback = this;
	return XN_STATUS_OK;
}

void DepthPanasonic::UnregisterFromMapOutputModeChange( XnCallbackHandle hCallback )
{
	// do nothing (we didn't really register)	
}

XnDepthPixel* DepthPanasonic::GetDepthMap()
{
	return m_pDepthMap;
}

XnDepthPixel DepthPanasonic::GetDeviceMaxDepth()
{
	//printf("GetDeviceMaxDepth\n");
	return MAX_DEPTH_VALUE;
}

void DepthPanasonic::GetFieldOfView( XnFieldOfView& FOV )
{

	// some numbers 
	// Números do ModuleSample
	FOV.fHFOV = 1.35;
	FOV.fVFOV = 1.35;

	// Forma calculada pelo Kinect
	XnUInt64 nZPD = m_s2dConfig.nZeroPlaneDistance;
	XnDouble fZPPS = m_s2dConfig.fZeroPlanePixelSize; 

	FOV.fHFOV = 2*atan(fZPPS*XN_SXGA_X_RES/2/nZPD);
	FOV.fVFOV = 2*atan(fZPPS*XN_VGA_Y_RES*2/2/nZPD);
}

XnStatus DepthPanasonic::RegisterToFieldOfViewChange( XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback )
{
	// no need. it never changes
	hCallback = this;
	return XN_STATUS_OK;
}

void DepthPanasonic::UnregisterFromFieldOfViewChange( XnCallbackHandle hCallback )
{
	// do nothing (we didn't really register)	
}

XN_THREAD_PROC DepthPanasonic::SchedulerThread( void* pCookie )
{
	DepthPanasonic* pThis = (DepthPanasonic*)pCookie;

	while (pThis->m_bGenerating)
	{
		// wait 33 ms (to produce 30 FPS)
		xnOSSleep(1000000/SUPPORTED_FPS/1000);

		pThis->OnNewFrame();
	}

	XN_THREAD_PROC_RETURN(0);
}

void DepthPanasonic::OnNewFrame()
{
	m_bDataAvailable = TRUE;
	m_dataAvailableEvent.Raise();
}

// Esta função retorna as propriedades inteiras do módulo, é necessária para o funcionamento das aplicações
XnStatus DepthPanasonic::GetIntProperty(const XnChar* strName, XnUInt64& nValue) const
{
	if (strcmp(strName, "MaxShift") == 0)
    {
        nValue = m_s2dConfig.nDeviceMaxShiftValue;
        return XN_STATUS_OK;
    }
    else if (strcmp(strName, XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE) == 0)
    {
        nValue = m_s2dConfig.nZeroPlaneDistance;
        return XN_STATUS_OK;
    }
    else if (strcmp(strName, "ConstShift") == 0)
    {
        nValue = m_s2dConfig.nConstShift;
        return XN_STATUS_OK;
    }
	else if (strcmp(strName, XN_STREAM_PROPERTY_DEVICE_MAX_DEPTH) == 0)
    {
		nValue = m_s2dConfig.nDeviceMaxDepthValue;
        return XN_STATUS_OK;
    }
	else if (strcmp(strName, XN_STREAM_PROPERTY_PIXEL_SIZE_FACTOR) == 0)
    {
		nValue = m_s2dConfig.nPixelSizeFactor;
        return XN_STATUS_OK;
    }
	else if (strcmp(strName, XN_STREAM_PROPERTY_PARAM_COEFF) == 0)
    {
		nValue = m_s2dConfig.nParamCoeff;
        return XN_STATUS_OK;
    }
	else if (strcmp(strName, XN_STREAM_PROPERTY_SHIFT_SCALE) == 0)
    {
		nValue = m_s2dConfig.nShiftScale;
        return XN_STATUS_OK;
    }
	else if (strcmp(strName, XN_STREAM_PROPERTY_MIN_DEPTH) == 0)
    {
		nValue = 0;
        return XN_STATUS_OK;
    }
	else if (strcmp(strName, XN_STREAM_PROPERTY_MAX_DEPTH) == 0)
    {
		nValue = m_s2dConfig.nDeviceMaxDepthValue;
        return XN_STATUS_OK;
    }	
    else
    {
        return xn::ModuleDepthGenerator::GetIntProperty(strName, nValue);
    }
}

// Esta função retorna as propriedades reais do módulo, é necessária para o funcionamento das aplicações
XnStatus DepthPanasonic::GetRealProperty(const XnChar* strName, XnDouble& dValue) const
{
	//printf ("RealPropriedade: %s \n", strName);
    if (strcmp(strName, XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE) == 0)
    {
        dValue = m_s2dConfig.fZeroPlanePixelSize;
        return XN_STATUS_OK;
    }
    else if (strcmp(strName, "LDDIS") == 0)
    {
        dValue = m_s2dConfig.fEmitterDCmosDistance;
        return XN_STATUS_OK;
    }
	else if (strcmp(strName, XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE) == 0)
    {
		dValue = m_s2dConfig.fEmitterDCmosDistance;
        return XN_STATUS_OK;
    }
    else
    {
        return xn::ModuleDepthGenerator::GetRealProperty(strName, dValue);
    }
}


// Esta função retorna as propriedades de string do módulo
XnStatus DepthPanasonic::GetStringProperty(const XnChar* strName, XnChar* csValue, XnUInt32 nBufSize) const { 

	return xn::ModuleDepthGenerator::GetStringProperty(strName, csValue, nBufSize);

}

// Esta função retorna outras propriedades do módulo (como as matrizes de conversão), é necessária para o funcionamento das aplicações
XnStatus DepthPanasonic::GetGeneralProperty(const XnChar* strName, XnUInt32 nBufferSize, void* pBuffer) const { 

	if (strcmp(strName, XN_STREAM_PROPERTY_S2D_TABLE) == 0)
	{
		XnUInt32 nTableSize = m_ShiftToDepthTables.nShiftsCount * sizeof(XnDepthPixel);
		if (nBufferSize < nTableSize)
		{
			return XN_STATUS_OUTPUT_BUFFER_OVERFLOW;
		}
		xnOSMemCopy(pBuffer, m_ShiftToDepthTables.pShiftToDepthTable, nTableSize);
		return XN_STATUS_OK;
	}
	else  if (strcmp(strName, XN_STREAM_PROPERTY_D2S_TABLE) == 0)
	{
		XnUInt32 nTableSize = m_ShiftToDepthTables.nDepthsCount * sizeof(XnUInt16);
		if (nBufferSize < nTableSize)
		{
			return XN_STATUS_OUTPUT_BUFFER_OVERFLOW;
		}

		xnOSMemCopy(pBuffer, m_ShiftToDepthTables.pDepthToShiftTable, nTableSize);
		return XN_STATUS_OK;
	}
	else
	{
		return xn::ModuleDepthGenerator::GetGeneralProperty(strName, nBufferSize, pBuffer);
	}
	
	
}

/*
XnStatus DepthPanasonic::SetCropping(const XnCropping &Cropping)
{
	//printf ("SetCropping\n");
	//printf("nXOffset: %d, nYOffset: %d, nXSize: %d, nYSize: %d\n\n", Cropping.nXOffset, Cropping.nYOffset, Cropping.nXSize, Cropping.nYSize);
	crop = XnGeneralBufferPack((void*)&Cropping, sizeof(Cropping));
	//crop2 = (XnCropping*)&Cropping;
	return XN_STATUS_OK; //m_pSensor->SetProperty(m_strModule, XN_STREAM_PROPERTY_CROPPING, gbValue);
}


XnStatus DepthPanasonic::GetCropping(XnCropping &Cropping)
{
	//printf ("GetCropping\n");
	//printf("nXOffset: %d, nYOffset: %d, nXSize: %d, nYSize: %d\n\n", Cropping.nXOffset, Cropping.nYOffset, Cropping.nXSize, Cropping.nYSize);
	XN_PACK_GENERAL_BUFFER(Cropping);
	Cropping = (XnCropping &)crop.pData;

	//Cropping = (XnCropping &)crop2;
	return XN_STATUS_OK; // m_pSensor->GetProperty(m_strModule, XN_STREAM_PROPERTY_CROPPING, XN_PACK_GENERAL_BUFFER(Cropping));
}

XnStatus DepthPanasonic::RegisterToCroppingChange(XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback)
{
	//printf ("RegisterToCroppingChange\n");
	const XnChar* aProps[] = 
	{
		XN_CAPABILITY_CROPPING,
		NULL
	};

	return XN_STATUS_OK; //RegisterToProps(handler, pCookie, hCallback, aProps);
}

void DepthPanasonic::UnregisterFromCroppingChange(XnCallbackHandle hCallback)
{
	//printf ("UnregisterFromCroppingChange\n");
	//UnregisterFromProps(hCallback);
}
*/

/*XnStatus XnPixelStream::CropImpl(XnStreamData* pStreamOutput, const XnCropping* pCropping)
{
	XnStatus nRetVal = XN_STATUS_OK;

	XnUChar* pPixelData = (XnUChar*)pStreamOutput->pData;
	XnUInt32 nCurDataSize = 0;

	for (XnUInt32 y = pCropping->nYOffset; y < XnUInt32(pCropping->nYOffset + pCropping->nYSize); ++y)
	{
		XnUChar* pOrigLine = &pPixelData[y * GetXRes() * GetBytesPerPixel()];

		// move line
		xnOSMemCopy(pPixelData + nCurDataSize, pOrigLine + pCropping->nXOffset * GetBytesPerPixel(), pCropping->nXSize * GetBytesPerPixel());
		nCurDataSize += pCropping->nXSize * GetBytesPerPixel();
	}

	// update size
	pStreamOutput->nDataSize = nCurDataSize;

	return XN_STATUS_OK;
}*/