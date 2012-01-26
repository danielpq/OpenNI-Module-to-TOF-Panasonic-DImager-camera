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


#include <XnModuleCppInterface.h>
#include <XnEvent.h>
#include <XnOS.h>
#include <Dimagerdll.h> // DLL da camera Panasonic
#include <math.h>
//#include <fstream>
//#include <iostream>

//using namespace std;


//---------------------------------------------------------------------------
// Depth Specific Properties
//---------------------------------------------------------------------------
/** Integer */ 
#define XN_STREAM_PROPERTY_GAIN						"Gain"
/** Integer */ 
#define XN_STREAM_PROPERTY_HOLE_FILTER				"HoleFilter"
/** Integer */ 
#define XN_STREAM_PROPERTY_MIN_DEPTH				"MinDepthValue"
/** Integer */ 
#define XN_STREAM_PROPERTY_MAX_DEPTH				"MaxDepthValue"
/** Integer */ 
#define XN_STREAM_PROPERTY_SHADOW					"ShadowValue"
/** Integer */ 
#define XN_STREAM_PROPERTY_NO_SAMPLE				"NoSampleValue"
/** Boolean */ 
#define XN_STREAM_PROPERTY_REGISTRATION				"Registration"
/** XnProcessingType */ 
#define XN_STREAM_PROPERTY_REGISTRATION_TYPE		"RegistrationType"
/** Boolean */
#define XN_STREAM_PROPERTY_WHITE_BALANCE_ENABLED	"WhiteBalancedEnabled"
/** XnDepthAGCBin* */
#define XN_STREAM_PROPERTY_AGC_BIN					"AGCBin"
/** Integer */ 
#define XN_STREAM_PROPERTY_CONST_SHIFT				"ConstShift"
/** Integer */ 
#define XN_STREAM_PROPERTY_PIXEL_SIZE_FACTOR		"PixelSizeFactor"
/** Integer */ 
#define XN_STREAM_PROPERTY_MAX_SHIFT				"MaxShift"
/** Integer */ 
#define XN_STREAM_PROPERTY_PARAM_COEFF				"ParamCoeff"
/** Integer */ 
#define XN_STREAM_PROPERTY_SHIFT_SCALE				"ShiftScale"
/** XN_DEPTH_TYPE[] */ 
#define XN_STREAM_PROPERTY_S2D_TABLE				"S2D"
/** XnUInt16[] */ 
#define XN_STREAM_PROPERTY_D2S_TABLE				"D2S"
/** Integer */
#define XN_STREAM_PROPERTY_DEVICE_MAX_DEPTH			"DeviceMaxDepth"
/** XN_DEPTH_TYPE */ 
#define XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE		"ZPD"
/** Real */ 
#define XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE	"ZPPS"
/** Real */ 
#define XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE	"LDDIS"
/** Boolean */
#define XN_STREAM_PROPERTY_GMC_MODE					"GmcMode"

// Estrutura que armazena os parâmetros da câmera
typedef struct XnShiftToDepthConfig
{
	// The zero plane distance in depth units. 
	XnDepthPixel nZeroPlaneDistance;
	// The zero plane pixel size 
	XnFloat fZeroPlanePixelSize;
	// The distance between the emitter and the Depth Cmos 
	XnFloat fEmitterDCmosDistance;
	// The maximum possible shift value from this device.
	XnUInt32 nDeviceMaxShiftValue;
	// The maximum possible depth from this device (as opposed to a cut-off).
	XnUInt32 nDeviceMaxDepthValue;

	XnUInt32 nConstShift;
	XnUInt32 nPixelSizeFactor;
	XnUInt32 nParamCoeff;
	XnUInt32 nShiftScale;

	// The minimum depth value in the depth buffer. 
	XnDepthPixel nDepthMinCutOff;

	// The maximum depth value in the depth buffer. 
	XnDepthPixel nDepthMaxCutOff;

} XnShiftToDepthConfig;

// Estrutura que armazena as tabelas de conversão S2D e D2S
typedef struct XnShiftToDepthTables
{
	XnBool bIsInitialized;
	// The shift-to-depth table.
	XnDepthPixel* pShiftToDepthTable;
	// The number of entries in the shift-to-depth table. 
	XnUInt32 nShiftsCount;
	// The depth-to-shift table. 
	XnUInt16* pDepthToShiftTable;
	// The number of entries in the depth-to-shift table. 
	XnUInt32 nDepthsCount;
} XnShiftToDepthTables;


class DepthPanasonic : 
	public virtual xn::ModuleDepthGenerator,
	public virtual xn::ModuleMirrorInterface
	//public virtual xn::ModuleCroppingInterface
	//public virtual xn::ModuleProductionNode
{
public:
	DepthPanasonic();
	virtual ~DepthPanasonic();

	XnStatus Init();

	// ProductionNode methods
	virtual XnBool IsCapabilitySupported(const XnChar* strCapabilityName);

	// Generator methods
	virtual XnStatus StartGenerating();
	virtual XnBool IsGenerating();
	virtual void StopGenerating();
	virtual XnStatus RegisterToGenerationRunningChange(XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback);
	virtual void UnregisterFromGenerationRunningChange(XnCallbackHandle hCallback);
	virtual XnStatus RegisterToNewDataAvailable(XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback);
	virtual void UnregisterFromNewDataAvailable(XnCallbackHandle hCallback);
	virtual XnBool IsNewDataAvailable(XnUInt64& nTimestamp);
	virtual XnStatus UpdateData();
	virtual XnUInt32 GetDataSize();
	virtual XnUInt64 GetTimestamp();
	virtual XnUInt32 GetFrameID();
	virtual xn::ModuleMirrorInterface* GetMirrorInterface();

	// Mirror methods
	virtual XnStatus SetMirror(XnBool bMirror);
	virtual XnBool IsMirrored();
	virtual XnStatus RegisterToMirrorChange(XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback);
	virtual void UnregisterFromMirrorChange(XnCallbackHandle hCallback);

	// MapGenerator methods
	virtual XnUInt32 GetSupportedMapOutputModesCount();
	virtual XnStatus GetSupportedMapOutputModes(XnMapOutputMode aModes[], XnUInt32& nCount);
	virtual XnStatus SetMapOutputMode(const XnMapOutputMode& Mode);
	virtual XnStatus GetMapOutputMode(XnMapOutputMode& Mode);
	virtual XnStatus RegisterToMapOutputModeChange(XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback);
	virtual void UnregisterFromMapOutputModeChange(XnCallbackHandle hCallback);

	// DepthGenerator methods
	virtual XnDepthPixel* GetDepthMap();
	virtual XnDepthPixel GetDeviceMaxDepth();
	virtual void GetFieldOfView(XnFieldOfView& FOV);
	virtual XnStatus RegisterToFieldOfViewChange(XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback);
	virtual void UnregisterFromFieldOfViewChange(XnCallbackHandle hCallback);

	// Funções que retornam as propriedades da câmera
	virtual XnStatus GetIntProperty(const XnChar* strName, XnUInt64& nValue) const;
	virtual XnStatus GetRealProperty(const XnChar* strName, XnDouble& dValue) const;
	virtual XnStatus GetStringProperty(const XnChar* strName, XnChar* csValue, XnUInt32 nBufSize) const;
	virtual XnStatus GetGeneralProperty(const XnChar* strName, XnUInt32 nBufferSize, void* pBuffer) const;


	// ShiftToDepth Functions (convertem de valores de distância para mm)
	XnStatus XnShiftToDepthUpdate(XnShiftToDepthTables* pShiftToDepth, const XnShiftToDepthConfig* pConfig);
	XnStatus XnShiftToDepthConvert(XnShiftToDepthTables* pShiftToDepth, XnUInt16* pInput, XnUInt32 nInputSize, XnDepthPixel* pOutput);


	// Novas funções (não são necessárias, ao mesmo para o rastreamento da mão)
	/*XnStatus UpdateRealWorldTranslationData();
	void RealWorldTranslationPropChanged(void* pCookie);
	XnUInt32 GetSupportedUserPositionsCount();
	XnStatus SetUserPosition(XnUInt32 nIndex, const XnBoundingBox3D& Position);
	XnStatus GetUserPosition(XnUInt32 nIndex, XnBoundingBox3D& Position);
	XnStatus RegisterToUserPositionChange(XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback);
	void UnregisterFromUserPositionChange(XnCallbackHandle hCallback);*/
	//void FilterProperties(XnActualPropertiesHash* pHash);


	// Cropping (não são necessárias, ao mesmo para o rastreamento da mão)
	/*xn::ModuleCroppingInterface* GetCroppingInterface() { return this; }
	XnStatus SetCropping(const XnCropping &Cropping);
	XnStatus GetCropping(XnCropping &Cropping);
	XnStatus RegisterToCroppingChange(XnModuleStateChangedHandler handler, void* pCookie, XnCallbackHandle& hCallback);
	void UnregisterFromCroppingChange(XnCallbackHandle hCallback);*/

private:
	XN_DECLARE_EVENT_0ARG(ChangeEvent, ChangeEventInterface);

	static XN_THREAD_PROC SchedulerThread(void* pCookie);
	void OnNewFrame();

	XnBool m_bGenerating;
	XnBool m_bDataAvailable;
	XnDepthPixel* m_pDepthMap;
	XnUInt32 m_nFrameID;
	XnUInt64 m_nTimestamp;
	XN_THREAD_HANDLE m_hScheduler;
	XnBool m_bMirror;
	ChangeEvent m_generatingEvent;
	ChangeEvent m_dataAvailableEvent;
	ChangeEvent m_mirrorEvent;

	// Aloca memória para os vetores retornados pela câmera (160*120*2 bytes = 38400 bytes)
	unsigned short *kdat;
	unsigned short *ndat;
  

	XnShiftToDepthConfig m_s2dConfig;
	XnShiftToDepthTables m_ShiftToDepthTables;
	
	// Cropping
	/*XnGeneralBuffer crop;
	XnCropping *crop2;*/
};
