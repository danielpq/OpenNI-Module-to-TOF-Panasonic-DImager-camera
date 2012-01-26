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


#include "ExportedDepthPanasonic.h"
#include "DepthPanasonic.h"

ExportedDepthPanasonic::ExportedDepthPanasonic()
{}

void ExportedDepthPanasonic::GetDescription( XnProductionNodeDescription* pDescription )
{
	pDescription->Type = XN_NODE_TYPE_DEPTH;
	strcpy(pDescription->strVendor, "PrimeSense");
	strcpy(pDescription->strName, "SensorV2");
	pDescription->Version.nMajor = 5;
	pDescription->Version.nMinor = XN_MINOR_VERSION;
	pDescription->Version.nMaintenance = XN_MAINTENANCE_VERSION;
	pDescription->Version.nBuild = XN_BUILD_VERSION;
}

XnStatus ExportedDepthPanasonic::EnumerateProductionTrees( xn::Context& context, xn::NodeInfoList& TreesList, xn::EnumerationErrors* pErrors )
{
	XnStatus nRetVal = XN_STATUS_OK;

	// return one option
	XnProductionNodeDescription desc;
	GetDescription(&desc);

	// Talvez devesse ser maior
	nRetVal = TreesList.Add(desc, NULL, NULL);
	XN_IS_STATUS_OK(nRetVal);


	return (XN_STATUS_OK);
}

XnStatus ExportedDepthPanasonic::Create( xn::Context& context, const XnChar* strInstanceName, const XnChar* strCreationInfo, xn::NodeInfoList* pNeededTrees, const XnChar* strConfigurationDir, xn::ModuleProductionNode** ppInstance )
{
	XnStatus nRetVal = XN_STATUS_OK;
	
	DepthPanasonic* pDepth = new DepthPanasonic();

	nRetVal = pDepth->Init();
	if (nRetVal != XN_STATUS_OK)
	{
		delete pDepth;
		return (nRetVal);
	}

	*ppInstance = pDepth;

	
	return (XN_STATUS_OK);
}

void ExportedDepthPanasonic::Destroy( xn::ModuleProductionNode* pInstance )
{
	delete pInstance;
}