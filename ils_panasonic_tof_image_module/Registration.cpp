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


#include "ExportedImagePanasonic.h"
#include <XnModuleCppRegistratration.h>

// tell OpenNI this shared library is a module
XN_EXPORT_MODULE(Module)

// tell OpenNI we export a depth node, and give it its exporter class.
XN_EXPORT_IMAGE(ExportedImagePanasonic)