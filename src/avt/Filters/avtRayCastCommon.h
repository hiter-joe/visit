// Copyright (c) Lawrence Livermore National Security, LLC and other VisIt
// Project developers.  See the top-level LICENSE file for dates and other
// details.  No copyright assignment is required to contribute to VisIt.

#ifndef AVT_RAYCAST_COMMON_H
#define AVT_RAYCAST_COMMON_H

#ifndef CLAMP
# define CLAMP(x, l, h) (x > l ? x < h ? x : h : l)
#endif

#ifndef M_MIN
# define M_MIN(x, r) (x < r ? x : r)
#endif

#ifndef M_MAX
# define M_MAX(x, r) (x > r ? x : r)
#endif

// ***********************************************************************
//  Struct:  ImgMetaData
//
//  Purpose:
//    Holds information about patches but not the image
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***********************************************************************

struct ImgMetaData
{
    int procId;       // Processor that produced the patch
    int patchNumber;  // Id of the patch on that processor
    int destProcId;   // Destination proc where this patch gets composited

    bool inUse;       // Whether the patch is composed locally or not

    int dims[2];      // Height, width

    int screen_ll[2]; // Lower left  position in the final image
    int screen_ur[2]; // Upper right position in the final image

    float avg_z;      // Camera space depth of the patch (average)
    float eye_z;      // Camera space z
    float clip_z;     // Clip space z
};

// ***********************************************************************
//  Struct:  ImgData
//
//  Purpose:
//    Holds the image data generated
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***********************************************************************

struct ImgData
{
    // Acts as a key
    int procId;        // Processor that produced the patch
    int patchNumber;   // Id of the patch on that processor

    float *imagePatch; // The image data - RGBA

    ImgData()
    {
        procId = -1;
        patchNumber = -1;
        imagePatch = nullptr;
    }

    bool operator==(const ImgData &a)
    {
        return (patchNumber == a.patchNumber);
    }
};

#endif
