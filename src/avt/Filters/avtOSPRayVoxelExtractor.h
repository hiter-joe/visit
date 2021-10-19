// Copyright (c) Lawrence Livermore National Security, LLC and other VisIt
// Project developers.  See the top-level LICENSE file for dates and other
// details.  No copyright assignment is required to contribute to VisIt.

// ************************************************************************* //
//                            avtOSPRayVoxelExtractor.h                      //
// ************************************************************************* //

#ifndef AVT_OSPRAY_VOXEL_EXTRACTOR_H
#define AVT_OSPRAY_VOXEL_EXTRACTOR_H

#include <filters_exports.h>

#include <avtOSPRayCommon.h>
#include <avtViewInfo.h>
#include <avtVoxelExtractor.h>

#include <vtkMatrix4x4.h>

#include <string>
#include <vector>

class avtVolume;
class avtCellList;

class vtkRectilinearGrid;


// ****************************************************************************
//  Class: avtOSPRayVoxelExtractor
//
//  Purpose:
//      Extracts sample points from a collection of voxels.  It assumes that
//      the voxels it has been given are in camera space and does not try to
//      populate points that are not in the cube [-1, 1], [-1, 1], [-1, 1].
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ****************************************************************************

class AVTFILTERS_API avtOSPRayVoxelExtractor : public avtVoxelExtractor
{
  public:
                     avtOSPRayVoxelExtractor(int, int, int,
					     avtVolume *, avtCellList *);
    virtual         ~avtOSPRayVoxelExtractor();

    void             Extract(vtkRectilinearGrid *,
                             std::vector<std::string> &varnames,
                             std::vector<int> &varsize);

    // void             SetVariableInformation(std::vector<std::string> &names,
    //                                         std::vector<int> varsize);

    void             SetProcIdPatchID(int c, int p)   { proc = c; patch = p; };
#ifdef COMMENT_OUT_FOR_NOW
    void             SetOSPRay(OSPVisItContext* o)        { ospray_core = o; };
#endif
    void             SetViewInfo(const avtViewInfo & v)      { viewInfo = v; };
    void             SetSamplingRate(double r)           { samplingRate = r; };
    void             SetRenderingExtents(int extents[4])
    {
        renderingExtents[0] = extents[0];
        renderingExtents[1] = extents[1];
        renderingExtents[2] = extents[2];
        renderingExtents[3] = extents[3];
    };

    void             SetMVPMatrix(vtkMatrix4x4 *mvp)
    {
        model_to_screen_transform->DeepCopy(mvp);
        vtkMatrix4x4::Invert(model_to_screen_transform,
                             screen_to_model_transform);
    };

    void             SetScalarRange(double r[2])
    {
        scalarRange[0] = r[0];
        scalarRange[1] = r[1];
    };

    void             SetTFVisibleRange(double r[2])
    {
        tFVisibleRange[0] = r[0];
        tFVisibleRange[1] = r[1];
    };

    void             GetImageDimensions(bool &,
					int dims[2],
					int screen_ll[2],
                                        int screen_ur[2],
					float &,
					float &) const;
    
    void             GetComputedImage(float *image);

  protected:
    void             ExtractWorldSpaceGridOSPRay(vtkRectilinearGrid *,
                                                 std::vector<std::string> &varnames,
                                                 std::vector<int> &varsize);

    // The output image
    float           *finalImage {nullptr};

    // Some meta information
    bool             drawn {false};  // Whether or not the patch was drawn.
    int              patch {-1};     // Patch id.
    int              proc  {-1};     // Processor id.

    // Fields
    avtViewInfo      viewInfo;
#ifdef COMMENT_OUT_FOR_NOW
    OSPVisItContext *ospray_core;
#endif
    double           samplingRate {0};
    int              renderingExtents[4];

    // Transform matrices
    vtkMatrix4x4    *model_to_screen_transform {nullptr};
    vtkMatrix4x4    *screen_to_model_transform {nullptr};

    // Others
    double           scalarRange[2];
    double           tFVisibleRange[2];
    int              imgWidth {0};
    int              imgHeight{0} ;
    int              imgDims[2];       // Patch size
    int              imgLowerLeft[2];  // Coordinates in the whole image
    int              imgUpperRight[2]; // Coordinates in the whole image
    float            eyeSpaceDepth;    // For blending patches
    float            clipSpaceDepth;   // Clip space depth for
                                       // blending with the background
    int              xMin, xMax, yMin, yMax;
};

#endif
