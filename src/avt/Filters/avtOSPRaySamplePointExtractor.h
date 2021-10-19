// Copyright (c) Lawrence Livermore National Security, LLC and other VisIt
// Project developers.  See the top-level LICENSE file for dates and other
// details.  No copyright assignment is required to contribute to VisIt.

// ************************************************************************* //
//                      avtOSPRaySamplePointExtractor.h                      //
// ************************************************************************* //

#ifndef AVT_OSPRAY_SAMPLE_POINT_EXTRACTOR_H
#define AVT_OSPRAY_SAMPLE_POINT_EXTRACTOR_H

#include <filters_exports.h>
#include <avtSamplePointExtractorBase.h>

#include <avtDataTree.h>
#include <avtViewInfo.h>
#include <avtOSPRayCommon.h>

#include <map>
#include <vector>

class avtOSPRayVoxelExtractor;

class vtkDataSet;
class vtkMatrix4x4;

// ****************************************************************************
//  Class: avtOSPRaySamplePointExtractor
//
//  Purpose:
//      This is a component that will take an avtDataset as an input and find
//      all of the sample points from that dataset.
//
//  Programmer: Qi WU
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ****************************************************************************

class AVTFILTERS_API avtOSPRaySamplePointExtractor 
    : public avtSamplePointExtractorBase
{
  public:
                          avtOSPRaySamplePointExtractor(int, int, int);
    virtual              ~avtOSPRaySamplePointExtractor();

    virtual const char   *GetType(void)
                                   { return "avtOSPRaySamplePointExtractor"; };
    virtual const char   *GetDescription(void)
                                         { return "Extracting sample points";};

#ifdef COMMENT_OUT_FOR_NOW
    void                  SetOSPRay(OSPVisItContext* o)   { ospray_core = o; };
#endif
    void                  SetViewInfo(const avtViewInfo & v) { viewInfo = v; };
    void                  SetSamplingRate(double r)      { samplingRate = r; };
    void                  SetRenderingExtents(int extents[4]) 
    {
        renderingExtents[0] = extents[0];
        renderingExtents[1] = extents[1];
        renderingExtents[2] = extents[2];
        renderingExtents[3] = extents[3];
    }
    
    void                  SetMVPMatrix(vtkMatrix4x4 *mvp)
    {
        modelViewProj->DeepCopy(mvp);
    };

    int                   GetImgPatchSize() const { return patchCount; };
    void                  GetAndDelImgData(int patchId,
                                           ImgData &tempImgData);
    ImgMetaData           GetImgMetaPatch(int patchId) const
                              { return imageMetaPatchVector.at(patchId); };
    void                  DelImgPatches();
    
    std::vector<ImgMetaData>    imageMetaPatchVector;
    std::multimap<int, ImgData> imgDataHashMap;

    typedef std::multimap<int, ImgData>::iterator iter_t;

  protected:
    
    virtual void               InitSampling(avtDataTree_p dt);
    virtual void               DoSampling(vtkDataSet *, int);
    virtual void               SetUpExtractors(void);
    virtual void               SendJittering(void);
    virtual bool               FilterUnderstandsTransformedRectMesh(void);
    void                       RasterBasedSample(vtkDataSet *, int num = 0);
    ImgMetaData InitMetaPatch(int id);

#ifdef COMMENT_OUT_FOR_NOW
    OSPVisItContext          *ospray_core {nullptr};
#endif
    avtOSPRayVoxelExtractor  *osprayVoxelExtractor {nullptr};

    avtViewInfo               viewInfo;
    vtkMatrix4x4             *modelViewProj {nullptr};
    double                    samplingRate {0};
    int                       renderingExtents[4];
    int                       patchCount {0};
};

#endif
