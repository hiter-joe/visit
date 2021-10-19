// Copyright (c) Lawrence Livermore National Security, LLC and other VisIt
// Project developers.  See the top-level LICENSE file for dates and other
// details.  No copyright assignment is required to contribute to VisIt.

// ************************************************************************* //
//                    avtOSPRaySamplePointExtractor.C                        //
// ************************************************************************* //

#include <avtOSPRaySamplePointExtractor.h>

#include <avtCellList.h>
#include <avtOSPRayVoxelExtractor.h>
#include <avtParallel.h>
#include <avtSamplePoints.h>
#include <avtVolume.h>

#include <DebugStream.h>
#include <ImproperUseException.h>
#include <StackTimer.h>

#include <vtkRectilinearGrid.h>
#include <vtkMatrix4x4.h>

#include <string>
#include <utility>

// ****************************************************************************
//  Method: avtOSPRaySamplePointExtractor constructor
//
//  Arguments:
//      w       The width.
//      h       The height.
//      d       The depth.
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ****************************************************************************

avtOSPRaySamplePointExtractor::avtOSPRaySamplePointExtractor(int w,
                                                             int h,
                                                             int d)
    : avtSamplePointExtractorBase(w, h, d)
{
#ifdef COMMENT_OUT_FOR_NOW
    ospray_core = nullptr;
#endif
    osprayVoxelExtractor = nullptr;
    modelViewProj = vtkMatrix4x4::New();
    patchCount = 0;
    imageMetaPatchVector.clear();
    imgDataHashMap.clear();
}


// ****************************************************************************
//  Method: avtOSPRaySamplePointExtractor destructor
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ****************************************************************************

avtOSPRaySamplePointExtractor::~avtOSPRaySamplePointExtractor()
{
    if (osprayVoxelExtractor != nullptr)
    {
        delete osprayVoxelExtractor;
        osprayVoxelExtractor = nullptr;
    }

    DelImgPatches();
}


// ****************************************************************************
//  Method: avtOSPRaySamplePointExtractor::SetUpExtractors
//
//  Purpose:
//      Sets up the extractors and tell them which volume to extract into.
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ****************************************************************************

void
avtOSPRaySamplePointExtractor::SetUpExtractors(void)
{
    StackTimer t0("avtOSPRaySamplePointExtractor::SetUpExtractors");
    avtSamplePoints_p output = GetTypedOutput();

    // The volume will always be a nullptr the first time through.
    // For subsequent tiles (provided tiling is being done) it will
    // not be the case.
    if (output->GetVolume() == nullptr)
        output->SetVolume(width, height, depth);
    else
        output->GetVolume()->ResetSamples();

    output->ResetCellList();

    avtVolume *volume = output->GetVolume();

    if (shouldDoTiling)
        volume->Restrict(width_min, width_max-1, height_min, height_max-1);

    // Set up the extractor with the current cell list.
    avtCellList *cl = output->GetCellList();

    if (osprayVoxelExtractor != nullptr)
    {
        delete osprayVoxelExtractor;
    }

    osprayVoxelExtractor = new avtOSPRayVoxelExtractor(width, height, depth,
                                                       volume, cl);

    // Commneted out because jittering is explicitly set via a method
    // call.
    // osprayVoxelExtractor->SetJittering(jitter);

    if (shouldDoTiling)
    {
        osprayVoxelExtractor->Restrict(width_min, width_max-1,
                                      height_min, height_max-1);
    }
}

// ****************************************************************************
//  Method: avtOSPRaySamplePointExtractor::InitSampling
//
//  Purpose:
//      Initialize sampling, called by base class ExecuteTree method before.
//      the actual iteration starts. This function might be useful for
//      children classes
//
//  Arguments:
//      dt      The dataset tree that should be processed.
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ****************************************************************************

void
avtOSPRaySamplePointExtractor::InitSampling(avtDataTree_p dt)
{
#ifdef COMMENT_OUT_FOR_NOW
    ospray::Context* ospray = (ospray::Context*)ospray_core;

    for (int i = 0; i < dt->GetNChildren(); ++i)
    {
        ospray->InitPatch(i);
    }
#endif
    patchCount = 0;
    imageMetaPatchVector.clear();
    imgDataHashMap.clear();
}

// ****************************************************************************
//  Method: avtOSPRaySamplePointExtractor::DoSampling
//
//  Purpose:
//      Performs sampling, called by base class ExecuteTree method.
//
//  Arguments:
//      ds      The data set that should be processed.
//      idx     The index of the dataset.
//
//  Programmer: Kathleen Biagas
//  Creation:   April 18, 2018
//
//  Modifications:
//
// ****************************************************************************

void
avtOSPRaySamplePointExtractor::DoSampling(vtkDataSet *ds, int idx)
{
    // Initialize ospray
    StackTimer t0("avtOSPRaySamplePointExtractor::DoSampling "
                  "OSPVisItContext::InitPatch");

    // Volume scalar range
    double scalarRange[2];
    {
        StackTimer t1("avtOSPRaySamplePointExtractor::DoSampling "
                      "Retrieve Volume Scalar Range");
        ds->GetScalarRange(scalarRange);
    }

    // Transfer function visible range
    double tfnVisibleRange[2];
    {
        StackTimer t2("avtOSPRaySamplePointExtractor::DoSampling "
                      "Retrieve TFN Visible Range");
        tfnVisibleRange[0] = transferFn1D->GetMinVisibleScalar();
        tfnVisibleRange[1] = transferFn1D->GetMaxVisibleScalar();
    }

    osprayVoxelExtractor->SetScalarRange(scalarRange);
    osprayVoxelExtractor->SetTFVisibleRange(tfnVisibleRange);

    RasterBasedSample(ds, idx);
}


// ****************************************************************************
//  Method: avtOSPRaySamplePointExtractor::RasterBasedSample
//
//  Purpose:
//      Does raster based sampling.
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
//    Eric Brugger, Thu Apr 18 14:15:53 PDT 2019
//    Converted the generation of the exception from ospray::Exception
//    to EXCEPTION1.
//
// ****************************************************************************

void
avtOSPRaySamplePointExtractor::RasterBasedSample(vtkDataSet *ds, int num)
{
    StackTimer t0("avtOSPRaySamplePointExtractor::RasterBasedSample");

    if (ds->GetDataObjectType() == VTK_RECTILINEAR_GRID)
    {
        avtDataAttributes &atts = GetInput()->GetInfo().GetAttributes();

        avtSamplePoints_p samples = GetTypedOutput();
        int numVars = samples->GetNumberOfRealVariables();

        std::vector<std::string> varnames;
        std::vector<int>         varsizes;

        for (int i=0; i<numVars; ++i)
        {
            varnames.push_back(samples->GetVariableName(i));
            varsizes.push_back(samples->GetVariableSize(i));
        }

        // Voxel extractor Setup
        osprayVoxelExtractor->SetProcIdPatchID(PAR_Rank(), num);
#ifdef COMMENT_OUT_FOR_NOW
        osprayVoxelExtractor->SetOSPRay(ospray_core);
#endif
        osprayVoxelExtractor->SetViewInfo(viewInfo);
        osprayVoxelExtractor->SetSamplingRate(samplingRate);
        osprayVoxelExtractor->SetRenderingExtents(renderingExtents);
        osprayVoxelExtractor->SetMVPMatrix(modelViewProj);

        // Note (Qi): probably not necessary.
        const double *xform = nullptr;
        if (atts.GetRectilinearGridHasTransform())
            xform = atts.GetRectilinearGridTransform();

        osprayVoxelExtractor->SetGridsAreInWorldSpace
            (rectilinearGridsAreInWorldSpace, view, aspect, xform);

        // Extract
        osprayVoxelExtractor->Extract((vtkRectilinearGrid *) ds, varnames,
                                      varsizes);

        // Get the rendering results, putting put them into a vector
        // sorted based on z value.
        ImgMetaData tmpImageMetaPatch;
        tmpImageMetaPatch = InitMetaPatch(patchCount);

        osprayVoxelExtractor->GetImageDimensions
            (tmpImageMetaPatch.inUse,     tmpImageMetaPatch.dims,
             tmpImageMetaPatch.screen_ll, tmpImageMetaPatch.screen_ur,
             tmpImageMetaPatch.eye_z,     tmpImageMetaPatch.clip_z);

        // ARS - What is happending here and why??
        if (tmpImageMetaPatch.inUse == 1)
        {
            tmpImageMetaPatch.avg_z = tmpImageMetaPatch.eye_z;
            tmpImageMetaPatch.destProcId = tmpImageMetaPatch.procId;
            imageMetaPatchVector.push_back(tmpImageMetaPatch);

            ImgData tmpImageDataHash;
            tmpImageDataHash.procId = tmpImageMetaPatch.procId;
            tmpImageDataHash.patchNumber = tmpImageMetaPatch.patchNumber;
            tmpImageDataHash.imagePatch =
                new float[tmpImageMetaPatch.dims[0] *
                          tmpImageMetaPatch.dims[1] * 4];

            osprayVoxelExtractor->GetComputedImage(tmpImageDataHash.imagePatch);
            imgDataHashMap.insert
                (std::pair<int, ImgData>
                   (tmpImageDataHash.patchNumber, tmpImageDataHash));

            ++patchCount;
        }
    }
    else
    {
        // All other grids
        if (num == 0) {
            const std::string msg =
                "The data wasn't rendered because OSPRay currently only "
                "supports volume rendering rectilinear grids.";
            EXCEPTION1(ImproperUseException, msg);
        }
    }
}


// ****************************************************************************
//  Method: avtOSPRaySamplePointExtractor::SendJittering
//
//  Purpose:
//      Tell the individual cell extractors whether or not to jitter.
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
// ****************************************************************************

void
avtOSPRaySamplePointExtractor::SendJittering()
{
    if (osprayVoxelExtractor != nullptr)
    {
        osprayVoxelExtractor->SetJittering(jitter);
    }
    else
    {
        const std::string msg =
          "Calling avtOSPRaySamplePointExtractor::SendJittering() "
          "when there is no extractor defined.";
        EXCEPTION1(ImproperUseException, msg);
    }
}


// ****************************************************************************
//  Method:  avtOSPRaySamplePointExtractor::FilterUnderstandsTransformedRectMesh
//
//  Purpose: If this filter returns true, it means the filter
//    correctly deals with rectilinear grids having an implied
//    transform set in the data attributes.  It can do this
//    conditionally if desired.
//
//  Arguments:
//    none
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
// ****************************************************************************

bool
avtOSPRaySamplePointExtractor::FilterUnderstandsTransformedRectMesh()
{
    return true;
}


// ****************************************************************************
//  Method: avtOSPRaySamplePointExtractor::DelImgPatches
//
//  Purpose:
//      Deletes the image patches previously defined in RasterBasedSample.
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ****************************************************************************
void
avtOSPRaySamplePointExtractor::DelImgPatches()
{
    imageMetaPatchVector.clear();

    for (iter_t it=imgDataHashMap.begin(); it!=imgDataHashMap.end(); ++it)
    {
        if ((*it).second.imagePatch != nullptr) {
            delete [](*it).second.imagePatch;
        }

        (*it).second.imagePatch = nullptr;
    }

    imgDataHashMap.clear();
}


// ****************************************************************************
//  Method: avtOSPRaySamplePointExtractor::GetImgData
//
//  Purpose:
//      Copies a patch and returns it.
//      Does shallow copy instead deep copy for efficiency
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ****************************************************************************
void
avtOSPRaySamplePointExtractor::GetAndDelImgData(int patchId,
                                                ImgData &tempImgData)
{
    size_t imagePatchSize =
        imageMetaPatchVector[patchId].dims[0] *
        imageMetaPatchVector[patchId].dims[1] * sizeof(float) * 4;

    iter_t it = imgDataHashMap.find(patchId);
    tempImgData.procId = it->second.procId;
    tempImgData.patchNumber = it->second.patchNumber;

    // Do a shallow copy instead of deep copy
    tempImgData.imagePatch = it->second.imagePatch;

    // memcpy(tempImgData.imagePatch,
    //        it->second.imagePatch,
    //        imagePatchSize);

    // ARS - Possible memory leak if the tempImgData is not deleted.
    // delete [](*it).second.imagePatch;

    it->second.imagePatch = nullptr;
}


// ****************************************************************************
//  Method: avtOSPRaySamplePointExtractor::InitMetaPatch
//
//  Purpose: Inits the metea data for a patch.
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ****************************************************************************
ImgMetaData
avtOSPRaySamplePointExtractor::InitMetaPatch(int id)
{
    ImgMetaData temp;

    temp.inUse = 0;
    temp.procId = PAR_Rank();
    temp.destProcId = PAR_Rank();
    temp.patchNumber = id;
    temp.dims[0] = temp.dims[1] = -1;
    temp.screen_ll[0] = temp.screen_ll[1] = -1;
    temp.screen_ur[0] = temp.screen_ur[1] = -1;
    temp.avg_z  = -1.0;
    temp.eye_z  = -1.0;
    temp.clip_z = -1.0;

    return temp;
}
