// Copyright (c) Lawrence Livermore National Security, LLC and other VisIt
// Project developers.  See the top-level LICENSE file for dates and other
// details.  No copyright assignment is required to contribute to VisIt.

// ************************************************************************* //
//                       avtOSPRayRayTracer.C                                //
// ************************************************************************* //

#include <avtOSPRayRayTracer.h>

#include <visit-config.h>

#include <avtImage.h>
#include <avtParallel.h>
#include <avtOSPRaySamplePointExtractor.h>
#include <avtWorldSpaceToImageSpaceTransform.h>

#include <DebugStream.h>
#include <ImproperUseException.h>
#include <StackTimer.h>

#include <vtkImageData.h>
#include <vtkMatrix4x4.h>

#include <vector>

bool OSPRaySortImgMetaDataByDepth(ImgMetaData const& before,
                                  ImgMetaData const& after)
{
    return before.avg_z > after.avg_z;
}

bool OSPRaySortImgMetaDataByEyeSpaceDepth(ImgMetaData const& before,
                                          ImgMetaData const& after)
{
    return before.eye_z > after.eye_z;
}

// ****************************************************************************
//  Method: avtOSPRayRayTracer constructor
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ****************************************************************************

avtOSPRayRayTracer::avtOSPRayRayTracer() : avtRayTracerBase()
{
    // ARS - attributes
    gradientShadingEnabled = false;
    shadowsEnabled = false;
    useGridAccelerator = false;
    preIntegration = false;
    singleShade = false;
    oneSidedLighting = false;
    aoTransparencyEnabled = false;
    spp = 1;
    aoSamples = 0;
    aoDistance = 1e6;
    samplingRate = 3.0;
    minContribution = 0.001;

    materialProperties[0] = 0.4;
    materialProperties[1] = 0.75;
    materialProperties[2] = 0.0;
    materialProperties[3] = 15.0;

#ifdef COMMENT_OUT_FOR_NOW
    ospray_core = nullptr;
#endif
}


// ****************************************************************************
//  Method: avtOSPRayRayTracer destructor
//
//  Purpose:
//      Defines the destructor.  Note: this should not be inlined in the header
//      because it causes problems for certain compilers.
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
// ****************************************************************************

avtOSPRayRayTracer::~avtOSPRayRayTracer()
{
}


// ****************************************************************************
//  Method: avtOSPRayRayTracer::Execute
//
//  Purpose:
//      Executes the ray tracer.
//      This means:
//      - Put the input mesh through a transform so it is in camera space.
//      - Get the sample points.
//      - Communicate the sample points (parallel only).
//      - Composite the sample points along rays.
//      - Communicate the pixels from each ray (parallel only).
//      - Output the image.
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
//    Alister Maguire, Tue Jul 23 15:15:22 PDT 2019
//    Added a patch from Johannes Guenther that enables lighting,
//    shadows, transparency, and spp.
//
// ****************************************************************************

void
avtOSPRayRayTracer::Execute()
{
    // Initialization and Debug
    debug5 << "[avrRayTracer] entering execute" << std::endl;

    // Qi debug - Memory
#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckMemoryHere("[avtOSPRayRayTracer] Execute", "debug5");
#endif
    // initialize current time
    StackTimer t0("Ray Tracing");

    // Start of original pipeline
    bool parallelOn = (imgComm.GetParSize() == 1) ? false : true;

    
    // First transform all of the domains into the camera space.
    double aspect = 1.0;

    if (screen[1] > 0)
    {
        aspect = (double)screen[0] / (double)screen[1];
    }

    double scale[3] = {1,1,1};
    vtkMatrix4x4 *transform = vtkMatrix4x4::New();
    avtWorldSpaceToImageSpaceTransform::CalculateTransform(view, transform,
                                                           scale, aspect);

    double newNearPlane, newFarPlane, oldNearPlane, oldFarPlane;
    TightenClippingPlanes(view, transform, newNearPlane, newFarPlane);

    oldNearPlane = view.nearPlane;  oldFarPlane  = view.farPlane;
    view.nearPlane = newNearPlane;  view.farPlane  = newFarPlane;
    transform->Delete();

    avtWorldSpaceToImageSpaceTransform trans(view, aspect);
    trans.SetInput(GetInput());
    trans.SetPassThruRectilinearGrids(true);

    // **********************************************************
    // Compute Projection
    vtkImageData  *opaqueImageVTK = opaqueImage->GetImage().GetImageVTK();
    unsigned char *opaqueImageData =
        (unsigned char *) opaqueImageVTK->GetScalarPointer(0, 0, 0);
    float         *opaqueImageZB = opaqueImage->GetImage().GetZBuffer();

    std::vector<float> opaqueImageDepth(screen[0] * screen[1], oldFarPlane);

    vtkMatrix4x4  *model_to_screen_transform = vtkMatrix4x4::New();
    vtkMatrix4x4  *screen_to_model_transform = vtkMatrix4x4::New();
    vtkMatrix4x4  *screen_to_camera_transform = vtkMatrix4x4::New();

    int            renderingExtents[4];
    double         sceneSize[2];
    double         dbounds[6];  // Extents of the volume in world coordinates

    {
        GetSpatialExtents(dbounds);
#ifdef COMMENT_OUT_FOR_NOW
        ospray::ComputeProjections(view, aspect, oldNearPlane, oldFarPlane,
                                   scale, dbounds, screen,
                                   model_to_screen_transform,
                                   screen_to_model_transform,
                                   screen_to_camera_transform,
                                   sceneSize, renderingExtents);
#endif
        // ARS - Get the screen depth, project to the camera space,
        // and store the result.
        for (int y = 0; y < screen[1]; ++y)
        {
            for (int x = 0; x < screen[0]; ++x)
            {
                int index = x + y * screen[0];
                int    screenCoord[2] = {x, y};
                double screenDepth = opaqueImageZB[index] * 2 - 1;
                double worldCoord[3];
#ifdef COMMENT_OUT_FOR_NOW
                ospray::ProjectScreenToCamera(screenCoord, screenDepth,
					      screen[0], screen[1],
					      screen_to_camera_transform,
					      worldCoord);
#endif
                opaqueImageDepth[index] = -worldCoord[2];
            }
        }

        // Debug
        debug5 << "[avtOSPRayRayTracer] avtViewInfo settings: " << endl
               << "\tcamera: "
               << view.camera[0] << ", "
               << view.camera[1] << ", "
               << view.camera[2] << std::endl
               << "\tfocus: "
               << view.focus[0] << ", "
               << view.focus[1] << ", "
               << view.focus[2] << std::endl
               << "\tviewUp: "
               << view.viewUp[0] << ", "
               << view.viewUp[1] << ", "
               << view.viewUp[2] << std::endl
               << "\tviewAngle: " << view.viewAngle << std::endl
               << "\teyeAngle:  " << view.eyeAngle  << std::endl
               << "\tparallelScale: " << view.parallelScale  << std::endl
               << "\tsetScale: " << view.setScale << std::endl
               << "\tnearPlane: " << view.nearPlane << std::endl
               << "\tfarPlane:  " << view.farPlane  << std::endl
               << "\timagePan[0]: " << view.imagePan[0] << std::endl
               << "\timagePan[1]: " << view.imagePan[1] << std::endl
               << "\timageZoom:   " << view.imageZoom   << std::endl
               << "\torthographic: " << view.orthographic << std::endl
               << "\tshear[0]: " << view.shear[0] << std::endl
               << "\tshear[1]: " << view.shear[1] << std::endl
               << "\tshear[2]: " << view.shear[2] << std::endl;
        debug5 << "[avtOSPRayRayTracer] other settings " << std::endl
               << "\toldNearPlane: " << oldNearPlane
               << std::endl
               << "\toldFarPlane:  " << oldFarPlane
               << std::endl
               << "\taspect: " << aspect << std::endl
               << "\tscale:    "
               << scale[0] << " "
               << scale[1] << " "
               << scale[2] << " " << std::endl;
        debug5 << "[avtOSPRayRayTracer] sceneSize: "
               << sceneSize[0] << " "
               << sceneSize[1] << std::endl;
        debug5 << "[avtOSPRayRayTracer] screen: "
               << screen[0] << " " << screen[1] << std::endl;
        debug5 << "[avtOSPRayRayTracer] data bounds: "
               << dbounds[0] << " " << dbounds[1] << std::endl
               << "               data bounds  "
               << dbounds[2] << " " << dbounds[3] << std::endl
               << "               data bounds  "
               << dbounds[4] << " " << dbounds[5] << std::endl;
        debug5 << "[avtOSPRayRayTracer] rendering extents: "
               << renderingExtents[0] << " " << renderingExtents[1] << std::endl
               << "               rendering extents: "
               << renderingExtents[2] << " " << renderingExtents[3] << std::endl;
        debug5 << "[avtOSPRayRayTracer] full image size: "
               << renderingExtents[1] - renderingExtents[0] << " "
               << renderingExtents[3] - renderingExtents[2] << std::endl;
        debug5 << "[avtOSPRayRayTracer] model_to_screen_transform: "
               << *model_to_screen_transform << std::endl;
        debug5 << "[avtOSPRayRayTracer] screen_to_model_transform: "
               << *screen_to_model_transform << std::endl;
        debug5 << "[avtOSPRayRayTracer] screen_to_camera_transform: "
               << *screen_to_camera_transform << std::endl;

    }

    // Qi debug - Memory
#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckMemoryHere("[avtOSPRayRayTracer] Execute before ospray",
                            "debug5");
#endif

    // **********************************************************
    // OSPRay attributes
#ifdef COMMENT_OUT_FOR_NOW
    ospray::InitOSP(); // initialize ospray
    ospray::Context* ospray = (ospray::Context*) ospray_core;
    ospray->SetVariableName(activeVariable);
    ospray->SetBackgroundBuffer(opaqueImageData, opaqueImageDepth.data(),
                                screen);

    // ARS - OSPRay attributes
    ospray->SetAdaptiveSampling(false);
    ospray->SetAoSamples(aoSamples);
    ospray->SetSpp(spp);
    ospray->SetOneSidedLighting(oneSidedLighting);
    ospray->SetShadowsEnabled(shadowsEnabled);
    ospray->SetAoTransparencyEnabled(aoTransparencyEnabled);
    ospray->SetUseGridAccelerator(useGridAccelerator);
    ospray->SetPreIntegration(preIntegration);
    ospray->SetSingleShade(singleShade);
    ospray->SetGradientShadingEnabled(gradientShadingEnabled);
    ospray->SetSamplingRate(samplingRate);
    ospray->SetScaleAndDataBounds(scale, dbounds);
    ospray->SetSpecular(materialProperties[2], materialProperties[3]);

    debug5 << "[avtOSPRayRayTracer] setup the ospray camera" << std::endl;
    ospray::Camera cam(ospray->camera);
    cam.Set(view.orthographic, view.camera, view.focus, view.viewUp,
            view.viewAngle, view.imagePan, view.imageZoom, oldNearPlane,
            sceneSize, screen, renderingExtents);

    debug5 << "[avtOSPRayRayTracer] setup the ospray transfer function" << std::endl;
    ospray::TransferFunction tfn(ospray->tfn);
    tfn.Set(transferFn1D->GetTableFloat(),
            transferFn1D->GetNumberOfTableEntries(),
            transferFn1D->GetMin(),
            transferFn1D->GetMax());

    debug5 << "[avtOSPRayRayTracer] setup the ospray renderer" << std::endl;
    ospray::Renderer ren(ospray->renderer);
    ren.Init();
    ren.ResetLights();

    double light_scale = gradientShadingEnabled ? 0.9 : 1.0;
    ren.AddLight().Set(true,  materialProperties[0], light_scale); // ambient
    ren.AddLight().Set(false, materialProperties[1], light_scale, viewDirection);
    ren.AddLight().Set(false, 1.5,                   light_scale, viewDirection);
    // in VisIt there are only 8 lights
    for (int i = 0; i < 8; ++i)
    {
        const LightAttributes& la = lightList.GetLight(i);

        if (la.GetEnabledFlag())
        {
            if (la.GetType() == LightAttributes::Ambient)
            {
                ren.AddLight().Set(true, la.GetBrightness(),
                                   (double)la.GetColor().Red() / 255.0,
                                   (double)la.GetColor().Green() / 255.0,
                                   (double)la.GetColor().Blue() / 255.0);
            }
            else
            {
                ren.AddLight().Set(false, la.GetBrightness(),
                                   (double)la.GetColor().Red() / 255.0,
                                   (double)la.GetColor().Green() / 255.0,
                                   (double)la.GetColor().Blue() / 255.0,
                                   la.GetDirection());

            }
        }
    }

    ren.FinalizeLights();
    ren.Set(aoSamples, spp, oneSidedLighting, shadowsEnabled, aoTransparencyEnabled);

    // Qi debug - Memory
    ospray::CheckMemoryHere("[avtOSPRayRayTracer] Execute after ospray",
                            "debug5");
#endif

    // **********************************************************
    // Continuation of previous pipeline
    //
    // Extract all of the samples from the dataset.

    // ARS - Theses are the samples in the volume along the ray as
    // determined by the sampling rate.
    avtOSPRaySamplePointExtractor extractor(screen[0], screen[1],
                                            samplesPerRay);

#ifdef COMMENT_OUT_FOR_NOW
    extractor.SetOSPRay(ospray_core);
#endif
    extractor.SetJittering(true);
    extractor.SetTransferFn(transferFn1D);
    extractor.SetInput(trans.GetOutput());
    extractor.SetViewInfo(view);
    extractor.SetSamplingRate(samplingRate);
    extractor.SetRenderingExtents(renderingExtents); // rendered region
    extractor.SetMVPMatrix(model_to_screen_transform);

    //
    // For curvilinear and unstructured meshes, it is more efficient
    // to convert the cells to image space.  But not for rectilinear
    // meshes, so set some flags here that allow the extractor to do
    // the extraction in world space.
    //
    {
        trans.SetPassThruRectilinearGrids(true);
        extractor.SetRectilinearGridsAreInWorldSpace(true, view, aspect);
    }

    // Qi debug - Memory
#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckMemoryHere("[avtOSPRayRayTracer] Execute "
                            "raytracing setup done",
                            "debug5");
#endif

    // **********************************************************
    // Execute rendering
    //
    // ARS - The rendering applies the transfer function, lighting,
    // shading, etc.
    {
        StackTimer t1("AllPatchRendering");
        extractor.Update(GetGeneralContract());
    }

    /*
    avtDataObject_p samples = extractor.GetOutput();
    // Only required to force an update
    // Need to find a way to get rid of that!!!!
    avtRayCompositer rc(rayfoo);
    rc.SetInput(samples);
    avtImage_p image  = rc.GetTypedOutput();
    image->Update(GetGeneralContract());
    */

    // Debug
    int numPatches = extractor.GetImgPatchSize();
    debug5 << "[avtOSPRayRayTracer] Total num of patches "
           << numPatches << std::endl;

    for (int i=0; i<numPatches; i++) {
        ImgMetaData currImgMeta = extractor.GetImgMetaPatch(i);

        debug5 << "[avtOSPRayRayTracer] Rank " << PAR_Rank() << " "
               << "Idx " << i << " (" << currImgMeta.patchNumber << ") "
               << " depth " << currImgMeta.eye_z << std::endl
               << "current patch size = "
               << currImgMeta.dims[0] << ", "
               << currImgMeta.dims[1] << std::endl
               << "current patch starting"
               << " X = " << currImgMeta.screen_ll[0]
               << " Y = " << currImgMeta.screen_ll[1] << std::endl
               << "current patch ending"
               << " X = " << currImgMeta.screen_ur[0]
               << " Y = " << currImgMeta.screen_ur[1] << std::endl;
    }

    // **********************************************************
    // Image Compositing

    // Initialization
    int timingIdx;
    float *compositedData = nullptr;
    int compositedW, compositedH;
    int compositedExtents[4];

    // **********************************************************
    // IceT: If each rank has only one patch, IceT to composite
    if (imgComm.IceTValid() && extractor.GetImgPatchSize() == 1)
    {
        // Setup Local Tile
        ImgMetaData currImgMeta = extractor.GetImgMetaPatch(0);
        ImgData     currImgData;
        currImgData.imagePatch = nullptr;
        // Get a shallow copy of the data which must be deleted as
        // the extractor derefences it.
        extractor.GetAndDelImgData(currImgMeta.patchNumber, currImgData);

        // First Composition
        if (PAR_Size() > 1) {
            compositedW = renderingExtents[1] - renderingExtents[0];
            compositedH = renderingExtents[3] - renderingExtents[2];
            compositedExtents[0] = renderingExtents[0];
            compositedExtents[1] = renderingExtents[1];
            compositedExtents[2] = renderingExtents[2];
            compositedExtents[3] = renderingExtents[3];

            if (PAR_Rank() == 0) {
                compositedData = new float[4 * compositedW * compositedH]();
            }

            int currExtents[4] =
                {std::max(currImgMeta.screen_ll[0]-renderingExtents[0], 0),
                 std::min(currImgMeta.screen_ur[0]-renderingExtents[0],
                          compositedW),
                 std::max(currImgMeta.screen_ll[1]-renderingExtents[2], 0),
                 std::min(currImgMeta.screen_ur[1]-renderingExtents[2],
                          compositedH)};

            imgComm.IceTInit(compositedW, compositedH);
            imgComm.IceTSetTile(currImgData.imagePatch,
                                currExtents,
                                currImgMeta.eye_z);
            imgComm.IceTComposite(compositedData);

            // Delete the data as a shallow copy was done.
            if (currImgData.imagePatch != nullptr) {
                delete[] currImgData.imagePatch;
                currImgData.imagePatch = nullptr;
            }
        } else {
            compositedW = currImgMeta.dims[0];
            compositedH = currImgMeta.dims[1];
            compositedExtents[0] = renderingExtents[0];
            compositedExtents[1] = renderingExtents[0] + compositedW;
            compositedExtents[2] = renderingExtents[2];
            compositedExtents[3] = renderingExtents[2] + compositedH;
            compositedData = currImgData.imagePatch;
            currImgData.imagePatch = nullptr;
        }

        // Qi debug - Memory
#ifdef COMMENT_OUT_FOR_NOW
        ospray::CheckMemoryHere("[avtOSPRayRayTracer] Execute "
                                "IceT Compositing Done",
                                "debug5");
#endif
    }
    // **********************************************************
    // PARALLEL: Customized Parallel Direct Send Method
    else if (parallelOn == true)
    {
        // Parallel Direct Send
#ifdef COMMENT_OUT_FOR_NOW
        ospray::CheckSectionStart("avtOSPRayRayTracer", "Execute", timingIdx,
                                  "Parallel-Composite: "
                                  "Parallel Direct Send");
#endif
        int tags[2] = {1081, 1681};
        int tagGather = 2681;

        int *regions = nullptr;
        imgComm.RegionAllocation(regions);

        int myRegionHeight =
            imgComm.ParallelDirectSendManyPatches
            (extractor.imgDataHashMap, extractor.imageMetaPatchVector,
             numPatches, regions, imgComm.GetParSize(), tags,
             renderingExtents);

        imgComm.gatherImages(regions, imgComm.GetParSize(),
                             imgComm.intermediateImage,
                             imgComm.intermediateImageExtents,
                             imgComm.intermediateImageExtents,
                             tagGather, renderingExtents, myRegionHeight);

#ifdef COMMENT_OUT_FOR_NOW
        ospray::CheckSectionStop("avtOSPRayRayTracer", "Execute", timingIdx,
                                 "Parallel-Composite: "
                                 "Parallel Direct Send");

        // Some Cleanup
        ospray::CheckSectionStart("avtOSPRayRayTracer", "Execute", timingIdx,
                                  "Parallel-Composite: Some Cleanup");
#endif
        if (regions != nullptr)
        {
            delete [] regions;
            regions = nullptr;
        }

        if (imgComm.intermediateImage != nullptr)
        {
            delete [] imgComm.intermediateImage;
            imgComm.intermediateImage = nullptr;
        }

        imgComm.Barrier();

#ifdef COMMENT_OUT_FOR_NOW
        ospray::CheckSectionStop("avtOSPRayRayTracer", "Execute", timingIdx,
                                 "Parallel-Composite: Some Cleanup");
#endif

        // Setup for Final Composition
        compositedW = (imgComm.finalImageExtents[1] -
                       imgComm.finalImageExtents[0]);
        compositedH = (imgComm.finalImageExtents[3] -
                       imgComm.finalImageExtents[2]);

        compositedExtents[0] = imgComm.finalImageExtents[0];
        compositedExtents[1] = imgComm.finalImageExtents[1];
        compositedExtents[2] = imgComm.finalImageExtents[2];
        compositedExtents[3] = imgComm.finalImageExtents[3];

        if (PAR_Rank() == 0) {
            compositedData = imgComm.GetFinalImageBuffer();
        }

        // Qi debug - Memory
#ifdef COMMENT_OUT_FOR_NOW
        ospray::CheckMemoryHere("[avtOSPRayRayTracer] Execute "
                                "Parallel Compositing Done",
                                "debug5");
#endif
    }
    // **********************************************************
    // SERIAL: Image Composition
    else
    {
#ifdef COMMENT_OUT_FOR_NOW
        // Get the Metadata for All Patches
        ospray::CheckSectionStart("avtOSPRayRayTracer", "Execute", timingIdx,
                                  "Serial-Composite: Get the Metadata for "
                                  "All Patches");
#endif
        // Contains the metadata to composite the image
        std::vector<ImgMetaData> allPatchMeta;
        std::vector<ImgData>     allPatchData;

        // Get the number of patches
        int numPatches = extractor.GetImgPatchSize();
        for (int i=0; i<numPatches; i++)
        {
            allPatchMeta.push_back(extractor.GetImgMetaPatch(i));
        }

#ifdef COMMENT_OUT_FOR_NOW
        ospray::CheckSectionStop("avtOSPRayRayTracer", "Execute", timingIdx,
                                 "Serial-Composite: Get the Metadata for "
                                 "All Patches");

        // Sort with the Largest z First
        ospray::CheckSectionStart("avtOSPRayRayTracer", "Execute", timingIdx,
                                  "Serial-Composite: Sort with the Largest "
                                  "z First");
#endif

        std::sort(allPatchMeta.begin(), allPatchMeta.end(),
                  &OSPRaySortImgMetaDataByEyeSpaceDepth);

#ifdef COMMENT_OUT_FOR_NOW
        ospray::CheckSectionStop("avtOSPRayRayTracer", "Execute", timingIdx,
                                 "Serial-Composite: Sort with the Largest "
                                 "z First");

        // Blend Images
        ospray::CheckSectionStart("avtOSPRayRayTracer", "Execute", timingIdx,
                                  "Serial-Composite: Blend Images");
#endif
        compositedW = renderingExtents[1] - renderingExtents[0];
        compositedH = renderingExtents[3] - renderingExtents[2];
        compositedExtents[0] = renderingExtents[0];
        compositedExtents[1] = renderingExtents[0] + compositedW;
        compositedExtents[2] = renderingExtents[2];
        compositedExtents[3] = renderingExtents[2] + compositedH;

        if (PAR_Rank() == 0) {
            compositedData = new float[compositedW * compositedH * 4]();
        }

        for (int i=0; i<numPatches; i++)
        {
            ImgMetaData currImgMeta = allPatchMeta[i];
            ImgData     currImgData;
            // Get a shallow copy of the data which must be deleted as
            // the extractor derefences it.
            currImgData.imagePatch = nullptr;
            extractor.GetAndDelImgData(currImgMeta.patchNumber, currImgData);

            const float* currData = currImgData.imagePatch;
            const int currExtents[4] =
                {currImgMeta.screen_ll[0], currImgMeta.screen_ur[0],
                 currImgMeta.screen_ll[1], currImgMeta.screen_ur[1]};

            avtOSPRayImageCompositor::BlendBackToFront(currData,
                                                       currExtents,
                                                       compositedData,
                                                       compositedExtents);

            if (currImgData.imagePatch != nullptr) {
                delete[] currImgData.imagePatch;
                currImgData.imagePatch = nullptr;
            }
        }

        allPatchMeta.clear();
        allPatchData.clear();

#ifdef COMMENT_OUT_FOR_NOW
        ospray::CheckSectionStop("avtOSPRayRayTracer", "Execute", timingIdx,
                                 "Serial-Composite: Blend Images");
        // Qi debug - Memory
        ospray::CheckMemoryHere("[avtOSPRayRayTracer] Execute "
                                "Sequential Compositing Done",
                                "debug5");
#endif
    }

    // Final Composition for Displaying
    if (PAR_Rank() == 0)
    {
        avtImage_p finalImage = new avtImage(this);
        vtkImageData *finalVTKImage =
            avtImageRepresentation::NewImage(screen[0], screen[1]);
        finalImage->GetImage() = finalVTKImage;
        unsigned char *finalImageBuffer =
            finalImage->GetImage().GetRGBBuffer();

#ifdef COMMENT_OUT_FOR_NOW
        ospray::CompositeBackground(screen, compositedExtents,
                                    compositedW, compositedH,
                                    compositedData, opaqueImageData,
                                    opaqueImageZB, finalImageBuffer);
#endif
        finalVTKImage->Delete();
        SetOutput(finalImage);
    }

    if (compositedData != nullptr)
    {
        delete [] compositedData;
        compositedData = nullptr;
    }

    debug5 << "[avtOSPRayRayTracer] Raycasting OSPRay is Done !" << std::endl;

    // Clean up
    screen_to_model_transform->Delete();
    model_to_screen_transform->Delete();
    screen_to_camera_transform->Delete();

#ifdef COMMENT_OUT_FOR_NOW
    ospray::Finalize();
#endif
}
