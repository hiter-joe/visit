// Copyright (c) Lawrence Livermore National Security, LLC and other VisIt
// Project developers.  See the top-level LICENSE file for dates and other
// details.  No copyright assignment is required to contribute to VisIt.

// ************************************************************************* //
//                            avtOSPRayVoxelExtractor.C                      //
// ************************************************************************* //

#include <avtOSPRayVoxelExtractor.h>

#include <avtCellList.h>
#include <avtVolume.h>

#include <DebugStream.h>
#include <ImproperUseException.h>
#include <StackTimer.h>

#include <vtkCellData.h>
#include <vtkDataArray.h>
#include <vtkMatrix4x4.h>
#include <vtkPointData.h>
#include <vtkRectilinearGrid.h>

#include <complex>


// ****************************************************************************
//  Method: avtOSPRayVoxelExtractor constructor
//
//  Arguments:
//     w     The number of sample points in the x direction (width).
//     h     The number of sample points in the y direction (height).
//     d     The number of sample points in the z direction (depth).
//     vol   The volume to put samples into.
//     cl    The cell list to put cells whose sampling was deferred.
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ****************************************************************************

avtOSPRayVoxelExtractor::avtOSPRayVoxelExtractor(int w, int h, int d,
                                             avtVolume *vol, avtCellList *cl)
    : avtVoxelExtractor(w, h, d, vol, cl)
{
#ifdef COMMENT_OUT_FOR_NOW
    ospray_core = nullptr;
#endif
    model_to_screen_transform = vtkMatrix4x4::New();
    screen_to_model_transform = vtkMatrix4x4::New();

    proc  = 0;
    patch = 0;
    drawn = 0;
    imgDims[0] = imgDims[1] = 0;             // size of the patch
    imgLowerLeft[0] = imgLowerLeft[1] = 0;   // coordinates in the whole image
    imgUpperRight[0] = imgUpperRight[1] = 0; // coordinates in the whole image
    eyeSpaceDepth  = -1;
    clipSpaceDepth = -1;

    finalImage = nullptr;                         // the image data
}


// ****************************************************************************
//  Method: avtOSPRayVoxelExtractor destructor
//
//  Purpose:
//      Defines the destructor.  Note: this should not be inlined in the header
//      because it causes problems for certain compilers.
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ****************************************************************************

avtOSPRayVoxelExtractor::~avtOSPRayVoxelExtractor()
{
    model_to_screen_transform->Delete();
    screen_to_model_transform->Delete();

    if (finalImage != nullptr)
        delete []finalImage;

    finalImage = nullptr;
}

// ****************************************************************************
//  Method: avtOSPRayVoxelExtractor::Extract
//
//  Purpose:
//      Extracts the grid into the sample points.
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ****************************************************************************

void
avtOSPRayVoxelExtractor::Extract(vtkRectilinearGrid *rgrid,
                std::vector<std::string> &varnames, std::vector<int> &varsizes)
{
    if (gridsAreInWorldSpace || pretendGridsAreInWorldSpace)
    {
        ExtractWorldSpaceGridOSPRay(rgrid, varnames, varsizes);
    }
    else
    {
        const std::string msg =
	    "Attempt to extract an image space grid, "
	    "however, RayCasting OSPRay supports only "
	    "world space grid extraction";
	EXCEPTION1(ImproperUseException, msg);
    }
}


// ****************************************************************************
//  Method: avtOSPRayVoxelExtractor::ExtractWorldSpaceGridOSPRay
//
//  Purpose:
//      Compute region that patch covers
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
//    Alister Maguire, Fri Sep 11 13:02:48 PDT 2020
//    I've updated the ghost zone bounding box calculation so that it
//    excludes the entirety of all ghost zones.
//
// ****************************************************************************

void
avtOSPRayVoxelExtractor::ExtractWorldSpaceGridOSPRay(vtkRectilinearGrid *rgrid,
                 std::vector<std::string> &varnames, std::vector<int> &varsize)
{
    StackTimer t0("Calling avtOSPRayVoxelExtractor::"
                  "ExtractWorldSpaceGridOSPRay");

    // Initialization
#ifdef COMMENT_OUT_FOR_NOW
    ospray::Context* ospray = (ospray::Context*) ospray_core;
#endif
    // Flag to indicate if the patch is drawn or not.
    drawn = false;

    // Register data and early skipping check (not used).
    int w_min;
    int w_max;
    int h_min;
    int h_max;

    {
        StackTimer t1("avtOSPRayVoxelExtractor::ExtractWorldSpaceGridOSPRay "
                      "Register Data (VisIt preparation)");
        // Some of the sampling routines need a chance to pre-process
        // the data.  As such, register the grid here.
        // Stores the values in a structure so that it can be used
        RegisterGrid(rgrid, varnames, varsize);

        // Determine what image range wanted for this iteration.
        w_min = restrictedMinWidth;
        w_max = restrictedMaxWidth + 1;
        h_min = restrictedMinHeight;
        h_max = restrictedMaxHeight + 1;
        imgWidth = imgHeight = 0;

        // Let's find out if this range can even intersect the dataset.
        // If not, just skip it.
        // if (!FrustumIntersectsGrid(w_min, w_max, h_min, h_max))
        // {
        //     return;
        // }
    }

    // Obtain the data pointers and the ghost region information.
    void* volumePointer = nullptr;
    int   volumeDataType;
    int nX = 0, nY = 0, nZ = 0;
    bool ghost_bound[6] = {false};
    double volumeCube[6];
    {
        StackTimer t1("avtOSPRayVoxelExtractor::ExtractWorldSpaceGridOSPRay "
                      "Compute metadata & ghost boundary "
                      "(Pre-OSPRay preparation)");
        // Calculate patch dimensions for point array and cell array.
        //   This check to determine if the patch is cell data or
        //   point data. Assume that the cell dataset has a higher
        //   priority over the point data.
        debug5 << "[avtOSPRayVoxelExtractor] "
               << "ncell_arrays " << ncell_arrays << " "
               << "npt_arrays "   << npt_arrays << std::endl;

        if (ncell_arrays > 0)
        {
            const size_t varIdx = ncell_arrays - 1;
            // const size_t varIdx =
            //   std::find(varnames.begin(), varnames.end(), ospray->var) -
            //   varnames.begin();
            if (DebugStream::Level5() )
            {
                debug5 << "[avtOSPRayVoxelExtractor] Cell Dataset "
                       << std::endl << std::endl;

                for (int i = 0; i < ncell_arrays; ++i)
                    debug5 << "  variable_name: "
                           << rgrid->GetCellData()->GetArray(i)->GetName()
                           << std::endl
                           << "  idx_cell_arrays: " << i << std::endl
                           << "  cell_index["    << i << "] "
                           << cell_index[i]      << std::endl
                           << "  cell_size["     << i << "] "
                           << cell_size[i]       << std::endl
                           << "  cell_vartypes[" << i << "] "
                           << cell_vartypes[i]   << std::endl << std::endl;

#ifdef COMMENT_OUT_FOR_NOW
                if (rgrid->GetCellData()->GetArray(varIdx)->GetName() !=
                    ospray->GetVariableName())
                {
                    ospray::Warning("Error: The primary variable " +
                                    ospray->GetVariableName() +
                                    " was not found. The variable " +
                                    rgrid->GetCellData()->GetArray(varIdx)->GetName() +
                                    "was found instead. As such, "
                                    "the rendered volume might be wrong.");
                }

                if (cell_size[varIdx] != 1)
                {
                    ospray::Warning("Error: A non-scalar variable " +
                                    ospray->GetVariableName() +
                                    " of length " +
                                    std::to_string(cell_size[varIdx]) +
                                    " was found.");
                }
#endif
            }

            nX = dims[0] - 1;
            nY = dims[1] - 1;
            nZ = dims[2] - 1;

            volumePointer  = cell_arrays[varIdx];
            volumeDataType = cell_vartypes[varIdx];
        }
        else if (npt_arrays > 0)
        {
            const size_t varIdx = npt_arrays - 1;
            // const size_t varIdx =
            //   std::find(varnames.begin(), varnames.end(), ospray->var) -
            //   varnames.begin();
            if (DebugStream::Level5() )
            {
                debug5 << "[avtOSPRayVoxelExtractor] Point Dataset "
                       << std::endl << std::endl;

                for (int i = 0; i < npt_arrays; ++i)
                    debug5 << "  variable_name: "
                           << rgrid->GetPointData()->GetArray(i)->GetName()
                           << std::endl
                           << "  idx_pt_arrays: " << i << std::endl
                           << "  pt_index["    << i << "] "
                           << pt_index[i]      << std::endl
                           << "  pt_size["     << i << "] "
                           << pt_size[i]       << std::endl
                           << "  pt_vartypes[" << i << "] "
                           << pt_vartypes[i]   << std::endl << std::endl;

#ifdef COMMENT_OUT_FOR_NOW
                if (rgrid->GetPointData()->GetArray(varIdx)->GetName() !=
                    ospray->GetVariableName())
                {
                    ospray::Warning("Error: The primary variable " +
                                    ospray->GetVariableName() +
                                    " was not found. The variable " +
                                    rgrid->GetPointData()->GetArray(varIdx)->GetName() +
                                    "was found instead. As such, "
                                    "the rendered volume might be wrong.");
                }

                if (pt_size[varIdx] != 1)
                {
                    ospray::Warning("Error: A non-scalar variable " +
                                    ospray->GetVariableName() +
                                    " of length " +
                                    std::to_string(pt_size[varIdx]) +
                                    " was found.");
                }
#endif
            }

            nX = dims[0];
            nY = dims[1];
            nZ = dims[2];

            volumePointer  = pt_arrays[varIdx];
            volumeDataType = pt_vartypes[varIdx];
        }
        else
        {
	    const std::string msg =
	        "The dataset found is neither nodal nor zonal data. "
	        "OSPRay does not know how to handle it.";
	    EXCEPTION1(ImproperUseException, msg);
        }

        debug5 << "[avtOSPRayVoxelExtractor] patch dimension "
               << nX << " " << nY << " " << nZ << std::endl;
        // Calculate ghost region boundaries
        //   ghost_boundaries is an array to indicate if the patch contains
        //   any ghost regions in six different directions
        // Here I assume the patch is larger than 3-cube
        // If not then you might want to dig into this code and see if
        // there will be any special boundary cases
        //
        // debug5 << "VAR: ghost value " << (int)ghosts[0] << std::endl;

        if (ghosts != nullptr) {
            int gnX = 0, gnY = 0, gnZ = 0;
            gnX = dims[0] - 1;
            gnY = dims[1] - 1;
            gnZ = dims[2] - 1;
            for (int y = 1; y < (gnY-1); ++y) {
                for (int z = 1; z < (gnZ-1); ++z) {
                    if (!ghost_bound[0]) {
                        if (ghosts[z*gnY*gnX+y*gnX        ] != 0)
                            { ghost_bound[0] = true; }
                    }
                    if (!ghost_bound[3]) {
                        if (ghosts[z*gnY*gnX+y*gnX+(gnX-1)] != 0)
                            { ghost_bound[3] = true; }
                    }
                    if (ghost_bound[0] && ghost_bound[3]) { break; }
                }
            }
            for (int x = 1; x < (gnX-1); ++x) {
                for (int z = 1; z < (gnZ-1); ++z) {
                    if (!ghost_bound[1]) {
                        if (ghosts[z*gnY*gnX            +x] != 0)
                            { ghost_bound[1] = true; }
                    }
                    if (!ghost_bound[4]) {
                        if (ghosts[z*gnY*gnX+(gnY-1)*gnX+x] != 0)
                            { ghost_bound[4] = true; }
                    }
                    if (ghost_bound[1] && ghost_bound[4]) { break; }
                }
            }
            for (int x = 1; x < (gnX-1); ++x) {
                for (int y = 1; y < (gnY-1); ++y) {
                    if (!ghost_bound[2]) {
                        if (ghosts[                y*gnX+x] != 0)
                            { ghost_bound[2] = true; }
                    }
                    if (!ghost_bound[5]) {
                        if (ghosts[(gnZ-1)*gnY*gnX+y*gnX+x] != 0)
                            { ghost_bound[5] = true; }
                    }
                    if (ghost_bound[2] && ghost_bound[5]) { break; }
                }
            }
        }

        // Data bounding box
        volumeCube[0] = X[0];
        volumeCube[2] = Y[0];
        volumeCube[4] = Z[0];

        if (ncell_arrays > 0)
        {
            volumeCube[1] = X[nX];
            volumeCube[3] = Y[nY];
            volumeCube[5] = Z[nZ];
        }
        else
        {
            volumeCube[1] = X[nX-1];
            volumeCube[3] = Y[nY-1];
            volumeCube[5] = Z[nZ-1];
        }
    }

    // Determine the screen size of the patch being processed
    int patchScreenExtents[4];
    double patch_center[3];
    double patch_depth;
    {
        StackTimer t1("avtOSPRayVoxelExtractor::ExtractWorldSpaceGridOSPRay "
                      "Get screen size of the patch (Pre-OSPRay preparation)");
        double renderingDepthsExtents[2];
#ifdef COMMENT_OUT_FOR_NOW
        ospray::ProjectWorldToScreenCube(volumeCube, w_max, h_max,
                                         viewInfo.imagePan, viewInfo.imageZoom,
                                         model_to_screen_transform,
                                         patchScreenExtents,
                                         renderingDepthsExtents);
#endif
        xMin = patchScreenExtents[0];
        xMax = patchScreenExtents[1];
        yMin = patchScreenExtents[2];
        yMax = patchScreenExtents[3];
        debug5 << "[avtOSPRayVoxelExtractor] patch ghost bounds:"
               << "   " << ghost_bound[0] << " " << ghost_bound[3]
               << " | " << ghost_bound[1] << " " << ghost_bound[4]
               << " | " << ghost_bound[2] << " " << ghost_bound[5]
               << std::endl;
        patch_center[0] = (volumeCube[0] + volumeCube[1])/2.0;
        patch_center[1] = (volumeCube[2] + volumeCube[3])/2.0;
        patch_center[2] = (volumeCube[4] + volumeCube[5])/2.0;
        patch_depth = // use the norm of patch center as patch depth
            std::sqrt((patch_center[0]-viewInfo.camera[0])*
                      (patch_center[0]-viewInfo.camera[0])+
                      (patch_center[1]-viewInfo.camera[1])*
                      (patch_center[1]-viewInfo.camera[1])+
                      (patch_center[2]-viewInfo.camera[2])*
                      (patch_center[2]-viewInfo.camera[2]));
        eyeSpaceDepth = patch_depth;
        clipSpaceDepth = renderingDepthsExtents[0];
    }

    // Create the framebuffer
    {
        StackTimer t1("avtOSPRayVoxelExtractor::ExtractWorldSpaceGridOSPRay "
                      "Create ImgArray (Pre-OSPRay preparation)");
        debug5 << "[avtOSPRayVoxelExtractor] patch extents "
               << xMin << " " << xMax << " "
               << yMin << " " << yMax << std::endl;
        if (xMin < renderingExtents[0]) { xMin = renderingExtents[0]; }
        if (yMin < renderingExtents[2]) { yMin = renderingExtents[2]; }
        if (xMax > renderingExtents[1]) { xMax = renderingExtents[1]; }
        if (yMax > renderingExtents[3]) { yMax = renderingExtents[3]; }

        imgWidth  = xMax-xMin;
        imgHeight = yMax-yMin;
        finalImage = new float[((imgWidth)*4) * imgHeight];
    }

    // Render using OSPRay
    double volumePBox[6];
    double volumeBBox[6];
    {
        StackTimer t1("avtOSPRayVoxelExtractor::ExtractWorldSpaceGridOSPRay "
                      "Using OSPRay");
        {
            StackTimer t2("avtOSPRayVoxelExtractor::"
                          "ExtractWorldSpaceGridOSPRay "
                          "OSPRay bbox and clip (OSPRay preparation)");
            // Shift grid and make it cel centered for cell data.  For
            // cell centered data, The voxel is on the grid's left
            // boundary.
            volumePBox[0] = X[0];
            volumePBox[1] = Y[0];
            volumePBox[2] = Z[0];

            if (ncell_arrays > 0)
            {
                /* Zonal data needs to occupy a whole cell */
                volumePBox[3] = X[nX];
                volumePBox[4] = Y[nY];
                volumePBox[5] = Z[nZ];
            }
            else
            {
                volumePBox[3] = X[nX-1];
                volumePBox[4] = Y[nY-1];
                volumePBox[5] = Z[nZ-1];
            }

            // Compute boundingbox and clipping plane for ospray
            if (ncell_arrays > 0)
            {
                // The idea here is to create a bounding box that
                // excludes the ghost zones. The bounding box will
                // later be used by OSPRay to determine clipping
                // bounds, and the portion of the volume that contains
                // ghosts zones will be clipped away.
                volumeBBox[0] = ghost_bound[0] ? (X[1]): volumePBox[0];
                volumeBBox[1] = ghost_bound[1] ? (Y[1]) : volumePBox[1];
                volumeBBox[2] = ghost_bound[2] ? (Z[1]) : volumePBox[2];
                volumeBBox[3] = ghost_bound[3] ? (X[nX-1]): volumePBox[3];
                volumeBBox[4] = ghost_bound[4] ? (Y[nY-1]): volumePBox[4];
                volumeBBox[5] = ghost_bound[5] ? (Z[nZ-1]): volumePBox[5];
            }
            else
            {
                volumeBBox[0] = ghost_bound[0] ? X[1] : volumePBox[0];
                volumeBBox[1] = ghost_bound[1] ? Y[1] : volumePBox[1];
                volumeBBox[2] = ghost_bound[2] ? Z[1] : volumePBox[2];
                volumeBBox[3] = ghost_bound[3] ? X[nX-2] : volumePBox[3];
                volumeBBox[4] = ghost_bound[4] ? Y[nY-2] : volumePBox[4];
                volumeBBox[5] = ghost_bound[5] ? Z[nZ-2] : volumePBox[5];
            }

            debug5 << "[avtOSPRayVoxelExtractor] patch data position:"
                   << " " << volumePBox[0]
                   << " " << volumePBox[1]
                   << " " << volumePBox[2]
                   << " |"
                   << " " << volumePBox[3]
                   << " " << volumePBox[4]
                   << " " << volumePBox[5]
                   << std::endl;
            debug5 << "[avtOSPRayVoxelExtractor] patch data bbox:"
                   << " " << volumeBBox[0]
                   << " " << volumeBBox[1]
                   << " " << volumeBBox[2]
                   << " |"
                   << " " << volumeBBox[3]
                   << " " << volumeBBox[4]
                   << " " << volumeBBox[5]
                   << std::endl;
        }

        // Create volume and model
        {
            StackTimer t2("avtOSPRayVoxelExtractor::"
                          "ExtractWorldSpaceGridOSPRay "
                          "OSPRay Create Volume");
#ifdef COMMENT_OUT_FOR_NOW
            ospray->SetupPatch(patch, volumeDataType,
                               (size_t)nX * (size_t)nY * (size_t)nZ,
                               volumePointer, X, Y, Z, nX, nY, nZ,
                               volumePBox, volumeBBox);
#endif
        }
        // Render Volume
        {
            StackTimer t2("avtOSPRayVoxelExtractor::"
                          "ExtractWorldSpaceGridOSPRay "
                          "OSPRay Render Volume");

            if ((scalarRange[1] >= tFVisibleRange[0]) &&
                (scalarRange[0] <= tFVisibleRange[1]))
            {
#ifdef COMMENT_OUT_FOR_NOW
                ospray->RenderPatch(patch, xMin, xMax, yMin, yMax,
                                    imgWidth, imgHeight, finalImage);
#endif
                drawn = true;
            }
        }
    }

    //=======================================================================//
    // Send rays
    //=======================================================================//
    imgDims[0] = imgWidth;
    imgDims[1] = imgHeight;
    imgLowerLeft[0] = xMin;
    imgLowerLeft[1] = yMin;
    imgUpperRight[0] = xMax;
    imgUpperRight[1] = yMax;

    //=======================================================================//
    // Deallocate memory if not used
    //=======================================================================//
    if (drawn == false)
    {
        if (finalImage != nullptr)
        {
            delete []finalImage;
            finalImage = nullptr;
        }
    }
}


// ****************************************************************************
//  Method: avtOSPRayVoxelExtractor::getImageDimensions
//
//  Purpose:
//      Transfers the metadata of the patch
//
//  Programmer: Pascal Grosset
//  Creation:   August 14, 2016
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ****************************************************************************

void
avtOSPRayVoxelExtractor::GetImageDimensions(bool &inUse,
					    int dims[2],
                                            int screen_ll[2],
                                            int screen_ur[2],
                                            float &eyeDepth,
                                            float &clipDepth) const
{
    inUse = drawn;
    dims[0] = imgDims[0];
    dims[1] = imgDims[1];
    screen_ll[0] = imgLowerLeft[0];
    screen_ll[1] = imgLowerLeft[1];
    screen_ur[0] = imgUpperRight[0];
    screen_ur[1] = imgUpperRight[1];
    eyeDepth  = eyeSpaceDepth;
    clipDepth = clipSpaceDepth;
}

// ****************************************************************************
//  Method: avtOSPRayVoxelExtractor::getComputedImage
//
//  Purpose:
//      Allocates space to the pointer address and copy the image generated
//      to it
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ****************************************************************************

void
avtOSPRayVoxelExtractor::GetComputedImage(float *image)
{
    memcpy(image, finalImage, imgDims[0]*4*imgDims[1]*sizeof(float));

    if (finalImage != nullptr)
        delete []finalImage;

    finalImage = nullptr;
}
