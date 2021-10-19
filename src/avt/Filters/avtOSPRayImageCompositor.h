// Copyright (c) Lawrence Livermore National Security, LLC and other VisIt
// Project developers.  See the top-level LICENSE file for dates and other
// details.  No copyright assignment is required to contribute to VisIt.

// *************************************************************************//
//                          avtOSPRayImageCompositor.h                      //
// *************************************************************************//

#ifndef AVT_OSPRAY_IMAGE_COMPOSITOR_H
#define AVT_OSPRAY_IMAGE_COMPOSITOR_H

#include <avtOSPRayCommon.h>
#include <avtParallel.h>

#include <map>
#include <vector>

// ***************************************************************************
//  Class: avtOSPRayImageCompositor
//
//  Purpose:
//      Does the composition for the ray casting
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
// ***************************************************************************

class avtOSPRayIC_Implementation
{
 public:
    avtOSPRayIC_Implementation(int mpiSize, int mpiRank) {};
    virtual ~avtOSPRayIC_Implementation() {};
    virtual void Init (int W, int H) = 0;
    virtual void SetTile (const float* data, 
                          const int* extents, 
                          const float& depth) = 0;
    virtual void Composite (float*& output) = 0;
};

// ***************************************************************************
//  Class: avtOSPRayImageCompositor
//
//  Purpose:
//      Does the composition for the ray casting
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
// ***************************************************************************

class avtOSPRayImageCompositor
{ 
 public:    
    avtOSPRayImageCompositor();
    ~avtOSPRayImageCompositor();

    virtual const char *GetType(void)
    {
        return "avtOSPRayImageCompositor";
    }

    virtual const char *GetDescription(void) 
    {
        return "Doing compositing for ray casting OSPRay";
    }

    float* GetFinalImageBuffer () { return finalImage; }
    int GetParSize ()             { return mpiSize;   }
    int GetParRank ()             { return mpiRank;   }

    // These functions need to be static as they are called from
    // avtOSPRayIC_OneNode::Composite
    static void BlendFrontToBack(const float *, const int srcExtents[4],
                                 const int blendExtents[4], 
                                 float *&, const int dstExtents[4]);
    static void BlendBackToFront(const float *, const int srcExtents[4], 
                                 const int blendExtents[4], 
                                 float *&, const int dstExtents[4]);
    static void BlendFrontToBack(const float *, const int srcExtents[4], 
                                 float *&, const int dstExtents[4]);
    static void BlendBackToFront(const float *, const int srcExtents[4], 
                                 float *&, const int dstExtents[4]);
    void Barrier();

 private:
    //----------------------------------------------------------------------//
    static void ColorImage(float *&, const int, const int, 
                           const float color[4]);
    static void PlaceImage(const float *, const int srcExtents[4], 
                           float *&, const int dstExtents[4]);
    static void BlendWithBackground(float *&, const int extents[4],
                                    const float bgColor[4]);
 public:
    //----------------------------------------------------------------------//
    // Different Algorithms
    //----------------------------------------------------------------------//

    //----------------------------------------------------------------------//
    // IceT: (export OSPRAY_USE_ICET=1)
    //   Use IceT for compositing. This method only supports the case where 
    //   each rank produces only one tile.
    // Algorithms:
    //   Reduce
    //   Binary Swap
    //    
    //----------------------------------------------------------------------//
    bool IceTValid();
    void IceTInit(int W, int H);
    void IceTSetTile(const float*, const int*, const float&);
    void IceTComposite(float*&);

    //----------------------------------------------------------------------//
    // OneNode
    //   There is only one VisIt process
    //----------------------------------------------------------------------//
    bool OneNodeValid();
    void OneNodeInit(int W, int H);
    void OneNodeSetTile(const float*, const int*, const float&);
    void OneNodeComposite(float*&);

    //----------------------------------------------------------------------//
    // Serial
    //----------------------------------------------------------------------//
    //bool SerialValid();
    //void SerialInit(int W, int H);
    //void SerialSetTile(const float*, const int*, const float&);
    //void SerialComposite(float*&);

 private:
    // Basic MPI information
    int mpiSize; // total number of processes (# of ranks)
    int mpiRank; // my rank id
    // TODO this communicator shouldnt be in charge of this variable
    // Final image is here
    float *finalImage;
    // Image Compisition Implementation
    avtOSPRayIC_Implementation* compositor;

    // CLEAN UP BELOW
 private:
    //----------------------------------------------------------------------//
    void UpdateBoundingBox(int currentBoundingBox[4], 
                           const int imageExtents[4]);
    //----------------------------------------------------------------------//

    //----------------------------------------------------------------------//
    // Both currently unused but good for simple testing
    //----------------------------------------------------------------------//
 private:
    void GatherDepthAtRoot(const int, const float *, int &, int *&, float *&);
 public:
    void SerialDirectSend
        (int, float*, int*, float*, float bgColor[4], int, int);

    // Parallel Direct Send
 public:
    void RegionAllocation(int *&);
    int  ParallelDirectSendManyPatches
        (const std::multimap<int, ImgData>&,
	 const std::vector<ImgMetaData>&,
	 int, int*, int, int tags[2], int fullImageExtents[4]);

 private:
    // flags for patch
    int totalPatches;
    bool compositingDone;

    // image sizing for compositing
    int maxRegionHeight;
    int regularRegionSize;
    std::vector<int> regionRankExtents;

    //----------------------------------------------------------------------//
      
    void computeRegionExtents(int numRanks, int height);
        
    int getRegularRegionSize(){ return regularRegionSize; } 
    int getRegionStart(int region){ return regionRankExtents[region*3+0]; }
    int getRegionEnd(int region){ return regionRankExtents[region*3+1]; }
    int getRegionSize(int region){ return regionRankExtents[region*3+2]; }
    int getMaxRegionHeight(){ return maxRegionHeight; }

    int getScreenRegionStart(int region, int screenImgMinY, int screenImgMaxY)
    {
      return CLAMP(getRegionStart(region)+screenImgMinY,
                   screenImgMinY, screenImgMaxY);
    }

    int getScreenRegionEnd(int region, int screenImgMinY, int screenImgMaxY)
    {
      return CLAMP(getRegionEnd(region)+screenImgMinY,
                   screenImgMinY, screenImgMaxY);
    }

 public:
    // Get the final composited image
    void getcompositedImage(int imgBufferWidth, int imgBufferHeight,
			    unsigned char *wholeImage);


    int findRegionsForPatch(int patchExtents[4], int screenProjectedExtents[4],
			    int numRegions, int &from, int &to);


    void parallelDirectSend(float *imgData, int imgExtents[4], int region[], int numRegions, int tags[2], int fullImageExtents[4]);     
    void gatherImages(int regionGather[], int numToRecv, float * inputImg, int imgExtents[4], int boundingBox[4], int tag, int fullImageExtents[4], int myRegionHeight);

 public:
    // TODO: Remove all public fields
    int finalImageExtents[4];
    int finalBB[4];
    float *intermediateImage; // Intermediate image, e.g. in parallel direct send
    int intermediateImageExtents[4];
    int intermediateImageBBox[4];
};

#endif
