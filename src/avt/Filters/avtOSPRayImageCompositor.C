// Copyright (c) Lawrence Livermore National Security, LLC and other VisIt
// Project developers.  See the top-level LICENSE file for dates and other
// details.  No copyright assignment is required to contribute to VisIt.

// ************************************************************************ //
//                        avtOSPRayImageCompositor.C                         //
// ************************************************************************ //

#include <avtOSPRayImageCompositor.h>
#include <avtParallel.h>

#include <DebugStream.h>
#include <ImproperUseException.h>

#ifdef PARALLEL
#  include <mpi.h>

#  ifdef VISIT_OSPRAY_ICET
#    include <IceT.h>
#    include <IceTMPI.h>
#  endif
#endif

#include <algorithm>
#include <iostream>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include <cstdlib>

// ***************************************************************************
// Threaded Blending
// ***************************************************************************

bool CheckThreadedBlend_Communicator()
{
    bool use = true;
    const char* env_use = std::getenv("OSPRAY_SERIAL_BLEND");

    if (env_use)
    {
        use = atoi(env_use) <= 0;
    }

    return use;
}

static bool UseThreadedBlend_Communicator = CheckThreadedBlend_Communicator();


// ***************************************************************************
//  Class: avtOSPRayIC_IceT
// ***************************************************************************

class avtOSPRayIC_IceT : public avtOSPRayIC_Implementation
{
public:
    avtOSPRayIC_IceT(int mpiSize, int mpiRank);
    ~avtOSPRayIC_IceT();

    void Init(int, int);
    void SetTile(const float*, const int*, const float&);
    void Composite(float*&);

    static bool Valid();

private:
#if defined(PARALLEL) && defined(VISIT_OSPRAY_ICET)

    IceTInt          screen[2];
    IceTContext      context, prevContext;
    IceTCommunicator comm;
    IceTInt          MPISize;
    IceTInt          MPIRank;
    IceTImage        result;

    static const bool       usage;
    static const IceTDouble identity[16];
    static const IceTFloat  bgColor[4];
    static const IceTEnum   strategy;

    static const float* imgData;
    static int          imgMeta[4];

    static bool     CheckUsage();
    static IceTEnum CheckStrategy();
    static void DrawCallback(const IceTDouble*, const IceTDouble*,
                             const IceTFloat*, const IceTInt*,
                             IceTImage img);
#endif
};

// ***************************************************************************
//  Method: avtOSPRayIC_IceT::Valid
//
//  Purpose: Static method
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

bool avtOSPRayIC_IceT::Valid()
{
#if defined(PARALLEL) && defined(VISIT_OSPRAY_ICET)
    return usage;
#else
    return false;
#endif
}

#if defined(PARALLEL) && defined(VISIT_OSPRAY_ICET)
// ***************************************************************************
//  Method: avtOSPRayIC_IceT::
//
//  Purpose: Static value initialization
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

const bool avtOSPRayIC_IceT::usage = avtOSPRayIC_IceT::CheckUsage();

const IceTDouble avtOSPRayIC_IceT::identity[16] =
{
    IceTDouble(1.0), IceTDouble(0.0), IceTDouble(0.0), IceTDouble(0.0),
    IceTDouble(0.0), IceTDouble(1.0), IceTDouble(0.0), IceTDouble(0.0),
    IceTDouble(0.0), IceTDouble(0.0), IceTDouble(1.0), IceTDouble(0.0),
    IceTDouble(0.0), IceTDouble(0.0), IceTDouble(0.0), IceTDouble(1.0)
};

const IceTFloat avtOSPRayIC_IceT::bgColor[4] =
{
    IceTFloat(0.0f), IceTFloat(0.0f), IceTFloat(0.0f), IceTFloat(0.0f)
};

const IceTEnum avtOSPRayIC_IceT::strategy = avtOSPRayIC_IceT::CheckStrategy();

const float* avtOSPRayIC_IceT::imgData = nullptr;
int          avtOSPRayIC_IceT::imgMeta[4] = {0,0,0,0};


// ***************************************************************************
//  Method: avtOSPRayIC_IceT::CheckUsage
//
//  Purpose: Static method
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

bool avtOSPRayIC_IceT::CheckUsage()
{
    bool use_icet = false;

    const char* env_use_icet = std::getenv("OSPRAY_USE_ICET");

    if (env_use_icet)
    {
        use_icet = atoi(env_use_icet) > 0;
    }

    return use_icet;
}


// ***************************************************************************
//  Method: avtOSPRayIC_IceT::CheckStrategy
//
//  Purpose: Static methods
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

IceTEnum avtOSPRayIC_IceT::CheckStrategy()
{
    if (avtOSPRayIC_IceT::Valid())
    {
        IceTEnum ret;
        int strategy = 3;
        const char* env_icet_strategy = std::getenv("OSPRAY_ICET_STRATEGY");
        if (env_icet_strategy) { strategy = atoi(env_icet_strategy); }
        switch (strategy) {
        case 0:
            ret = ICET_STRATEGY_REDUCE;
            break;
        case 1:
            ret = ICET_SINGLE_IMAGE_STRATEGY_TREE;
            break;
        case 2:
            ret = ICET_SINGLE_IMAGE_STRATEGY_RADIXK;
            break;
        default:
            ret = ICET_SINGLE_IMAGE_STRATEGY_BSWAP;
            break;
        }
        return ret;
    }
    else
    {
        return false;
    }
}


// ***************************************************************************
//  Method: avtOSPRayIC_IceT::DrawCallback
//
//  Purpose: Static method
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

void avtOSPRayIC_IceT::DrawCallback(const IceTDouble*,
                                    const IceTDouble*,
                                    const IceTFloat*,
                                    const IceTInt*,
                                    IceTImage img)
{
    float *o = icetImageGetColorf(img);

    const int outputStride = icetImageGetWidth(img);

    for (int j = 0; j < imgMeta[3]; ++j)
    {
        for (int i = 0; i < imgMeta[2]; ++i)
        {
            const int gIdx = i + imgMeta[0] + (j + imgMeta[1]) * outputStride;
            const int lIdx = i + j * imgMeta[2];

            o[4 * gIdx + 0] = imgData[4 * lIdx + 0];
            o[4 * gIdx + 1] = imgData[4 * lIdx + 1];
            o[4 * gIdx + 2] = imgData[4 * lIdx + 2];
            o[4 * gIdx + 3] = imgData[4 * lIdx + 3];
        }
    }
}
#endif


// ***************************************************************************
//  Method: avtOSPRayIC_IceT::avtOSPRayIC_IceT
//
//  Purpose: Constructor
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

avtOSPRayIC_IceT::avtOSPRayIC_IceT(int mpiSize, int mpiRank)
    : avtOSPRayIC_Implementation(mpiSize, mpiRank)
{
#if defined(PARALLEL) && defined(VISIT_OSPRAY_ICET)
    MPISize = IceTInt(mpiSize);
    MPIRank = IceTInt(mpiRank);

    prevContext = icetGetContext();
    comm = icetCreateMPICommunicator(VISIT_MPI_COMM);
    context = icetCreateContext(comm);
    icetDestroyMPICommunicator(comm);

    // debug
    if (avtOSPRayIC_IceT::Valid() && mpiRank == 0) {
        switch (avtOSPRayIC_IceT::strategy) {
        case 0:
            debug5 << "[avtOSPRayIC_IceT] Strategy Reduce" << std::endl;
            break;
        case 1:
            debug5 << "[avtOSPRayIC_IceT] Strategy Tree" << std::endl;
            break;
        case 2:
            debug5 << "[avtOSPRayIC_IceT] Strategy Radix-k" << std::endl;
            break;
        default:
            debug5 << "[avtOSPRayIC_IceT] Strategy BSwap" << std::endl;
            break;
        }
    }
#endif
}


// ***************************************************************************
//  Method: avtOSPRayIC_IceT::avtOSPRayIC_IceT
//
//  Purpose: Destructor
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

avtOSPRayIC_IceT::~avtOSPRayIC_IceT()
{
#if defined(PARALLEL) && defined(VISIT_OSPRAY_ICET)
    icetDestroyContext(context);
    icetSetContext(prevContext);
#endif
}

// ***************************************************************************
//  Method: avtOSPRayIC_IceT::Init
//
//  Purpose:
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

void avtOSPRayIC_IceT::Init(int W, int H)
{
#if defined(PARALLEL) && defined(VISIT_OSPRAY_ICET)
    if (MPIRank == 0) {
        debug5 << "avtOSPRayIC_IceT::Init Start";
    }

    // Initialization
    screen[0] = W;
    screen[1] = H;

    // Setup IceT parameters
    if (/*ospray::CheckVerbose()*/ false /*|| DebugStream::Level5()*/) {
        icetDiagnostics(ICET_DIAG_FULL);
    }

    icetCompositeMode(ICET_COMPOSITE_MODE_BLEND);
    icetSetColorFormat(ICET_IMAGE_COLOR_RGBA_FLOAT);
    icetSetDepthFormat(ICET_IMAGE_DEPTH_NONE);
    icetEnable(ICET_ORDERED_COMPOSITE);
    icetDisable(ICET_INTERLACE_IMAGES);

    // Safety
    MPI_Barrier(MPI_COMM_WORLD);

    if (MPIRank == 0) {
        debug5 << " ... Done" << std::endl;
    }
#endif
}

// ***************************************************************************
//  Method: avtOSPRayIC_IceT::SetTile
//
//  Purpose:
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

void avtOSPRayIC_IceT::SetTile(const float* d,
                               const int*   e, /* xmin, xmax, ymin, ymax */
                               const float& z)
{
#if defined(PARALLEL) && defined(VISIT_OSPRAY_ICET)
    // Gather the depths
    if (MPIRank == 0) {
        debug5 << "avtOSPRayIC_IceT::SetTile Gather Depth";
    }

    std::vector<float>   all_depths(MPISize);
    std::vector<IceTInt> all_orders(MPISize);

    MPI_Allgather(&z, 1, MPI_FLOAT, all_depths.data(), 1,
                  MPI_FLOAT, MPI_COMM_WORLD);

    if (MPIRank == 0)
    {
        debug5 << " ... Done" << std::endl;
    }

    // Sort the depths in compositing order
    if (MPIRank == 0) {
        debug5 << "avtOSPRayIC_IceT::SetTile Sort Depths";
    }

    std::multimap<float,int> ordered_depths;
    for (int i = 0; i < MPISize; ++i)
    {
        ordered_depths.insert(std::pair<float, int>(all_depths[i], i));
    }

    int i = 0;
    for (std::multimap<float,int>::iterator it = ordered_depths.begin();
         it != ordered_depths.end(); ++it)
    {
        all_orders[i] = (*it).second;
        ++i;
    }

    icetCompositeOrder(all_orders.data());

    if (MPIRank == 0)
    {
        debug5 << " ... Done" << std::endl;
    }

    // Set IceT Tile Information
    icetResetTiles();
    icetAddTile(0, 0, screen[0], screen[1], 0);
    icetPhysicalRenderSize(screen[0], screen[1]);

    // Composite Stratagy
    if (strategy == ICET_STRATEGY_REDUCE)
    {
        icetStrategy(ICET_STRATEGY_REDUCE);
    }
    else
    {
        icetStrategy(ICET_STRATEGY_SEQUENTIAL);
        icetSingleImageStrategy(strategy);
    }

    // Bounding Box
    icetBoundingBoxf(((float) e[0]   /(screen[0]-1) - 0.5f) * 2.f,
                     ((float)(e[1]-1)/(screen[0]-1) - 0.5f) * 2.f,
                     ((float) e[2]   /(screen[1]-1) - 0.5f) * 2.f,
                     ((float)(e[3]-1)/(screen[1]-1) - 0.5f) * 2.f,
                     0.0, 0.0);

    // Compose
    avtOSPRayIC_IceT::imgData = d;
    avtOSPRayIC_IceT::imgMeta[0] = e[0];
    avtOSPRayIC_IceT::imgMeta[1] = e[2];
    avtOSPRayIC_IceT::imgMeta[2] = e[1] - e[0];
    avtOSPRayIC_IceT::imgMeta[3] = e[3] - e[2];
    icetDrawCallback(DrawCallback);
#endif
}

// ***************************************************************************
//  Method: avtOSPRayIC_IceT::Composite
//
//  Purpose:
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

void avtOSPRayIC_IceT::Composite(float*& output)
{
#if defined(PARALLEL) && defined(VISIT_OSPRAY_ICET)
    if (MPIRank == 0)
    {
        debug5 << "avtOSPRayIC_IceT::Composite";
    }

    result = icetDrawFrame(identity, identity, bgColor);

    if (MPIRank == 0)
    {
        debug5 << " ... Done" << std::endl;
    }

    if (MPIRank == 0)
    {
        icetImageCopyColorf(result, output, ICET_IMAGE_COLOR_RGBA_FLOAT);
    }
#endif
}

// ***************************************************************************
//  End Class: avtOSPRayIC_IceT
// ***************************************************************************


// ***************************************************************************
//  Begin Class: avtOSPRayIC_OneNode
// ***************************************************************************

struct MetaData_OneNode {
    int id;         // id of the patch on that processor
    int dims[2];    // height, width
    int extents[4]; // (lower left) (upper right)
    float z;        // camera space depth of the patch (average)
};

class avtOSPRayIC_OneNode : public avtOSPRayIC_Implementation
{
public:
    avtOSPRayIC_OneNode(int mpiSize, int mpiRank);
    ~avtOSPRayIC_OneNode();

    static bool Valid();

    void Init(int, int);
    void SetTile(const float*, const int*, const float&);
    void Composite(float*&);

private:
    static bool Predicate(MetaData_OneNode const& before,
                          MetaData_OneNode const& after);

    std::vector<MetaData_OneNode> allPatchMeta;
    std::vector<const float*>     allPatchData;
    int fullExtents[4];
};

// ***************************************************************************
//  Method: avtOSPRayIC_OneNode::Valid
//
//  Purpose: Static method
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

bool avtOSPRayIC_OneNode::Valid()
{
    return true;
 };

// ***************************************************************************
//  Method: avtOSPRayIC_OneNode::Predicate
//
//  Purpose: Static method for z sorting
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

bool avtOSPRayIC_OneNode::Predicate(MetaData_OneNode const& before,
                                    MetaData_OneNode const& after)
{
    return before.z > after.z;
}

// ***************************************************************************
//  Method: avtOSPRayIC_OneNode::avtOSPRayIC_OneNode
//
//  Purpose: Constructor
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

avtOSPRayIC_OneNode::avtOSPRayIC_OneNode(int mpiSize, int mpiRank)
    : avtOSPRayIC_Implementation(mpiSize, mpiRank)
{
}


// ***************************************************************************
//  Method: avtOSPRayIC_OneNode::~avtOSPRayIC_OneNode
//
//  Purpose: Destructor
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

avtOSPRayIC_OneNode::~avtOSPRayIC_OneNode()
{
    allPatchMeta.clear();
    allPatchData.clear();
};


// ***************************************************************************
//  Method: avtOSPRayIC_OneNode::Init
//
//  Purpose: Intialize the full extents
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

void avtOSPRayIC_OneNode::Init(int W, int H)
{
    fullExtents[0] = 0;
    fullExtents[1] = W;
    fullExtents[2] = 0;
    fullExtents[3] = H;
};


// ***************************************************************************
//  Method: avtOSPRayIC_OneNode::SetTile
//
//  Purpose: Set the patch data from each tile
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

void avtOSPRayIC_OneNode::SetTile(const float* d,
                                  const int*   e,
                                  const float& z)
{
    MetaData_OneNode m;
    m.id = allPatchMeta.size();
    m.dims[0] = e[1] - e[0];
    m.dims[1] = e[3] - e[2];
    m.extents[0] = e[0];
    m.extents[1] = e[1];
    m.extents[2] = e[2];
    m.extents[3] = e[3];
    m.z = z;

    allPatchMeta.push_back(m);
    allPatchData.push_back(d);
}


// ***************************************************************************
//  Method: avtOSPRayIC_OneNode::Composite
//
//  Purpose: Composite the patches after sorting.
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

void avtOSPRayIC_OneNode::Composite(float*& output)
{
    // Sort the patches with the largest z first
    std::sort(allPatchMeta.begin(), allPatchMeta.end(),
              &(avtOSPRayIC_OneNode::Predicate));

    for (int i=0; i<allPatchMeta.size(); ++i)
    {
        MetaData_OneNode m = allPatchMeta[i];
        avtOSPRayImageCompositor::BlendBackToFront(allPatchData[m.id],
                                                   m.extents,
                                                   output,
                                                   fullExtents);
    }
}

// ***************************************************************************
//  End Class: avtOSPRayIC_OneNode
// ***************************************************************************


// ***************************************************************************
//  Begin Class: avtOSPRayIC_Serial
// ***************************************************************************

// class avtOSPRayIC_Serial : public avtOSPRayIC_Implementation
// {
// public:
//     avtOSPRayIC_Serial(int mpiSize, int mpiRank);
//     ~avtOSPRayIC_Serial();
//     void Init(int, int);
//     void SetTile(const float*, const int*, const float&);
//     void Composite(float*&);
//     static bool Valid();
// };

// bool avtOSPRayIC_Serial::Valid() { return true; };

// ***************************************************************************
//  End Class: avtOSPRayIC_Serial
// ***************************************************************************


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::avtOSPRayImageCompositor
//
//  Purpose: Constructor
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

avtOSPRayImageCompositor::avtOSPRayImageCompositor()
{
#ifdef PARALLEL
    MPI_Comm_size(VISIT_MPI_COMM, &mpiSize);
    MPI_Comm_rank(VISIT_MPI_COMM, &mpiRank);
#else
    mpiSize = 1;
    mpiRank = 0;
#endif
    finalImage = nullptr;
    compositor = nullptr;

    // Debug
    if (mpiRank == 0) {
        if (!UseThreadedBlend_Communicator) {
            debug5 << "[avtOSPRayImageCompositor] "
                   << "Not Using Multi-Threading for Blending"
                   << std::endl;
        } else {
            debug5 << "[avtOSPRayImageCompositor] "
                   << "Using Multi-Threading for Blending"
                   << std::endl;
        }
        if (!avtOSPRayIC_IceT::Valid()) {
            debug5 << "[avtOSPRayImageCompositor] "
                   << "Not Using IceT for Image Compositing"
                   << std::endl;
        } else {
            debug5 << "[avtOSPRayImageCompositor] "
                   << "Using IceT for Image Compositing"
                   << std::endl;
        }
    }

    intermediateImageExtents[0] = intermediateImageExtents[1] = 0.0;
    intermediateImageExtents[2] = intermediateImageExtents[3] = 0.0;
    intermediateImageBBox[0] = intermediateImageBBox[1] = 0.0;
    intermediateImageBBox[2] = intermediateImageBBox[3] = 0.0;
    totalPatches = 0;
    intermediateImage = nullptr;
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::~avtOSPRayImageCompositor
//
//  Purpose:
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

avtOSPRayImageCompositor::~avtOSPRayImageCompositor()
{
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::BlendFrontToBack
//
//  Purpose:
//      Blends two patches in a front to back manner
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

void avtOSPRayImageCompositor::BlendFrontToBack(const float *srcImage,
                                                const int srcExtents[4],
                                                const int blendExtents[4],
                                                      float *&dstImage,
                                                const int dstExtents[4])
{
#ifdef COMMENT_OUT_FOR_NOW
    if (UseThreadedBlend_Communicator)
    {
        // ARS This function I believe was in the OSPRay module_visit.
        BlendFrontToBack(blendExtents,
                         srcExtents, srcImage,
                         dstExtents, dstImage);
    }
    else
#endif
    {
        // Image sizes
        const int srcX = srcExtents[1] - srcExtents[0];
        const int srcY = srcExtents[3] - srcExtents[2];
        const int dstX = dstExtents[1] - dstExtents[0];
        const int dstY = dstExtents[3] - dstExtents[2];
        // Determine the region to blend
        const int startX =
            std::max(std::max(blendExtents[0], srcExtents[0]), dstExtents[0]);
        const int startY =
            std::max(std::max(blendExtents[2], srcExtents[2]), dstExtents[2]);
        const int endX =
            std::min(std::min(blendExtents[1], srcExtents[1]), dstExtents[1]);
        const int endY =
            std::min(std::min(blendExtents[3], srcExtents[3]), dstExtents[3]);

        for (int y = startY; y < endY; ++y)
        {
            for (int x = startX; x < endX; ++x)
            {
                // Get indices
                int srcIndex = (srcX*(y-srcExtents[2]) + x-srcExtents[0])*4;
                int dstIndex = (dstX*(y-dstExtents[2]) + x-dstExtents[0])*4;

                // Front to back compositing
                if (dstImage[dstIndex + 3] < 1.0f) {
                    float trans = 1.0f - dstImage[dstIndex + 3];
                    dstImage[dstIndex+0] =
                        CLAMP(srcImage[srcIndex+0] * trans +
                              dstImage[dstIndex+0],
                              0.0f, 1.0f);
                    dstImage[dstIndex+1] =
                        CLAMP(srcImage[srcIndex+1] * trans +
                              dstImage[dstIndex+1],
                              0.0f, 1.0f);
                    dstImage[dstIndex+2] =
                        CLAMP(srcImage[srcIndex+2] * trans +
                              dstImage[dstIndex+2],
                              0.0f, 1.0f);
                    dstImage[dstIndex+3] =
                        CLAMP(srcImage[srcIndex+3] * trans +
                              dstImage[dstIndex+3],
                              0.0f, 1.0f);
                }
            }
        }
    }
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::BlendBackToFront
//
//  Purpose:
//      Blends two patches in a back to front manner
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

void avtOSPRayImageCompositor::BlendBackToFront(const float *srcImage,
                                                const int srcExtents[4],
                                                const int blendExtents[4],
                                                      float *&dstImage,
                                                const int dstExtents[4])
{
#ifdef COMMENT_OUT_FOR_NOW
    if (UseThreadedBlend_Communicator)
    {
        // ARS This function I believe was in the OSPRay module_visit.
        BlendBackToFront(blendExtents,
                         srcExtents, srcImage,
                         dstExtents, dstImage);
    }
    else
#endif
    {
        // image sizes
        const int srcX = srcExtents[1] - srcExtents[0];
        const int srcY = srcExtents[3] - srcExtents[2];
        const int dstX = dstExtents[1] - dstExtents[0];
        const int dstY = dstExtents[3] - dstExtents[2];
        // Determine the region to blend
        const int startX =
            std::max(std::max(blendExtents[0], srcExtents[0]), dstExtents[0]);
        const int startY =
            std::max(std::max(blendExtents[2], srcExtents[2]), dstExtents[2]);
        const int endX =
            std::min(std::min(blendExtents[1], srcExtents[1]), dstExtents[1]);
        const int endY =
            std::min(std::min(blendExtents[3], srcExtents[3]), dstExtents[3]);

        for (int y = startY; y < endY; ++y)
        {
            for (int x = startX; x < endX; ++x)
            {
                // Get indices
                int srcIndex = (srcX*(y-srcExtents[2]) + x-srcExtents[0])*4;
                int dstIndex = (dstX*(y-dstExtents[2]) + x-dstExtents[0])*4;

                // Back to front compositing
                float trans = 1.0f - srcImage[srcIndex + 3];
                dstImage[dstIndex+0] =
                    CLAMP(dstImage[dstIndex+0] * trans + srcImage[srcIndex+0],
                          0.0f, 1.0f);
                dstImage[dstIndex+1] =
                    CLAMP(dstImage[dstIndex+1] * trans + srcImage[srcIndex+1],
                          0.0f, 1.0f);
                dstImage[dstIndex+2] =
                    CLAMP(dstImage[dstIndex+2] * trans + srcImage[srcIndex+2],
                          0.0f, 1.0f);
                dstImage[dstIndex+3] =
                    CLAMP(dstImage[dstIndex+3] * trans + srcImage[srcIndex+3],
                          0.0f, 1.0f);
            }
        }
    }
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::BlendFrontToBack
//
//  Purpose:
//      Blends tow patches in a front to back manner
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

void avtOSPRayImageCompositor::BlendFrontToBack(const float * srcImage,
                                                const int srcExtents[4],
                                                float *& dstImage,
                                                const int dstExtents[4])
{
    BlendFrontToBack(srcImage, srcExtents, srcExtents, dstImage, dstExtents);
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::BlendBackToFront
//
//  Purpose:
//      Blends tow patches in a back to front manner
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

void avtOSPRayImageCompositor::BlendBackToFront(const float * srcImage,
                                                const int srcExtents[4],
                                                float *& dstImage,
                                                const int dstExtents[4])
{
    BlendBackToFront(srcImage, srcExtents, srcExtents, dstImage, dstExtents);
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::Barrier
//
//  Purpose:
//    Barrier, useful for debugging
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

void avtOSPRayImageCompositor::Barrier()
{
#ifdef PARALLEL
    MPI_Barrier(MPI_COMM_WORLD);
#endif
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::ColorImage
//
//  Purpose:
//       Fills a 4 channel image with a specific color
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

void avtOSPRayImageCompositor::ColorImage(float *&srcImage,
                                         const int widthSrc,
                                         const int heightSrc,
                                         const float color[4])
{
    for (int i = 0; i < heightSrc * widthSrc; ++i)
    {
        const int srcIndex = 4 * i;
        srcImage[srcIndex+0] = color[0];
        srcImage[srcIndex+1] = color[1];
        srcImage[srcIndex+2] = color[2];
        srcImage[srcIndex+3] = color[3];
    }
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::PlaceImage
//
//  Purpose:
//      Puts srcImage into dstImage
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

void avtOSPRayImageCompositor::PlaceImage(const float *srcImage,
                                         const int srcExtents[4],
                                         float *&dstImage,
                                         const int dstExtents[4])
{
    const int srcX = srcExtents[1] - srcExtents[0];
    const int srcY = srcExtents[3] - srcExtents[2];
    const int dstX = dstExtents[1] - dstExtents[0];
    const int dstY = dstExtents[3] - dstExtents[2];
    const int startingX = std::max(srcExtents[0], dstExtents[0]);
    const int startingY = std::max(srcExtents[2], dstExtents[2]);
    const int endingX = std::min(srcExtents[1], dstExtents[1]);
    const int endingY = std::min(srcExtents[3], dstExtents[3]);

    for (int y = startingY; y < endingY; ++y)
    {
        for (int x = startingX; x < endingX; ++x)
        {
            // index in the sub-image
            const int srcIndex =
                (srcX * (y-srcExtents[2]) + x-srcExtents[0]) * 4;
            // index in the larger buffer
            const int dstIndex =
                (dstX * (y-dstExtents[2]) + x-dstExtents[0]) * 4;
            dstImage[dstIndex+0] = srcImage[srcIndex+0];
            dstImage[dstIndex+1] = srcImage[srcIndex+1];
            dstImage[dstIndex+2] = srcImage[srcIndex+2];
            dstImage[dstIndex+3] = srcImage[srcIndex+3];
        }
    }
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::BlendWithBackground
//
//  Purpose:
//      Blends _image with the backgroundColor
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

void avtOSPRayImageCompositor::BlendWithBackground(float *&image,
                                                   const int extents[4],
                                                   const float bgColor[4])
{
    const int pixelSize = (extents[3]-extents[2]) * (extents[1]-extents[0]);
    // estimated potential speedup: 2.240
    for (int i = 0; i < pixelSize; ++i)
    {
        const int   idx = i * 4;
        const float alpha = (1.0 - image[idx+3]);
        image[idx+0] = bgColor[0] * alpha + image[idx+0];
        image[idx+1] = bgColor[1] * alpha + image[idx+1];
        image[idx+2] = bgColor[2] * alpha + image[idx+2];
        image[idx+3] = bgColor[3] * alpha + image[idx+3];
    }
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::IceTValid
//
//  Purpose:
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

bool avtOSPRayImageCompositor::IceTValid()
{
    return avtOSPRayIC_IceT::Valid();
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::IceTInit
//
//  Purpose:
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

void avtOSPRayImageCompositor::IceTInit(int W, int H)
{
    if (!avtOSPRayIC_IceT::Valid())
    {
        const std::string msg =
	    "The IceT compositor is not valid. "
	    "VisIt was probably compiled with IceT";
	EXCEPTION1(ImproperUseException, msg);
    }

    if (compositor)
      delete compositor;

    compositor = new avtOSPRayIC_IceT(mpiSize, mpiRank);
    compositor->Init(W, H);
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::IceTSetTile
//
//  Purpose:
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

void avtOSPRayImageCompositor::IceTSetTile(const float* d,
                                           const int*   e,
                                           const float& z)
{
#ifdef COMMENT_OUT_FOR_NOW
    ospray::timestamp timingDetail;

    ospray::CheckSectionStart("avtOSPRayImageCompositor",
                             "IceTSetTile", timingDetail,
                             "IceT Setup Image Tile");
#endif

    compositor->SetTile(d, e, z);

#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStop("avtOSPRayImageCompositor",
                            "IceTSetTile", timingDetail,
                            "IceT Setup Image Tile");
#endif
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::IceTComposite
//
//  Purpose:
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

void avtOSPRayImageCompositor::IceTComposite(float*& output)
{
#ifdef COMMENT_OUT_FOR_NOW
    ospray::timestamp timingDetail;

    ospray::CheckSectionStart("avtOSPRayImageCompositor",
                             "IceTComposite", timingDetail,
                             "IceT Image Composition");
#endif
    compositor->Composite(output);

    if (compositor != nullptr)
    {
        delete compositor;
        compositor = nullptr;
    }

#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStop("avtOSPRayImageCompositor",
                            "IceTComposite", timingDetail,
                            "IceT Image Composition");
#endif
}

// ***************************************************************************
//  Method: avtOSPRayImageCompositor::OneNodeValid
//
//  Purpose:
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

bool avtOSPRayImageCompositor::OneNodeValid()
{
    return avtOSPRayIC_OneNode::Valid();
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::OneNodeValid
//
//  Purpose:
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

void avtOSPRayImageCompositor::OneNodeInit(int W, int H)
{
    if (compositor) delete compositor;
    compositor = new avtOSPRayIC_OneNode(mpiSize, mpiRank);
    compositor->Init(W, H);
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::OneNodeValid
//
//  Purpose:
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

void avtOSPRayImageCompositor::OneNodeSetTile(const float* d,
                                             const int*   e,
                                             const float& z)
{
    compositor->SetTile(d, e, z);
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::OneNodeValid
//
//  Purpose:
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

void avtOSPRayImageCompositor::OneNodeComposite(float*& output)
{
    compositor->Composite(output);
    if (compositor != nullptr) { delete compositor; }
    compositor = nullptr;
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::UpdateBoundingBox
//
//  Purpose:
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

void
avtOSPRayImageCompositor::UpdateBoundingBox(int currentBoundingBox[4],
                                            const int imageExtents[4])
{
    if ((currentBoundingBox[0] == 0 && currentBoundingBox[1] == 0) &&
        (currentBoundingBox[2] == 0 && currentBoundingBox[3] == 0)) {
        currentBoundingBox[0]=imageExtents[0];
        currentBoundingBox[1]=imageExtents[1];
        currentBoundingBox[2]=imageExtents[2];
        currentBoundingBox[3]=imageExtents[3];
        return;
    }

    if (imageExtents[0] < currentBoundingBox[0])
        { currentBoundingBox[0] = imageExtents[0]; }

    if (imageExtents[2] < currentBoundingBox[2])
        { currentBoundingBox[2] = imageExtents[2]; }

    if (imageExtents[1] > currentBoundingBox[1])
        { currentBoundingBox[1] = imageExtents[1]; }

    if (imageExtents[3] > currentBoundingBox[3])
        { currentBoundingBox[3] = imageExtents[3]; }
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::GatherDepthAtRoot
//
//  Purpose:
//      Used by Serial Direct Send
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

void
avtOSPRayImageCompositor::GatherDepthAtRoot(const int numlocalPatches,
                                            const float *localPatchesDepth,
                                            int &totalPatches,
                                            int *&patchCountPerRank,
                                            float *&allPatchesDepth)
{
#ifdef PARALLEL
    // Get how many patches are coming from each MPI rank
    totalPatches = 0;
    int *patchesOffset = nullptr;

    if (mpiRank == 0) // root!
        { patchCountPerRank = new int[mpiSize](); }

    // reference
    // https://www.mpich.org/static/docs/v3.1/www3/MPI_Gather.html
    MPI_Gather(&numlocalPatches, /* send buffer */
               1, /* send count */
               MPI_INT,
               patchCountPerRank, /* address of receive buffer (root) */
               1, /* number of elements for any single receive (root) */
               MPI_INT,
               0, /* rank of receiving process (integer) */
               MPI_COMM_WORLD); /* communicator (handle) */

    // gather number of patch group
    if (mpiRank == 0)
    {
        patchesOffset = new int[mpiSize]();
        patchesOffset[0] = 0; // a bit redundant

        for (int i=0; i<mpiSize; ++i)
        {
            totalPatches += patchCountPerRank[i];
            if (i == 0)
            {
                patchesOffset[i] = 0;
            }
            else
            {
              patchesOffset[i] = patchesOffset[i-1] + patchCountPerRank[i-1];
            }
        }

        // allocate only at root
        allPatchesDepth = new float[totalPatches];
    }

    // Gathers into specified locations from all processes in a group
    MPI_Gatherv(localPatchesDepth, numlocalPatches, MPI_FLOAT,
                allPatchesDepth, /* receive all depth */
                patchCountPerRank, patchesOffset, MPI_FLOAT,
                0, MPI_COMM_WORLD);

    // Cleanup
    if (mpiRank == 0)
    {
        delete [] patchesOffset;
        patchesOffset = nullptr;
    }
#endif
}

// ***************************************************************************
//  Method: avtOSPRayImageCompositor::SerialDirectSend
//
//  Purpose:
//      A very simple compositing that we can fall back to if parallel direct
//      send is buggy.
//      Works with convex patches though
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

void
avtOSPRayImageCompositor::SerialDirectSend(int localNumPatches,
                                           float *localPatchesDepth,
                                           int *extents,
                                           float *imgData,
                                           float bgColor[4],
                                           int width, int height)
{
#ifdef PARALLEL
    //debug5 << "-- Serial Direct Send --" << std::endl;

    float *recvImage = nullptr;
    int tags[2] = {5781, 5782};

    // Retrieve depth info through MPI
    int    totalPatches; // total number of patches
    int   *totalPatchCountsPerRank = nullptr;
    float *totalPatchDepths = nullptr;
    GatherDepthAtRoot(localNumPatches,
                      localPatchesDepth,
                      totalPatches,
                      totalPatchCountsPerRank,
                      totalPatchDepths);
    //
    if (mpiRank == 0)
    {
        // Root
        int srcSize[2] = {width, height};
        int srcPos[2]  = {0, 0};
        int dstSize[2], dstPos[2];

        // Sort patches we will receive
        std::multimap<float,int> sortedPatches;

        int patchId = 0;
        for (int i=0; i<mpiSize; ++i)
        {
            for (int j=0; j<totalPatchCountsPerRank[i]; ++j)
            {
                sortedPatches.insert
                  (std::make_pair(totalPatchDepths[patchId++],i));
            }
        }

        // Create space for buffers
        int recvParams[4]; // minX, maxX, minY, maxY
        int imgExtents[4] = {0,width,0,height};

        recvImage = new float[width*height*4]();
        finalImage = new float[width*height*4]();

        int localIndex = 0;

        // Compositing
        for (std::multimap<float,int>::iterator it = sortedPatches.begin();
             it != sortedPatches.end(); ++it)
        {
            int rank = (*it).second;
            if (rank != mpiRank)
            {
                // recv image info
                MPI_Recv(recvParams, 4, MPI_INT, rank,
                         tags[0], MPI_COMM_WORLD, MPI_STATUS_IGNORE);
                // recv image
                MPI_Recv(recvImage, width*height*4, MPI_FLOAT, rank,
                         tags[1],  MPI_COMM_WORLD, MPI_STATUS_IGNORE);
                dstPos[0]  = dstPos[0];
                dstPos[1]  = dstPos[1];
                dstSize[0] = recvParams[2]-recvParams[0];
                dstSize[1] = recvParams[3]-recvParams[1];
            }
            else
            {
                // It's local
                recvParams[0] = extents[ localIndex*4 + 0];
                recvParams[1] = extents[ localIndex*4 + 1];
                recvParams[2] = extents[ localIndex*4 + 2];
                recvParams[3] = extents[ localIndex*4 + 3];
                recvImage = &imgData[ localIndex*(width*height*4) ];
                ++localIndex;
            }

            BlendFrontToBack(recvImage, recvParams, finalImage, imgExtents);
        }

        BlendWithBackground(finalImage, imgExtents, bgColor);
    }
    else
    {
        // Sender
        for (int i=0; i<localNumPatches; ++i)
        {
            int imgSize = ((extents[i*4 + 1] - extents[i*4 + 0]) *
                           (extents[i*4 + 3] - extents[i*4 + 2]) * 4);
            if (imgSize > 0)
            {
                MPI_Send(&extents[i*4],
                         4, MPI_INT, 0, tags[0], MPI_COMM_WORLD);
                MPI_Send(&imgData[i*(width*height*4)],
                         imgSize, MPI_FLOAT, 0, tags[1], MPI_COMM_WORLD);
            }
        }
    }

    // Cleanup
    if (totalPatchDepths != nullptr)
    {
        delete [] totalPatchDepths;
        totalPatchDepths = nullptr;
    }

    if (totalPatchCountsPerRank != nullptr)
    {
        delete [] totalPatchCountsPerRank;
        totalPatchCountsPerRank = nullptr;
    }

    if (recvImage != nullptr)
    {
        delete [] recvImage;
        recvImage = nullptr;
    }
#endif
}

// ***************************************************************************
//  Method: avtOSPRayImageCompositor::regionAllocation
//
//  Purpose:
//      Arbitrarily allocates regions to MPI ranks
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

void
avtOSPRayImageCompositor::RegionAllocation(int *& regions)
{
    regions = new int[mpiSize];

    // Initial allocation: partition for section rank
    for (int i=0; i<mpiSize; ++i)
    {
        regions[i] = i;
    }
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::parallelDirectSend
//
//  Purpose:
//      Parallel Direct Send rendering that can blend convex patches from each
//      MPI rank. However, since we are not guaranteed to have convex patches.
//      It's not used.
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************
void
avtOSPRayImageCompositor::parallelDirectSend(float *imgData,
                                            int imgExtents[4],
                                            int region[],
                                            int numRegions,
                                            int tags[2],
                                            int fullImageExtents[4])
{
#ifdef PARALLEL
    // Determine position in region (myPositionInRegion)
    int width  = fullImageExtents[1]-fullImageExtents[0];
    int height = fullImageExtents[3]-fullImageExtents[2];

    //debug5 << "fullImageExtents: " << fullImageExtents[0] << ", " << fullImageExtents[1] << "   " << fullImageExtents[2] << ", " << fullImageExtents[3] << endl;

    compositingDone = false;
    int myPositionInRegion = -1;
    bool inRegion = true;
    std::vector<int> regionVector(region, region+numRegions);
    std::vector<int>::iterator it = std::find(regionVector.begin(),
                                              regionVector.end(),
                                              mpiRank);

    if (it == regionVector.end())
    {
        inRegion = false;
        //debug5 << mpiRank << " ~ SHOULD NOT HAPPEN: Not found " << mpiRank <<  " !!!" << std::endl;
    }
    else
    {
        myPositionInRegion = it - regionVector.begin();
    }

    // Region boundaries
    int regionHeight = height/numRegions;
    int lastRegionHeight = height - regionHeight*(numRegions-1);

    // Extents of the region
    int myStartingHeight =
        fullImageExtents[2] +
        myPositionInRegion *
        regionHeight;
    int myEndingHeight = myStartingHeight + regionHeight;

    if (myPositionInRegion == numRegions-1)
        myEndingHeight = fullImageExtents[3];

    int myRegionHeight = myEndingHeight-myStartingHeight;

    // Size of one buffer
    int sizeOneBuffer = std::max(regionHeight,lastRegionHeight) * width * 4;

    //debug5 << "myPositionInRegion: " << myPositionInRegion << std::endl;
    //debug5 << "My extents: " << imgExtents[0] << ", " << imgExtents[1] << ", " << imgExtents[2] << ", " << imgExtents[3] << std::endl;
    //debug5 << "myRegionHeight: " << myRegionHeight << "  lastRegionHeight: " << lastRegionHeight << " regionHeight: " << regionHeight << "  myStartingHeight: " << myStartingHeight << "  myEndingHeight: " << myEndingHeight << std::endl;


    // MPI Async

    // Recv
    MPI_Request *recvMetaRq  = new MPI_Request[ numRegions-1 ];
    MPI_Request *recvImageRq = new MPI_Request[ numRegions-1 ];

    MPI_Status *recvMetaSt  = new MPI_Status[ numRegions-1 ];
    MPI_Status *recvImageSt = new MPI_Status[ numRegions-1 ];

    // Send
    MPI_Request *sendMetaRq  = new MPI_Request[ numRegions-1 ];
    MPI_Request *sendImageRq = new MPI_Request[ numRegions-1 ];

    MPI_Status *sendMetaSt  = new MPI_Status[ numRegions-1 ];
    MPI_Status *sendImageSt = new MPI_Status[ numRegions-1 ];


    // Create Buffers

    // Create buffer for receiving images
    float * recvDataBuffer = new float[sizeOneBuffer * numRegions];

    // Create buffer for receiving messages
    std::vector<int> msgBuffer;
    msgBuffer.clear();
    msgBuffer.resize(5 * numRegions);

    // Create buffer for sending messages
    int *sendExtents = new int[numRegions*5];

    //
    // Async Recv
    if (inRegion)
    {
        int recvCount=0;
        for (int i=0; i<numRegions; ++i)
        {
          if ( regionVector[i] == mpiRank )
              continue;

          int src = regionVector[i];

          MPI_Irecv(&msgBuffer[i*5],                              5, MPI_INT,   src, tags[0], MPI_COMM_WORLD,  &recvMetaRq[recvCount] );
          MPI_Irecv(&recvDataBuffer[i*sizeOneBuffer], sizeOneBuffer, MPI_FLOAT, src, tags[1], MPI_COMM_WORLD,  &recvImageRq[recvCount] );

          ++recvCount;
        }
    }

    //debug5 << "Async Recv setup done " << std::endl;

    // Async Send
    int sendCount = 0;
    int sendingOffset;

    for (int i=0; i<numRegions; ++i)
    {
        int regionStart, regionEnd, imgSize, dest;
        dest = regionVector[i];

        if ( dest == mpiRank )
          continue;

        regionStart = i*regionHeight;
        regionEnd = regionStart + regionHeight;
        if (i == numRegions-1) // the last one in region
          regionEnd = height;

        int startingYExtents = fullImageExtents[2] + regionStart;
        int endingYExtents = fullImageExtents[2] + regionEnd;

        //debug5 << "startingYExtents: " << startingYExtents <<"   endingYExtents: " << endingYExtents <<  std::endl;

        if (startingYExtents < imgExtents[2])
          startingYExtents = imgExtents[2];

        if (endingYExtents > imgExtents[3])
          endingYExtents = imgExtents[3];

        bool hasData = true;
        if (endingYExtents - startingYExtents <= 0 || imgExtents[1]-imgExtents[0] <= 0)
        {
            hasData = false;

            sendingOffset = 0;
            imgSize = sendExtents[i*5 + 0] = sendExtents[i*5 + 1] = sendExtents[i*5 + 2] = sendExtents[i*5 + 3] =  sendExtents[i*5 + 4] = 0;
        }
        else
        {
          imgSize = (endingYExtents-startingYExtents) * (imgExtents[1]-imgExtents[0]) * 4;
          sendingOffset = (startingYExtents-imgExtents[2]) * (imgExtents[1]-imgExtents[0]) * 4;

          sendExtents[i*5 + 0] = imgExtents[0];
          sendExtents[i*5 + 1] = imgExtents[1];
          sendExtents[i*5 + 2] = startingYExtents;
          sendExtents[i*5 + 3] = endingYExtents;
          sendExtents[i*5 + 4] = 0;
        }

        //std::cout << mpiRank << " ~ i: " << i << "   regionVector[index]: " << regionVector[index] << "  extents: " <<  sendExtents[index*5 + 0] << ", " << sendExtents[index*5 + 1]  << ", " << sendExtents[index*5 + 2] << ", " << sendExtents[index*5 + 3] << "  sending ... " << std::endl;

        MPI_Isend(&sendExtents[i*5],             5,   MPI_INT, dest, tags[0], MPI_COMM_WORLD, &sendMetaRq[sendCount]);
        MPI_Isend(&imgData[sendingOffset], imgSize, MPI_FLOAT, dest, tags[1], MPI_COMM_WORLD, &sendImageRq[sendCount]);

        //debug5 << "dest: " << dest <<"   sendExtents: " << sendExtents[i*5 +0] << ", " << sendExtents[i*5 +1] << "    " << sendExtents[i*5 +2] << ", " << sendExtents[i*5 +3] << std::endl << std::endl;

        ++sendCount;
    }

    //debug5 << "Async Recv" << std::endl;

    //
    // Create buffer for region
    intermediateImageExtents[0] = fullImageExtents[0];
    intermediateImageExtents[1] = fullImageExtents[1];
    intermediateImageExtents[2] = myStartingHeight;
    intermediateImageExtents[3] = myEndingHeight;

    intermediateImage = new float[width * (myEndingHeight-myStartingHeight) * 4]();

    int recvImageExtents[4];
    float *recvImageData;

    // Blend
    int numBlends = 0;
    int countBlend = 0;

    intermediateImageBBox[0] = intermediateImageBBox[2] = 0;
    intermediateImageBBox[1] = intermediateImageBBox[3] = 0;

    if (inRegion)
    {
        for (int i=0; i<numRegions; ++i)
        {
            int index = i;

            //debug5 << "regionVector[" << i << "] " << regionVector[index] << std::endl;

            if (regionVector[index] == mpiRank)
            {
                int startingYExtents = myStartingHeight;
                int endingYExtents = myEndingHeight;

                if (startingYExtents < imgExtents[2])
                  startingYExtents = imgExtents[2];

                if (endingYExtents > imgExtents[3])
                  endingYExtents = imgExtents[3];


                bool hasData = true;
                if (endingYExtents - startingYExtents <= 0)
                {
                    hasData = false;
                    endingYExtents = startingYExtents = 0;
                }

                if (hasData == true)
                {
                    int extentsSectionRecv[4];
                    extentsSectionRecv[0] = imgExtents[0];
                    extentsSectionRecv[1] = imgExtents[1];
                    extentsSectionRecv[2] = startingYExtents;
                    extentsSectionRecv[3] = endingYExtents;

                    BlendFrontToBack(imgData, imgExtents, extentsSectionRecv, intermediateImage, intermediateImageExtents);

                    UpdateBoundingBox(intermediateImageBBox, extentsSectionRecv);
                    ++numBlends;
                }
            }
            else
            {
              MPI_Wait(&recvMetaRq[countBlend], &recvMetaSt[countBlend]);

              for (int j=0; j<4; ++j)
                recvImageExtents[j] = msgBuffer[index*5 + j];

              bool hasData =  false;
              if (recvImageExtents[1]-recvImageExtents[0] > 0 && recvImageExtents[3]-recvImageExtents[2] > 0)
              {
                   hasData = true;
                   MPI_Wait(&recvImageRq[countBlend], &recvImageSt[countBlend]);
                   recvImageData = &recvDataBuffer[index*sizeOneBuffer];
              }

              if (hasData)
              {
                  BlendFrontToBack(recvImageData, recvImageExtents, intermediateImage, intermediateImageExtents);

                  UpdateBoundingBox(intermediateImageBBox, recvImageExtents);
                  ++numBlends;
              }

              ++countBlend;
            }
        }
    }
    else
    {
        compositingDone = true;
    }

    //debug5 << "PDS blending done" << std::endl;

    msgBuffer.clear();

    delete [] recvDataBuffer;
    recvDataBuffer = nullptr;

    if (numBlends == 0)
        intermediateImageBBox[0] = intermediateImageBBox[1] =
        intermediateImageBBox[2] = intermediateImageBBox[3] = 0;

    delete [] recvMetaRq;
    delete [] recvImageRq;
    delete [] recvMetaSt;
    delete [] recvImageSt;

    delete [] sendMetaRq;
    delete [] sendImageRq;
    delete [] sendMetaSt;
    delete [] sendImageSt;

    recvMetaRq  = nullptr;
    recvImageRq = nullptr;
    recvMetaSt  = nullptr;
    recvImageSt = nullptr;

    sendMetaRq  = nullptr;
    sendImageRq = nullptr;
    sendMetaSt  = nullptr;
    sendImageSt = nullptr;
#endif
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::findRegionsForPatch
//
//  Purpose:
//      Needed by Parallel Direct Send to determine the regions a
//      patch will overlap
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************
int
avtOSPRayImageCompositor::findRegionsForPatch(int patchExtents[4], int screenProjectedExtents[4], int numRegions, int &from, int &to)
{
    from = to = 0;
    if (patchExtents[1]-patchExtents[0] <=0 || patchExtents[3]-patchExtents[2] <=0)
        return 0;

    if ( patchExtents[0] > screenProjectedExtents[1])
        return 0;

    if ( patchExtents[1] < screenProjectedExtents[0])
        return 0;

    if ( patchExtents[2] > screenProjectedExtents[3])
        return 0;

    if ( patchExtents[3] < screenProjectedExtents[2])
        return 0;


    // find from
    for (int i=numRegions-1; i>=0; i--)
    {
        if ( patchExtents[2] >= getScreenRegionStart(i, screenProjectedExtents[2], screenProjectedExtents[3]) )
        {
            from = i;
            break;
        }
    }

    // find to
    for (int i=numRegions-1; i>=0; i--)
    {
        if ( patchExtents[3] > getScreenRegionStart(i, screenProjectedExtents[2], screenProjectedExtents[3]) )
        {
            to = i;
            break;
        }
    }

    return std::max( (to - from) + 1, 0);
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::computeRegionExtents
//
//  Purpose:
//      Compute extents for each region
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************
void
avtOSPRayImageCompositor::computeRegionExtents(int numRanks, int height)
{
    //debug5 << "computeRegionExtents height " << height << std::endl;

    int regionHeight = round((float)height/numRanks);

    regularRegionSize = regionHeight;
    maxRegionHeight = 0;

    regionRankExtents.resize(numRanks*3);

    for (int i=0; i<numRanks; ++i)
    {
        int startRegionExtents, endRegionExtents, _currentRegionHeight;

        startRegionExtents = CLAMP(regionHeight * i, 0, height);
        endRegionExtents = CLAMP(regionHeight * i + regionHeight, 0, height);

        if ( i == numRanks-1 && endRegionExtents < height )
            endRegionExtents = height;

        _currentRegionHeight = CLAMP(endRegionExtents-startRegionExtents, 0, height);
        maxRegionHeight = std::max(maxRegionHeight, _currentRegionHeight);

        regionRankExtents[i*3+0] = startRegionExtents;
        regionRankExtents[i*3+1] = endRegionExtents;
        regionRankExtents[i*3+2] = _currentRegionHeight;

        //debug5 << i << " : (start, end, region): " << startRegionExtents << ", " << endRegionExtents << ", " << _currentRegionHeight << std::endl;
    }
}


// ***************************************************************************
//  Method: avtOSPRayImageCompositor::ParallelDirectSendManyPatches
//
//  Purpose:
//      Parallel Direct Send rendering that can blend individual patches
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************

int
avtOSPRayImageCompositor::ParallelDirectSendManyPatches
    (const std::multimap<int, ImgData> &imgDataHashMap,
     const std::vector<ImgMetaData>    &imageMetaPatchVector,
     int numPatches,
     int *region,
     int numRegions,
     int tags[2],
     int fullImageExtents[4])
{
    int myRegionHeight = 0;
#ifdef PARALLEL

    //debug5 << "Parallel Direct Send" << endl;

    // Some Initializations
    for (int i=0; i<4; ++i)
    {
        intermediateImageExtents[i] = 0;
        intermediateImageBBox[i] = 0;
    }

    // **********************************************************
    // Find My Position in Regions
#ifdef COMMENT_OUT_FOR_NOW
    ospray::timestamp timingDetail;
    ospray::CheckSectionStart("avtOSPRayImageCompositor",
                              "ParallelDirectSendManyPatches", timingDetail,
                              "Find my position in regions");
#endif

    compositingDone = false;
    int myPositionInRegion = -1;
    bool inRegion = true;
    std::vector<int> regionVector(region, region+numRegions);
    const std::vector<int>::const_iterator it =
        std::find(regionVector.begin(), regionVector.end(), mpiRank);

    if (it == regionVector.end())
    {
        inRegion = false;
        //debug5 << mpiRank << " ~ SHOULD NOT HAPPEN!!!!: Not found "
        //       << mpiRank <<  " !!!" << std::endl;
    }
    else
    {
        myPositionInRegion = it - regionVector.begin();
    }

    int width =  fullImageExtents[1]-fullImageExtents[0];
    int height = fullImageExtents[3]-fullImageExtents[2];

#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStop("avtOSPRayImageCompositor",
                             "ParallelDirectSendManyPatches", timingDetail,
                             "Find my position in regions");
#endif
    //debug5 << mpiRank << " ~ myPositionInRegion: "
    //       << myPositionInRegion << ", numRanks: " << mpiSize << std::endl;
    //debug5 << "width: " << width << ", height : " << height
    //       << " | fullImageExtents: "
    //       << fullImageExtents[0] << ", "
    //       << fullImageExtents[1] << ", "
    //       << fullImageExtents[2] << ", "
    //       << fullImageExtents[3] << std::endl;

    // **********************************************************
    // Compute Region Boundaries
#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStart("avtOSPRayImageCompositor",
                              "ParallelDirectSendManyPatches", timingDetail,
                              "Compute region boundaries");
#endif
    computeRegionExtents(mpiSize, height); // ?
    int myStartingHeight = getScreenRegionStart
        (myPositionInRegion, fullImageExtents[2], fullImageExtents[3]);
    int myEndingHeight   = getScreenRegionEnd
        (myPositionInRegion, fullImageExtents[2], fullImageExtents[3]);
    myRegionHeight = CLAMP((myEndingHeight-myStartingHeight), 0, height);

    // debug5 << "myStartingHeight: " << myStartingHeight << ", "
    //        << "myEndingHeight: "   << myEndingHeight   << ", "
    //        << "myRegionHeight: "   << myRegionHeight   << std::endl;

#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStop("avtOSPRayImageCompositor",
                             "ParallelDirectSendManyPatches", timingDetail,
                             "Compute region boundaries");
#endif

    // Size of one buffer
    int sizeOneBuffer = getMaxRegionHeight() * width * 4;

    // **********************************************************
    // Determine how many patches and pixel to send to each region.
#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStart("avtOSPRayImageCompositor",
                              "ParallelDirectSendManyPatches", timingDetail,
                              "Determine how many patches and pixels to send "
                              "to each region");
#endif
    std::vector<int> numPatchesPerRegion;
    std::vector<int> areaPerRegion;
    std::set<int> numOfRegions;
    numPatchesPerRegion.resize(numRegions);
    areaPerRegion.resize(numRegions);

    // 2D array: extents for each partition
    std::vector < std::vector<float> > extentsPerPartiton;
    for (int i=0; i<numRegions; ++i) {
        extentsPerPartiton.push_back(std::vector<float>());
    }

    // debug5 << "Parallel Direct Send ~ numPatches " << numPatches << endl;
    int totalSendBufferSize = 0;
    for (int i=0; i<numPatches; ++i)
    {
        int _patchExtents[4];
#ifdef COMMENT_OUT_FOR_NOW
        ospray::ImgMetaData temp;
        temp = imageMetaPatchVector.at(i);
        _patchExtents[0] = temp.screen_ll[0];   // minX
        _patchExtents[1] = temp.screen_ur[0];   // maxX
        _patchExtents[2] = temp.screen_ll[1];   // minY
        _patchExtents[3] = temp.screen_ur[1];   // maxY
#endif
        const std::multimap<int, ImgData>::const_iterator it =
          imgDataHashMap.find( i );

        int from, to;
        int numRegionIntescection = findRegionsForPatch(_patchExtents,
                                                        fullImageExtents,
                                                        numRegions,
                                                        from, to);
        if (numRegionIntescection <= 0) continue;
        // debug5 << "\nParallel Direct Send ~ patch " << i
        //        << "  from:" << from << "  to:" << to
        //        << "  numPatches: " << numPatches
        //        << "   _patchExtents: "
        //        << _patchExtents[0] << ", "
        //        << _patchExtents[1] << ", "
        //        << _patchExtents[2] << ", "
        //        << _patchExtents[3]
        //        << ", fullImageExtents[2]: " << fullImageExtents[2]
        //        << ", numRegions: " <<  numRegions
        //        << ", totalSendBufferSize: " << totalSendBufferSize << endl;
        for (int j=from; j<=to; ++j)
          ++numPatchesPerRegion[j];

        for (int partition=from; partition<=to; partition++)
        {
            int _extentsYStart = std::max( _patchExtents[2], getScreenRegionStart(partition, fullImageExtents[2], fullImageExtents[3]) );
            int _extentsYEnd   = std::min( _patchExtents[3], getScreenRegionEnd(  partition, fullImageExtents[2], fullImageExtents[3]) );
            int _area = (_extentsYEnd-_extentsYStart)*(_patchExtents[1]-_patchExtents[0]);
            areaPerRegion[partition] += _area;
            totalSendBufferSize += _area;
            //debug5 << "_patchExtents[2]: " << _patchExtents[2] << ", region start: " << getScreenRegionStart(partition, fullImageExtents[2], fullImageExtents[3]) <<  ", _extentsYStart: " << _extentsYStart<< endl;
            //debug5 << "_patchExtents[3]: " << _patchExtents[3] << ", region end: " << getScreenRegionEnd(partition, fullImageExtents[2], fullImageExtents[3]) << ", _extentsYEnd: " << _extentsYEnd << endl;
            //debug5 << "_area " << _area << endl;
            extentsPerPartiton[partition].push_back(i);
            extentsPerPartiton[partition].push_back(_patchExtents[0]);
            extentsPerPartiton[partition].push_back(_patchExtents[1]);
            extentsPerPartiton[partition].push_back(_extentsYStart);
            extentsPerPartiton[partition].push_back(_extentsYEnd);
#ifdef COMMENT_OUT_FOR_NOW
            extentsPerPartiton[partition].push_back(temp.eye_z);
#endif
            numOfRegions.insert(partition);
        }
    }

    totalSendBufferSize *= 4;                           // to account for RGBA
    int numRegionsWithData = numOfRegions.size();
    //debug5 << "\nParallel Direct Send ~ creating buffers" << endl;

#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStop("avtOSPRayImageCompositor",
                             "ParallelDirectSendManyPatches", timingDetail,
                             "Determine how many patches and pixels to send "
                             "to each region");
#endif

    // **********************************************************
    // Copy the data for each region for each patch
#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStart("avtOSPRayImageCompositor",
                              "ParallelDirectSendManyPatches", timingDetail,
                              "Copy the data for each region for each patch");
#endif
    // Create buffer
    float *sendDataBuffer        = new float[totalSendBufferSize];     // contains all the data arranged by region
    int   *sendDataBufferSize    = new int[numRegionsWithData]();
    int   *sendDataBufferOffsets = new int[numRegionsWithData]();
    int   *sendBuffer            = new int[numRegions*2]();

    int regionWithDataCount = 0;
    int numRegionsToSend = 0;

    // Populate the buffer with data
    int dataSendBufferOffset = 0;
    for (int i=0; i<numRegions; ++i)
    {
        int _dataSize = 0;
        //debug5 << "Region: " << i << "  size: " << extentsPerPartiton[i].size() << std::endl;
        for (int j=0; j<extentsPerPartiton[i].size(); j+=6)
        {
            int _patchID = extentsPerPartiton[i][j + 0];
            const std::multimap<int, ImgData>::const_iterator it = imgDataHashMap.find( _patchID );
            int _width = (extentsPerPartiton[i][j+2] - extentsPerPartiton[i][j+1]);
            int _bufferSize = _width * (extentsPerPartiton[i][j+4] - extentsPerPartiton[i][j+3]) * 4;
            int _dataOffset = extentsPerPartiton[i][j+3] - imageMetaPatchVector[_patchID].screen_ll[1];

            memcpy(&sendDataBuffer[dataSendBufferOffset],
                   &(((*it).second).imagePatch[_width * _dataOffset * 4]),
                   _bufferSize*sizeof(float) );

            dataSendBufferOffset += _bufferSize;
            _dataSize += _bufferSize;
        }

        if (_dataSize != 0)
        {
            sendDataBufferSize[regionWithDataCount] = _dataSize;

            ++regionWithDataCount;

            if (regionWithDataCount != numRegionsWithData)
              sendDataBufferOffsets[regionWithDataCount] = sendDataBufferOffsets[regionWithDataCount-1] + sendDataBufferSize[regionWithDataCount-1];

            if (regionVector[i] != mpiRank)
              ++numRegionsToSend;
        }

        sendBuffer[i*2+0] = numPatchesPerRegion[i];
        sendBuffer[i*2+1] = areaPerRegion[i];

        //debug5 << "Region: " << i << "  numPatchesPerRegion: " << sendBuffer[i*2+0] << ", sendBuffer[i*2+1]: " << sendBuffer[i*2+1] << std::endl;
    }

#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStop("avtOSPRayImageCompositor",
                            "ParallelDirectSendManyPatches", timingDetail,
                            "Copy the Data for Each Region for Each Patch");
#endif

    // **********************************************************
    // Exchange Information about Size to Recv
#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStart("avtOSPRayImageCompositor",
                             "ParallelDirectSendManyPatches", timingDetail,
                             "Exchange information about size to recv");
#endif
    int *recvInfoATABuffer = new int[numRegions*2]();
    MPI_Alltoall(sendBuffer, 2, MPI_INT,  recvInfoATABuffer, 2, MPI_INT, MPI_COMM_WORLD);

    delete [] sendBuffer;
    sendBuffer = nullptr;

#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStop("avtOSPRayImageCompositor",
                            "ParallelDirectSendManyPatches", timingDetail,
                            "Exchange information about size to recv");
#endif
    //debug5 << "Parallel Direct Send ~ Exchange information about size to recv" << endl;

    // **********************************************************
    // Calculate Buffer Size Needed
#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStart("avtOSPRayImageCompositor",
                             "ParallelDirectSendManyPatches", timingDetail,
                             "Calculate buffer size needed");
#endif
    int infoBufferSize = 0;
    int dataBufferSize = 0;
    int numRegionsToRecvFrom = 0;
    for (int i=0; i<numRegions; ++i)
    {
        infoBufferSize += recvInfoATABuffer[i*2 + 0];   // number of patches per region
        dataBufferSize += recvInfoATABuffer[i*2 + 1];   // area per region
        //debug5 << "From: " << i << ", #patches: " << recvInfoATABuffer[i*2 + 0] << ", " << recvInfoATABuffer[i*2 + 1] << std::endl;
        if (i == mpiRank) continue;
        if (recvInfoATABuffer[i*2 + 0] != 0)
          ++numRegionsToRecvFrom;
    }

#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStop("avtOSPRayImageCompositor",
                            "ParallelDirectSendManyPatches", timingDetail,
                            "Calculate buffer size needed");
#endif

    // **********************************************************
    // Create Structure for MPI Async send/recv

    // Send
    MPI_Request *sendMetaRq  = new MPI_Request[ numRegionsToSend ];
    MPI_Status  *sendMetaSt  = new MPI_Status [ numRegionsToSend ];
    MPI_Request *sendImageRq = new MPI_Request[ numRegionsToSend ];
    MPI_Status  *sendImageSt = new MPI_Status [ numRegionsToSend ];
    // Recv
    MPI_Request *recvMetaRq  = nullptr;
    MPI_Status  *recvMetaSt  = nullptr;
    MPI_Request *recvImageRq = nullptr;
    MPI_Status  *recvImageSt = nullptr;
    // Counters
    int recvInfoCount = 0;
    int offsetMeta = 0;
    int offsetData = 0;

    // **********************************************************
    // Create Recv Buffers
#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStart("avtOSPRayImageCompositor",
                             "ParallelDirectSendManyPatches", timingDetail,
                             "Create recv buffers");
#endif

    float *recvInfoBuffer = new float[infoBufferSize*6];  // 6 - passing 6 parameters for each patch
    float *recvDataBuffer =  new float[dataBufferSize*4]; // 4 - to account for RGBA

    //debug5 << "infoBufferSize: " << infoBufferSize << ", dataBufferSize: " << dataBufferSize << std::endl;

    if (myRegionHeight != 0)
    {
        // Recv
        recvMetaRq = new MPI_Request[ numRegionsToRecvFrom ];
        recvMetaSt = new MPI_Status [ numRegionsToRecvFrom ];

        recvImageRq = new MPI_Request[ numRegionsToRecvFrom ];
        recvImageSt = new MPI_Status [ numRegionsToRecvFrom ];

        // Async Recv for info
        for (int i=0; i<numRegions; ++i)
        {
            if (recvInfoATABuffer[i*2 + 0] == 0)
              continue;

            if ( regionVector[i] == mpiRank )
              continue;

            int src = regionVector[i];
            MPI_Irecv(&recvInfoBuffer[offsetMeta], recvInfoATABuffer[i*2 + 0]*6, MPI_FLOAT, src, tags[0], MPI_COMM_WORLD,  &recvMetaRq[recvInfoCount] );
            MPI_Irecv(&recvDataBuffer[offsetData], recvInfoATABuffer[i*2 + 1]*4, MPI_FLOAT, src, tags[1], MPI_COMM_WORLD,  &recvImageRq[recvInfoCount] );

            offsetMeta += recvInfoATABuffer[i*2 + 0]*6;
            offsetData += recvInfoATABuffer[i*2 + 1]*4;
            ++recvInfoCount;
        }
        //debug5 << "Async recv setup - numRegionsToRecvFrom: " << numRegionsToRecvFrom << "   recvInfoCount: " << recvInfoCount << endl;
    }

#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStop("avtOSPRayImageCompositor",
                            "ParallelDirectSendManyPatches", timingDetail,
                            "Create recv buffers");
#endif

    // **********************************************************
    // Async Send
#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStart("avtOSPRayImageCompositor",
                             "ParallelDirectSendManyPatches", timingDetail,
                             "Async send");
#endif

    int offset = 0;
    int sendCount = 0;
    int mpiSendCount = 0;
    for (int i=0; i<numRegions; ++i)
    {
        if ( extentsPerPartiton[i].size() != 0 )
        {
            if ( regionVector[i] == mpiRank )
            {
                memcpy( &recvInfoBuffer[offsetMeta], &extentsPerPartiton[i][0], extentsPerPartiton[i].size()*sizeof(float) );
                memcpy( &recvDataBuffer[offsetData], &sendDataBuffer[offset],   sendDataBufferSize[ sendCount ]*sizeof(float) );

                offset += sendDataBufferSize[sendCount];
                ++sendCount;
            }
            else
            {
                MPI_Isend(&extentsPerPartiton[i][0],  extentsPerPartiton[i].size(),  MPI_FLOAT, region[i], tags[0], MPI_COMM_WORLD, &sendMetaRq[mpiSendCount]);
                MPI_Isend(&sendDataBuffer[offset], sendDataBufferSize[ sendCount ], MPI_FLOAT, region[i], tags[1], MPI_COMM_WORLD, &sendImageRq[mpiSendCount]);

                offset += sendDataBufferSize[sendCount];
                ++sendCount;
                ++mpiSendCount;
            }
        }
    }

    //debug5 << "Asyn send setup done ~ numRegionsToSend: " << numRegionsToSend << "  mpiSendCount: " << mpiSendCount << endl;

    if (myRegionHeight != 0)
    {
        //debug5 << "MPI_Waitall ..." << std::endl;
        MPI_Waitall(recvInfoCount, recvImageRq, recvImageSt);   // Means that we have reveived everything!

        //debug5 << "MAPI_WAITALL done!" << std::endl;

        if (recvInfoATABuffer != nullptr)
        {
            delete []recvInfoATABuffer;
            recvInfoATABuffer = nullptr;
        }

        //debug5 << "Sorting..." << std::endl;

        // Sort the data
        std::multimap<float,int> patchData;
        std::vector<int> patchOffset;
        patchOffset.push_back(0);
        for (int i=0; i<infoBufferSize; ++i)
        {
            patchData.insert( std::pair<float,int> (recvInfoBuffer[i*6 + 5],i));
            int _patchSize = (recvInfoBuffer[i*6 + 4]-recvInfoBuffer[i*6 + 3]) * (recvInfoBuffer[i*6 + 2]-recvInfoBuffer[i*6 + 1]) * 4;
            int _offset = patchOffset[i] + _patchSize;

            if (i != infoBufferSize-1)
              patchOffset.push_back(_offset);
        }

        // Create buffer for current region
        intermediateImageBBox[0] = intermediateImageExtents[0] = fullImageExtents[0];
        intermediateImageBBox[1] = intermediateImageExtents[1] = fullImageExtents[1];
        intermediateImageBBox[2] = intermediateImageExtents[2] = myStartingHeight;
        intermediateImageBBox[3] = intermediateImageExtents[3] = myEndingHeight;
        intermediateImage = new float[width * (myEndingHeight - myStartingHeight) * 4]();
        //debug5 << "intermediate image size " << width << ", " << (myEndingHeight - myStartingHeight) << std::endl;

        // Blend
        int numBlends = 0;
        for (std::multimap<float,int>::iterator it=patchData.begin(); it!=patchData.end(); ++it)
        {
            int _id = (*it).second;
            int _extents[4];
            _extents[0] = recvInfoBuffer[_id*6 + 1];
            _extents[1] = recvInfoBuffer[_id*6 + 2];
            _extents[2] = recvInfoBuffer[_id*6 + 3];
            _extents[3] = recvInfoBuffer[_id*6 + 4];
            BlendFrontToBack(&recvDataBuffer[patchOffset[_id]], _extents, _extents, intermediateImage, intermediateImageExtents);
            ++numBlends;
        }

        if (numBlends == 0)
        {
            intermediateImageBBox[0] = intermediateImageBBox[1] =
            intermediateImageBBox[2] = intermediateImageBBox[3] = 0;
        }
    }

    // Means that everything has been sent
    MPI_Waitall(numRegionsToSend, sendImageRq, sendImageSt);

    if (myRegionHeight == 0)
        compositingDone = true;

#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStop("avtOSPRayImageCompositor",
                             "ParallelDirectSendManyPatches", timingDetail,
                             "Async Send");
#endif

    // **********************************************************
    // Cleanup
#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStart("avtOSPRayImageCompositor",
                              "ParallelDirectSendManyPatches", timingDetail,
                              "Cleanup");
#endif
    delete [] sendDataBuffer;
    delete [] sendDataBufferSize;
    delete [] sendDataBufferOffsets;

    sendDataBuffer = nullptr;
    sendDataBufferSize = nullptr;
    sendDataBufferOffsets = nullptr;

    delete [] sendMetaRq;
    delete [] sendImageRq;
    delete [] sendMetaSt;
    delete [] sendImageSt;

    sendMetaSt = nullptr;
    sendMetaRq = nullptr;
    sendImageRq = nullptr;
    sendImageSt = nullptr;

    if (myRegionHeight != 0)
    {
        delete [] recvInfoBuffer;
        delete [] recvDataBuffer;
        delete [] recvMetaRq;
        delete [] recvMetaSt;
        delete [] recvImageRq;
        delete [] recvImageSt;

        recvInfoBuffer = nullptr;
        recvDataBuffer = nullptr;
        recvMetaRq     = nullptr;
        recvImageRq    = nullptr;
        recvInfoBuffer = nullptr;
        recvDataBuffer = nullptr;
        recvMetaRq     = nullptr;
        recvImageRq    = nullptr;
        recvMetaSt     = nullptr;
        recvImageSt    = nullptr;
    }

#ifdef COMMENT_OUT_FOR_NOW
    ospray::CheckSectionStop("avtOSPRayImageCompositor",
                             "ParallelDirectSendManyPatches", timingDetail,
                             "Cleanup");
#endif
    //debug5 << "All Parallel Direct Send is Done" << std::endl;
#endif
    return myRegionHeight;
}

// ***************************************************************************
//  Method: avtOSPRayImageCompositor::gatherImages
//
//  Purpose:
//      Gather images from Parallel Direct Send
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// **************************************************************************
void
avtOSPRayImageCompositor::gatherImages(int regionGather[],
                                       int totalNumRanks,
                                       float * inputImg,
                                       int imgExtents[4],
                                       int boundingBox[4],
                                       int tag,
                                       int fullImageExtents[4],
                                       int myRegionHeight)
{
#ifdef PARALLEL
    //debug5 << "gatherImages starting... totalNumRanks: " << totalNumRanks << ", compositingDone: " << compositingDone
    //     << ", imgExtents: " << imgExtents[0] << ", " << imgExtents[1] << ", " << imgExtents[2] << ", " << imgExtents[3] << std::endl;

    for (int i=0; i<4; ++i)
        finalImageExtents[i] = finalBB[i] = 0;

    if (mpiRank == 0)
    {
        int width =  fullImageExtents[1]-fullImageExtents[0];
        int height = fullImageExtents[3]-fullImageExtents[2];

        //debug5 << "Gather Images at 0, final size: " << fullImageExtents[1]-fullImageExtents[0] << " x " << fullImageExtents[3]-fullImageExtents[2] << std::endl;

        // Receive at root/display node!
        finalImage = new float[width*height*4];
        finalImageExtents[0] = fullImageExtents[0];
        finalImageExtents[1] = fullImageExtents[1];
        finalImageExtents[2] = fullImageExtents[2];
        finalImageExtents[3] = fullImageExtents[3];

        int numRegionsWithData = 0;
        int numToRecv = 0;
        for (int i=0; i<totalNumRanks; ++i)
        {
            if (getRegionSize(i) != 0)
              ++numRegionsWithData;
        }

        numToRecv = numRegionsWithData;

        // Remove itself from the recv
        if (getRegionSize(mpiRank) != 0)
            --numToRecv;

        // Create buffers for async reciving
        MPI_Request *recvImageRq = new MPI_Request[ numToRecv ];
        MPI_Status  *recvImageSt = new MPI_Status [ numToRecv ];

        int lastBufferSize    = getRegionSize(totalNumRanks-1) * width * 4;
        int regularBufferSize = regularRegionSize * width * 4;

        //debug5 << "numToRecv: " << numToRecv << ", numRegionsWithData: " << numRegionsWithData << std::endl;
        //debug5 << "regularBufferSize: " << regularBufferSize << ", lastBufferSize: " << lastBufferSize << std::endl;

        // Async Recv
        int recvCount=0;
        for (int i=0; i<numRegionsWithData; ++i)
        {
            int src = regionGather[i];

            if (src == mpiRank)
              continue;

            if (i == totalNumRanks-1)
            {
                if (lastBufferSize != 0)
                {
                    MPI_Irecv(&finalImage[i*regularBufferSize], lastBufferSize,     MPI_FLOAT, src, tag, MPI_COMM_WORLD,  &recvImageRq[recvCount] );
                }
            }
            else
            {
                MPI_Irecv(&finalImage[i*regularBufferSize], regularBufferSize,  MPI_FLOAT, src, tag, MPI_COMM_WORLD,  &recvImageRq[recvCount] );
            }

            //debug5 << i << " ~ recvCount: " << recvCount << std::endl;
            ++recvCount;
        }

        if (compositingDone == false)   // If root has data for the final image
          PlaceImage(inputImg, imgExtents, finalImage, finalImageExtents);

        MPI_Waitall(numToRecv, recvImageRq, recvImageSt);
        compositingDone = true;

        delete [] recvImageRq;
        delete [] recvImageSt;

        recvImageRq = nullptr;
        recvImageSt = nullptr;
    }
    else
    {
        if (compositingDone == false)
        {
            int imgSize = ((imgExtents[1]-imgExtents[0]) *
                           (imgExtents[3]-imgExtents[2]) * 4);

            //debug5 << "imgSize: " << imgSize << std::endl;

            MPI_Send(inputImg, imgSize, MPI_FLOAT, 0, tag, MPI_COMM_WORLD);
            compositingDone = true;
        }
    }

#endif
}

// ***************************************************************************
//  Method: avtOSPRayImageCompositor::getcompositedImage
//
//  Purpose:
//      Returns the whole image and deletes finial image.
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ***************************************************************************

void avtOSPRayImageCompositor::getcompositedImage(int imgBufferWidth,
                                                  int imgBufferHeight,
                                                  unsigned char *wholeImage)
{
    for (int i=0; i< imgBufferHeight; ++i)
    {
        for (int j=0; j<imgBufferWidth; ++j)
        {
            int bufferIndex   = (imgBufferWidth*4*i) + (j*4);
            int wholeImgIndex = (imgBufferWidth*3*i) + (j*3);

            wholeImage[wholeImgIndex+0] = finalImage[bufferIndex+0] * 255;
            wholeImage[wholeImgIndex+1] = finalImage[bufferIndex+1] * 255;
            wholeImage[wholeImgIndex+2] = finalImage[bufferIndex+2] * 255;
        }
    }

    if (finalImage != nullptr)
    {
        delete [] finalImage;
        finalImage = nullptr;
    }
}
