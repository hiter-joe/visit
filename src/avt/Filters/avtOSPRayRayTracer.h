// Copyright (c) Lawrence Livermore National Security, LLC and other VisIt
// Project developers.  See the top-level LICENSE file for dates and other
// details.  No copyright assignment is required to contribute to VisIt.

// ************************************************************************* //
//                           avtOSPRayRayTracer.h                            //
// ************************************************************************* //

#ifndef AVT_OSPRAY_RAY_TRACER_H
#define AVT_OSPRAY_RAY_TRACER_H

#include <filters_exports.h>

#include <avtRayTracerBase.h>
#include <avtOSPRayCommon.h>
#include <avtOSPRayImageCompositor.h>
#include <LightList.h>

class   vtkMatrix4x4;

// ****************************************************************************
//  Class: avtOSPRayRayTracer
//
//  Purpose:
//      Performs ray tracing, taking in a dataset as a source and has an
//      image as an output.
//
//  Programmer: Qi Wu
//  Creation:   June 18, 2018
//
//  Modifications:
//
// ****************************************************************************

class AVTFILTERS_API avtOSPRayRayTracer : public avtRayTracerBase
{
public:
    avtOSPRayRayTracer();
    virtual              ~avtOSPRayRayTracer();

    virtual const char   *GetType(void)      { return "avtOSPRayRayTracer"; };
    virtual const char   *GetDescription(void) 
                                             { return "OSPRay Ray tracing"; };

    void SetActiveVariable(const char* s)             { activeVariable = s; };
    void SetLightInfo(const LightList& l)                  { lightList = l; };
    // void SetOSPRay(OSPVisItContext *ptr)               { ospray_core = ptr; };

    void SetLighting(bool l)                  { gradientShadingEnabled = l; };
    void SetShadowsEnabled(bool l)                    { shadowsEnabled = l; };
    void SetUseGridAccelerator(bool l)            { useGridAccelerator = l; };
    void SetPreIntegration(bool l)                    { preIntegration = l; };
    void SetSingleShade(bool l)                          { singleShade = l; };
    void SetOneSidedLighting(bool l)                { oneSidedLighting = l; };
    void SetAoTransparencyEnabled(bool l)      { aoTransparencyEnabled = l; };

    void SetAoSamples(int v)                               { aoSamples = v; };
    void SetSpp(int v)                                           { spp = v; };
    
    void SetAoDistance(double v)                          { aoDistance = v; };
    void SetSamplingRate(double v)                      { samplingRate = v; };
    void SetMinContribution(double v)                { minContribution = v; };
    
    void SetMatProperties(double v[4]) 
                    { for (int i=0; i<4; i++) materialProperties[i] = v[i]; };
    void SetViewDirection(double v[3])
                         { for (int i=0; i<3; i++) viewDirection[i] = v[i]; };
protected:
    virtual void             Execute(void);

    const char*              activeVariable;
    LightList                lightList;
    // OSPVisItContext         *ospray_core;

    bool                     gradientShadingEnabled;
    bool                     shadowsEnabled;
    bool                     useGridAccelerator;
    bool                     preIntegration;
    bool                     singleShade;
    bool                     oneSidedLighting;
    bool                     aoTransparencyEnabled;
    int                      spp;
    int                      aoSamples;
    double                   aoDistance;
    double                   samplingRate;
    double                   minContribution;
    
    double                   materialProperties[4];
    double                   viewDirection[3];

    avtOSPRayImageCompositor imgComm;
};

#endif
