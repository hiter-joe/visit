/*****************************************************************************
*
* Copyright (c) 2000 - 2018, Lawrence Livermore National Security, LLC
* Produced at the Lawrence Livermore National Laboratory
* LLNL-CODE-442911
* All rights reserved.
*
* This file is  part of VisIt. For  details, see https://visit.llnl.gov/.  The
* full copyright notice is contained in the file COPYRIGHT located at the root
* of the VisIt distribution or at http://www.llnl.gov/visit/copyright.html.
*
* Redistribution  and  use  in  source  and  binary  forms,  with  or  without
* modification, are permitted provided that the following conditions are met:
*
*  - Redistributions of  source code must  retain the above  copyright notice,
*    this list of conditions and the disclaimer below.
*  - Redistributions in binary form must reproduce the above copyright notice,
*    this  list of  conditions  and  the  disclaimer (as noted below)  in  the
*    documentation and/or other materials provided with the distribution.
*  - Neither the name of  the LLNS/LLNL nor the names of  its contributors may
*    be used to endorse or promote products derived from this software without
*    specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR  IMPLIED WARRANTIES, INCLUDING,  BUT NOT  LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND  FITNESS FOR A PARTICULAR  PURPOSE
* ARE  DISCLAIMED. IN  NO EVENT  SHALL LAWRENCE  LIVERMORE NATIONAL  SECURITY,
* LLC, THE  U.S.  DEPARTMENT OF  ENERGY  OR  CONTRIBUTORS BE  LIABLE  FOR  ANY
* DIRECT,  INDIRECT,   INCIDENTAL,   SPECIAL,   EXEMPLARY,  OR   CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT  LIMITED TO, PROCUREMENT OF  SUBSTITUTE GOODS OR
* SERVICES; LOSS OF  USE, DATA, OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER
* CAUSED  AND  ON  ANY  THEORY  OF  LIABILITY,  WHETHER  IN  CONTRACT,  STRICT
* LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE)  ARISING IN ANY  WAY
* OUT OF THE  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGE.
*
*****************************************************************************/

#ifndef SURFACEATTRIBUTES_H
#define SURFACEATTRIBUTES_H
#include <string>
#include <AttributeSubject.h>

#include <ColorAttribute.h>

// ****************************************************************************
// Class: SurfaceAttributes
//
// Purpose:
//    Attributes for the surface plot
//
// Notes:      Autogenerated by xml2atts.
//
// Programmer: xml2atts
// Creation:   omitted
//
// Modifications:
//   
// ****************************************************************************

class SurfaceAttributes : public AttributeSubject
{
public:
    enum ColorByType
    {
        Constant,
        ZValue
    };
    enum Scaling
    {
        Linear,
        Log,
        Skew
    };
    enum LimitsMode
    {
        OriginalData,
        CurrentPlot
    };

    // These constructors are for objects of this class
    SurfaceAttributes();
    SurfaceAttributes(const SurfaceAttributes &obj);
protected:
    // These constructors are for objects derived from this class
    SurfaceAttributes(private_tmfs_t tmfs);
    SurfaceAttributes(const SurfaceAttributes &obj, private_tmfs_t tmfs);
public:
    virtual ~SurfaceAttributes();

    virtual SurfaceAttributes& operator = (const SurfaceAttributes &obj);
    virtual bool operator == (const SurfaceAttributes &obj) const;
    virtual bool operator != (const SurfaceAttributes &obj) const;
private:
    void Init();
    void Copy(const SurfaceAttributes &obj);
public:

    virtual const std::string TypeName() const;
    virtual bool CopyAttributes(const AttributeGroup *);
    virtual AttributeSubject *CreateCompatible(const std::string &) const;
    virtual AttributeSubject *NewInstance(bool) const;

    // Property selection methods
    virtual void SelectAll();
    void SelectSurfaceColor();
    void SelectWireframeColor();
    void SelectColorTableName();

    // Property setting methods
    void SetLegendFlag(bool legendFlag_);
    void SetLightingFlag(bool lightingFlag_);
    void SetSurfaceFlag(bool surfaceFlag_);
    void SetWireframeFlag(bool wireframeFlag_);
    void SetLimitsMode(LimitsMode limitsMode_);
    void SetMinFlag(bool minFlag_);
    void SetMaxFlag(bool maxFlag_);
    void SetColorByZFlag(bool colorByZFlag_);
    void SetScaling(Scaling scaling_);
    void SetLineStyle(int lineStyle_);
    void SetLineWidth(int lineWidth_);
    void SetSurfaceColor(const ColorAttribute &surfaceColor_);
    void SetWireframeColor(const ColorAttribute &wireframeColor_);
    void SetSkewFactor(double skewFactor_);
    void SetMin(double min_);
    void SetMax(double max_);
    void SetColorTableName(const std::string &colorTableName_);
    void SetInvertColorTable(bool invertColorTable_);

    // Property getting methods
    bool                 GetLegendFlag() const;
    bool                 GetLightingFlag() const;
    bool                 GetSurfaceFlag() const;
    bool                 GetWireframeFlag() const;
    LimitsMode           GetLimitsMode() const;
    bool                 GetMinFlag() const;
    bool                 GetMaxFlag() const;
    bool                 GetColorByZFlag() const;
    Scaling              GetScaling() const;
    int                  GetLineStyle() const;
    int                  GetLineWidth() const;
    const ColorAttribute &GetSurfaceColor() const;
          ColorAttribute &GetSurfaceColor();
    const ColorAttribute &GetWireframeColor() const;
          ColorAttribute &GetWireframeColor();
    double               GetSkewFactor() const;
    double               GetMin() const;
    double               GetMax() const;
    const std::string    &GetColorTableName() const;
          std::string    &GetColorTableName();
    bool                 GetInvertColorTable() const;

    // Persistence methods
    virtual bool CreateNode(DataNode *node, bool completeSave, bool forceAdd);
    virtual void SetFromNode(DataNode *node);

    // Enum conversion functions
    static std::string ColorByType_ToString(ColorByType);
    static bool ColorByType_FromString(const std::string &, ColorByType &);
protected:
    static std::string ColorByType_ToString(int);
public:
    static std::string Scaling_ToString(Scaling);
    static bool Scaling_FromString(const std::string &, Scaling &);
protected:
    static std::string Scaling_ToString(int);
public:
    static std::string LimitsMode_ToString(LimitsMode);
    static bool LimitsMode_FromString(const std::string &, LimitsMode &);
protected:
    static std::string LimitsMode_ToString(int);
public:

    // Keyframing methods
    virtual std::string               GetFieldName(int index) const;
    virtual AttributeGroup::FieldType GetFieldType(int index) const;
    virtual std::string               GetFieldTypeName(int index) const;
    virtual bool                      FieldsEqual(int index, const AttributeGroup *rhs) const;

    // User-defined methods
    bool ChangesRequireRecalculation(const SurfaceAttributes &) const;

    // IDs that can be used to identify fields in case statements
    enum {
        ID_legendFlag = 0,
        ID_lightingFlag,
        ID_surfaceFlag,
        ID_wireframeFlag,
        ID_limitsMode,
        ID_minFlag,
        ID_maxFlag,
        ID_colorByZFlag,
        ID_scaling,
        ID_lineStyle,
        ID_lineWidth,
        ID_surfaceColor,
        ID_wireframeColor,
        ID_skewFactor,
        ID_min,
        ID_max,
        ID_colorTableName,
        ID_invertColorTable,
        ID__LAST
    };

private:
    bool           legendFlag;
    bool           lightingFlag;
    bool           surfaceFlag;
    bool           wireframeFlag;
    int            limitsMode;
    bool           minFlag;
    bool           maxFlag;
    bool           colorByZFlag;
    int            scaling;
    int            lineStyle;
    int            lineWidth;
    ColorAttribute surfaceColor;
    ColorAttribute wireframeColor;
    double         skewFactor;
    double         min;
    double         max;
    std::string    colorTableName;
    bool           invertColorTable;

    // Static class format string for type map.
    static const char *TypeMapFormatString;
    static const private_tmfs_t TmfsStruct;
};
#define SURFACEATTRIBUTES_TMFS "bbbbibbbiiiaadddsb"

#endif
