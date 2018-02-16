// ***************************************************************************
//
// Copyright (c) 2000 - 2018, Lawrence Livermore National Security, LLC
// Produced at the Lawrence Livermore National Laboratory
// LLNL-CODE-442911
// All rights reserved.
//
// This file is  part of VisIt. For  details, see https://visit.llnl.gov/.  The
// full copyright notice is contained in the file COPYRIGHT located at the root
// of the VisIt distribution or at http://www.llnl.gov/visit/copyright.html.
//
// Redistribution  and  use  in  source  and  binary  forms,  with  or  without
// modification, are permitted provided that the following conditions are met:
//
//  - Redistributions of  source code must  retain the above  copyright notice,
//    this list of conditions and the disclaimer below.
//  - Redistributions in binary form must reproduce the above copyright notice,
//    this  list of  conditions  and  the  disclaimer (as noted below)  in  the
//    documentation and/or other materials provided with the distribution.
//  - Neither the name of  the LLNS/LLNL nor the names of  its contributors may
//    be used to endorse or promote products derived from this software without
//    specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR  IMPLIED WARRANTIES, INCLUDING,  BUT NOT  LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND  FITNESS FOR A PARTICULAR  PURPOSE
// ARE  DISCLAIMED. IN  NO EVENT  SHALL LAWRENCE  LIVERMORE NATIONAL  SECURITY,
// LLC, THE  U.S.  DEPARTMENT OF  ENERGY  OR  CONTRIBUTORS BE  LIABLE  FOR  ANY
// DIRECT,  INDIRECT,   INCIDENTAL,   SPECIAL,   EXEMPLARY,  OR   CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT  LIMITED TO, PROCUREMENT OF  SUBSTITUTE GOODS OR
// SERVICES; LOSS OF  USE, DATA, OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER
// CAUSED  AND  ON  ANY  THEORY  OF  LIABILITY,  WHETHER  IN  CONTRACT,  STRICT
// LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE)  ARISING IN ANY  WAY
// OUT OF THE  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// ***************************************************************************

package llnl.visit.plots;

import llnl.visit.AttributeSubject;
import llnl.visit.CommunicationBuffer;
import llnl.visit.Plugin;
import llnl.visit.ColorAttribute;

// ****************************************************************************
// Class: TensorAttributes
//
// Purpose:
//    Attributes for the tensor plot
//
// Notes:      Autogenerated by xml2java.
//
// Programmer: xml2java
// Creation:   omitted
//
// Modifications:
//   
// ****************************************************************************

public class TensorAttributes extends AttributeSubject implements Plugin
{
    private static int TensorAttributes_numAdditionalAtts = 11;

    public TensorAttributes()
    {
        super(TensorAttributes_numAdditionalAtts);

        useStride = false;
        stride = 1;
        nTensors = 400;
        scale = 0.25;
        scaleByMagnitude = true;
        autoScale = true;
        colorByEigenvalues = true;
        useLegend = true;
        tensorColor = new ColorAttribute(0, 0, 0);
        colorTableName = new String("Default");
        invertColorTable = false;
    }

    public TensorAttributes(int nMoreFields)
    {
        super(TensorAttributes_numAdditionalAtts + nMoreFields);

        useStride = false;
        stride = 1;
        nTensors = 400;
        scale = 0.25;
        scaleByMagnitude = true;
        autoScale = true;
        colorByEigenvalues = true;
        useLegend = true;
        tensorColor = new ColorAttribute(0, 0, 0);
        colorTableName = new String("Default");
        invertColorTable = false;
    }

    public TensorAttributes(TensorAttributes obj)
    {
        super(obj);

        useStride = obj.useStride;
        stride = obj.stride;
        nTensors = obj.nTensors;
        scale = obj.scale;
        scaleByMagnitude = obj.scaleByMagnitude;
        autoScale = obj.autoScale;
        colorByEigenvalues = obj.colorByEigenvalues;
        useLegend = obj.useLegend;
        tensorColor = new ColorAttribute(obj.tensorColor);
        colorTableName = new String(obj.colorTableName);
        invertColorTable = obj.invertColorTable;

        SelectAll();
    }

    public int Offset()
    {
        return super.Offset() + super.GetNumAdditionalAttributes();
    }

    public int GetNumAdditionalAttributes()
    {
        return TensorAttributes_numAdditionalAtts;
    }

    public boolean equals(TensorAttributes obj)
    {
        // Create the return value
        return ((useStride == obj.useStride) &&
                (stride == obj.stride) &&
                (nTensors == obj.nTensors) &&
                (scale == obj.scale) &&
                (scaleByMagnitude == obj.scaleByMagnitude) &&
                (autoScale == obj.autoScale) &&
                (colorByEigenvalues == obj.colorByEigenvalues) &&
                (useLegend == obj.useLegend) &&
                (tensorColor == obj.tensorColor) &&
                (colorTableName.equals(obj.colorTableName)) &&
                (invertColorTable == obj.invertColorTable));
    }

    public String GetName() { return "Tensor"; }
    public String GetVersion() { return "1.0"; }

    // Property setting methods
    public void SetUseStride(boolean useStride_)
    {
        useStride = useStride_;
        Select(0);
    }

    public void SetStride(int stride_)
    {
        stride = stride_;
        Select(1);
    }

    public void SetNTensors(int nTensors_)
    {
        nTensors = nTensors_;
        Select(2);
    }

    public void SetScale(double scale_)
    {
        scale = scale_;
        Select(3);
    }

    public void SetScaleByMagnitude(boolean scaleByMagnitude_)
    {
        scaleByMagnitude = scaleByMagnitude_;
        Select(4);
    }

    public void SetAutoScale(boolean autoScale_)
    {
        autoScale = autoScale_;
        Select(5);
    }

    public void SetColorByEigenvalues(boolean colorByEigenvalues_)
    {
        colorByEigenvalues = colorByEigenvalues_;
        Select(6);
    }

    public void SetUseLegend(boolean useLegend_)
    {
        useLegend = useLegend_;
        Select(7);
    }

    public void SetTensorColor(ColorAttribute tensorColor_)
    {
        tensorColor = tensorColor_;
        Select(8);
    }

    public void SetColorTableName(String colorTableName_)
    {
        colorTableName = colorTableName_;
        Select(9);
    }

    public void SetInvertColorTable(boolean invertColorTable_)
    {
        invertColorTable = invertColorTable_;
        Select(10);
    }

    // Property getting methods
    public boolean        GetUseStride() { return useStride; }
    public int            GetStride() { return stride; }
    public int            GetNTensors() { return nTensors; }
    public double         GetScale() { return scale; }
    public boolean        GetScaleByMagnitude() { return scaleByMagnitude; }
    public boolean        GetAutoScale() { return autoScale; }
    public boolean        GetColorByEigenvalues() { return colorByEigenvalues; }
    public boolean        GetUseLegend() { return useLegend; }
    public ColorAttribute GetTensorColor() { return tensorColor; }
    public String         GetColorTableName() { return colorTableName; }
    public boolean        GetInvertColorTable() { return invertColorTable; }

    // Write and read methods.
    public void WriteAtts(CommunicationBuffer buf)
    {
        if(WriteSelect(0, buf))
            buf.WriteBool(useStride);
        if(WriteSelect(1, buf))
            buf.WriteInt(stride);
        if(WriteSelect(2, buf))
            buf.WriteInt(nTensors);
        if(WriteSelect(3, buf))
            buf.WriteDouble(scale);
        if(WriteSelect(4, buf))
            buf.WriteBool(scaleByMagnitude);
        if(WriteSelect(5, buf))
            buf.WriteBool(autoScale);
        if(WriteSelect(6, buf))
            buf.WriteBool(colorByEigenvalues);
        if(WriteSelect(7, buf))
            buf.WriteBool(useLegend);
        if(WriteSelect(8, buf))
            tensorColor.Write(buf);
        if(WriteSelect(9, buf))
            buf.WriteString(colorTableName);
        if(WriteSelect(10, buf))
            buf.WriteBool(invertColorTable);
    }

    public void ReadAtts(int index, CommunicationBuffer buf)
    {
        switch(index)
        {
        case 0:
            SetUseStride(buf.ReadBool());
            break;
        case 1:
            SetStride(buf.ReadInt());
            break;
        case 2:
            SetNTensors(buf.ReadInt());
            break;
        case 3:
            SetScale(buf.ReadDouble());
            break;
        case 4:
            SetScaleByMagnitude(buf.ReadBool());
            break;
        case 5:
            SetAutoScale(buf.ReadBool());
            break;
        case 6:
            SetColorByEigenvalues(buf.ReadBool());
            break;
        case 7:
            SetUseLegend(buf.ReadBool());
            break;
        case 8:
            tensorColor.Read(buf);
            Select(8);
            break;
        case 9:
            SetColorTableName(buf.ReadString());
            break;
        case 10:
            SetInvertColorTable(buf.ReadBool());
            break;
        }
    }

    public String toString(String indent)
    {
        String str = new String();
        str = str + boolToString("useStride", useStride, indent) + "\n";
        str = str + intToString("stride", stride, indent) + "\n";
        str = str + intToString("nTensors", nTensors, indent) + "\n";
        str = str + doubleToString("scale", scale, indent) + "\n";
        str = str + boolToString("scaleByMagnitude", scaleByMagnitude, indent) + "\n";
        str = str + boolToString("autoScale", autoScale, indent) + "\n";
        str = str + boolToString("colorByEigenvalues", colorByEigenvalues, indent) + "\n";
        str = str + boolToString("useLegend", useLegend, indent) + "\n";
        str = str + indent + "tensorColor = {" + tensorColor.Red() + ", " + tensorColor.Green() + ", " + tensorColor.Blue() + ", " + tensorColor.Alpha() + "}\n";
        str = str + stringToString("colorTableName", colorTableName, indent) + "\n";
        str = str + boolToString("invertColorTable", invertColorTable, indent) + "\n";
        return str;
    }


    // Attributes
    private boolean        useStride;
    private int            stride;
    private int            nTensors;
    private double         scale;
    private boolean        scaleByMagnitude;
    private boolean        autoScale;
    private boolean        colorByEigenvalues;
    private boolean        useLegend;
    private ColorAttribute tensorColor;
    private String         colorTableName;
    private boolean        invertColorTable;
}

