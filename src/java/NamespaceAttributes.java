// ***************************************************************************
//
// Copyright (c) 2000 - 2008, Lawrence Livermore National Security, LLC
// Produced at the Lawrence Livermore National Laboratory
// LLNL-CODE-400142
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

package llnl.visit;

import java.lang.Integer;
import java.util.Vector;

// ****************************************************************************
// Class: NamespaceAttributes
//
// Purpose:
//    This class contain the information needed to represent a namespace.
//
// Notes:      Autogenerated by xml2java.
//
// Programmer: xml2java
// Creation:   omitted
//
// Modifications:
//   
// ****************************************************************************

public class NamespaceAttributes extends AttributeSubject
{
    public NamespaceAttributes()
    {
        super(4);

        type = -1;
        subsets = new Vector();
        min = -1;
        max = -1;
    }

    public NamespaceAttributes(NamespaceAttributes obj)
    {
        super(4);

        int i;

        type = obj.type;
        subsets = new Vector();
        for(i = 0; i < obj.subsets.size(); ++i)
        {
            Integer iv = (Integer)obj.subsets.elementAt(i);
            subsets.addElement(new Integer(iv.intValue()));
        }
        min = obj.min;
        max = obj.max;

        SelectAll();
    }

    public boolean equals(NamespaceAttributes obj)
    {
        int i;

        // Compare the elements in the subsets vector.
        boolean subsets_equal = (obj.subsets.size() == subsets.size());
        for(i = 0; (i < subsets.size()) && subsets_equal; ++i)
        {
            // Make references to Integer from Object.
            Integer subsets1 = (Integer)subsets.elementAt(i);
            Integer subsets2 = (Integer)obj.subsets.elementAt(i);
            subsets_equal = subsets1.equals(subsets2);
        }
        // Create the return value
        return ((type == obj.type) &&
                subsets_equal &&
                (min == obj.min) &&
                (max == obj.max));
    }

    // Property setting methods
    public void SetType(int type_)
    {
        type = type_;
        Select(0);
    }

    public void SetSubsets(Vector subsets_)
    {
        subsets = subsets_;
        Select(1);
    }

    public void SetMin(int min_)
    {
        min = min_;
        Select(2);
    }

    public void SetMax(int max_)
    {
        max = max_;
        Select(3);
    }

    // Property getting methods
    public int    GetType() { return type; }
    public Vector GetSubsets() { return subsets; }
    public int    GetMin() { return min; }
    public int    GetMax() { return max; }

    // Write and read methods.
    public void WriteAtts(CommunicationBuffer buf)
    {
        if(WriteSelect(0, buf))
            buf.WriteInt(type);
        if(WriteSelect(1, buf))
            buf.WriteIntVector(subsets);
        if(WriteSelect(2, buf))
            buf.WriteInt(min);
        if(WriteSelect(3, buf))
            buf.WriteInt(max);
    }

    public void ReadAtts(int n, CommunicationBuffer buf)
    {
        for(int i = 0; i < n; ++i)
        {
            int index = (int)buf.ReadByte();
            switch(index)
            {
            case 0:
                SetType(buf.ReadInt());
                break;
            case 1:
                SetSubsets(buf.ReadIntVector());
                break;
            case 2:
                SetMin(buf.ReadInt());
                break;
            case 3:
                SetMax(buf.ReadInt());
                break;
            }
        }
    }

    public String toString(String indent)
    {
        String str = new String();
        str = str + intToString("type", type, indent) + "\n";
        str = str + intVectorToString("subsets", subsets, indent) + "\n";
        str = str + intToString("min", min, indent) + "\n";
        str = str + intToString("max", max, indent) + "\n";
        return str;
    }


    // Attributes
    private int    type;
    private Vector subsets; // vector of Integer objects
    private int    min;
    private int    max;
}

