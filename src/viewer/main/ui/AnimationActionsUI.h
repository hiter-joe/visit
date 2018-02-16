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

#ifndef ANIMATION_ACTION_UI_H
#define ANIMATION_ACTION_UI_H
#include <viewer_exports.h>
#include <ViewerActionUISingle.h>
#include <ViewerActionUIToggle.h>

// ****************************************************************************
// Class: TimeSliderReverseStepActionUI
//
// Purpose:
//   Handles the reverse step through an animation ActionUI.
//
// Notes:      
//
// Programmer: Brad Whitlock
// Creation:   Wed Feb 5 15:56:39 PST 2003
//
// Modifications:
//   
// ****************************************************************************

class VIEWER_API TimeSliderReverseStepActionUI : public ViewerActionUISingle
{
public:
    TimeSliderReverseStepActionUI(ViewerActionLogic *L);
    virtual ~TimeSliderReverseStepActionUI() { }

    virtual bool Enabled() const;
};

// ****************************************************************************
// Class: AnimationReversePlayActionUI
//
// Purpose:
//   Handles the reverse play through an animation ActionUI.
//
// Notes:      
//
// Programmer: Brad Whitlock
// Creation:   Wed Feb 5 15:56:42 PST 2003
//
// Modifications:
//   
// ****************************************************************************

class VIEWER_API AnimationReversePlayActionUI : public ViewerActionUIToggle
{
public:
    AnimationReversePlayActionUI(ViewerActionLogic *L);
    virtual ~AnimationReversePlayActionUI() { }

    virtual bool Enabled() const;
    virtual bool Checked() const;
};

// ****************************************************************************
// Class: AnimationStopActionUI
//
// Purpose:
//   Handles the stop ActionUI for an animation.
//
// Notes:      
//
// Programmer: Brad Whitlock
// Creation:   Wed Feb 5 15:57:23 PST 2003
//
// Modifications:
//   
// ****************************************************************************

class VIEWER_API AnimationStopActionUI : public ViewerActionUIToggle
{
public:
    AnimationStopActionUI(ViewerActionLogic *L);
    virtual ~AnimationStopActionUI() { }

    virtual bool Enabled() const;
    virtual bool Checked() const;
};

// ****************************************************************************
// Class: AnimationPlayActionUI
//
// Purpose:
//   Handles the play ActionUI for an animation.
//
// Notes:      
//
// Programmer: Brad Whitlock
// Creation:   Wed Feb 5 15:58:23 PST 2003
//
// Modifications:
//   
// ****************************************************************************

class VIEWER_API AnimationPlayActionUI : public ViewerActionUIToggle
{
public:
    AnimationPlayActionUI(ViewerActionLogic *L);
    virtual ~AnimationPlayActionUI() { }

    virtual bool Enabled() const;
    virtual bool Checked() const;
};

// ****************************************************************************
// Class: TimeSliderForwardStepActionUI
//
// Purpose:
//   Handles the forward step ActionUI for an animation.
//
// Notes:      
//
// Programmer: Brad Whitlock
// Creation:   Wed Feb 5 15:58:43 PST 2003
//
// Modifications:
//   
// ****************************************************************************

class VIEWER_API TimeSliderForwardStepActionUI : public ViewerActionUISingle
{
public:
    TimeSliderForwardStepActionUI(ViewerActionLogic *L);
    virtual ~TimeSliderForwardStepActionUI() { }

    virtual bool Enabled() const;
};

#endif
