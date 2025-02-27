// Copyright (c) Lawrence Livermore National Security, LLC and other VisIt
// Project developers.  See the top-level LICENSE file for dates and other
// details.  No copyright assignment is required to contribute to VisIt.

#include <PyInverseGhostZoneAttributes.h>
#include <ObserverToCallback.h>
#include <stdio.h>
#include <Py2and3Support.h>

// ****************************************************************************
// Module: PyInverseGhostZoneAttributes
//
// Purpose:
//   This class contains attributes for the inverse ghost zone operator.
//
// Note:       Autogenerated by xml2python. Do not modify by hand!
//
// Programmer: xml2python
// Creation:   omitted
//
// ****************************************************************************

//
// This struct contains the Python type information and a InverseGhostZoneAttributes.
//
struct InverseGhostZoneAttributesObject
{
    PyObject_HEAD
    InverseGhostZoneAttributes *data;
    bool        owns;
    PyObject   *parent;
};

//
// Internal prototypes
//
static PyObject *NewInverseGhostZoneAttributes(int);
std::string
PyInverseGhostZoneAttributes_ToString(const InverseGhostZoneAttributes *atts, const char *prefix)
{
    std::string str;
    char tmpStr[1000];

    if(atts->GetRequestGhostZones())
        snprintf(tmpStr, 1000, "%srequestGhostZones = 1\n", prefix);
    else
        snprintf(tmpStr, 1000, "%srequestGhostZones = 0\n", prefix);
    str += tmpStr;
    if(atts->GetShowDuplicated())
        snprintf(tmpStr, 1000, "%sshowDuplicated = 1\n", prefix);
    else
        snprintf(tmpStr, 1000, "%sshowDuplicated = 0\n", prefix);
    str += tmpStr;
    if(atts->GetShowEnhancedConnectivity())
        snprintf(tmpStr, 1000, "%sshowEnhancedConnectivity = 1\n", prefix);
    else
        snprintf(tmpStr, 1000, "%sshowEnhancedConnectivity = 0\n", prefix);
    str += tmpStr;
    if(atts->GetShowReducedConnectivity())
        snprintf(tmpStr, 1000, "%sshowReducedConnectivity = 1\n", prefix);
    else
        snprintf(tmpStr, 1000, "%sshowReducedConnectivity = 0\n", prefix);
    str += tmpStr;
    if(atts->GetShowAMRRefined())
        snprintf(tmpStr, 1000, "%sshowAMRRefined = 1\n", prefix);
    else
        snprintf(tmpStr, 1000, "%sshowAMRRefined = 0\n", prefix);
    str += tmpStr;
    if(atts->GetShowExterior())
        snprintf(tmpStr, 1000, "%sshowExterior = 1\n", prefix);
    else
        snprintf(tmpStr, 1000, "%sshowExterior = 0\n", prefix);
    str += tmpStr;
    if(atts->GetShowNotApplicable())
        snprintf(tmpStr, 1000, "%sshowNotApplicable = 1\n", prefix);
    else
        snprintf(tmpStr, 1000, "%sshowNotApplicable = 0\n", prefix);
    str += tmpStr;
    return str;
}

static PyObject *
InverseGhostZoneAttributes_Notify(PyObject *self, PyObject *args)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)self;
    obj->data->Notify();
    Py_INCREF(Py_None);
    return Py_None;
}

/*static*/ PyObject *
InverseGhostZoneAttributes_SetRequestGhostZones(PyObject *self, PyObject *args)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)self;

    PyObject *packaged_args = 0;

    // Handle args packaged into a tuple of size one
    // if we think the unpackaged args matches our needs
    if (PySequence_Check(args) && PySequence_Size(args) == 1)
    {
        packaged_args = PySequence_GetItem(args, 0);
        if (PyNumber_Check(packaged_args))
            args = packaged_args;
    }

    if (PySequence_Check(args))
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_TypeError, "expecting a single number arg");
    }

    if (!PyNumber_Check(args))
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_TypeError, "arg is not a number type");
    }

    long val = PyLong_AsLong(args);
    bool cval = bool(val);

    if (val == -1 && PyErr_Occurred())
    {
        Py_XDECREF(packaged_args);
        PyErr_Clear();
        return PyErr_Format(PyExc_TypeError, "arg not interpretable as C++ bool");
    }
    if (fabs(double(val))>1.5E-7 && fabs((double(long(cval))-double(val))/double(val))>1.5E-7)
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_ValueError, "arg not interpretable as C++ bool");
    }

    Py_XDECREF(packaged_args);

    // Set the requestGhostZones in the object.
    obj->data->SetRequestGhostZones(cval);

    Py_INCREF(Py_None);
    return Py_None;
}

/*static*/ PyObject *
InverseGhostZoneAttributes_GetRequestGhostZones(PyObject *self, PyObject *args)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)self;
    PyObject *retval = PyInt_FromLong(obj->data->GetRequestGhostZones()?1L:0L);
    return retval;
}

/*static*/ PyObject *
InverseGhostZoneAttributes_SetShowDuplicated(PyObject *self, PyObject *args)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)self;

    PyObject *packaged_args = 0;

    // Handle args packaged into a tuple of size one
    // if we think the unpackaged args matches our needs
    if (PySequence_Check(args) && PySequence_Size(args) == 1)
    {
        packaged_args = PySequence_GetItem(args, 0);
        if (PyNumber_Check(packaged_args))
            args = packaged_args;
    }

    if (PySequence_Check(args))
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_TypeError, "expecting a single number arg");
    }

    if (!PyNumber_Check(args))
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_TypeError, "arg is not a number type");
    }

    long val = PyLong_AsLong(args);
    bool cval = bool(val);

    if (val == -1 && PyErr_Occurred())
    {
        Py_XDECREF(packaged_args);
        PyErr_Clear();
        return PyErr_Format(PyExc_TypeError, "arg not interpretable as C++ bool");
    }
    if (fabs(double(val))>1.5E-7 && fabs((double(long(cval))-double(val))/double(val))>1.5E-7)
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_ValueError, "arg not interpretable as C++ bool");
    }

    Py_XDECREF(packaged_args);

    // Set the showDuplicated in the object.
    obj->data->SetShowDuplicated(cval);

    Py_INCREF(Py_None);
    return Py_None;
}

/*static*/ PyObject *
InverseGhostZoneAttributes_GetShowDuplicated(PyObject *self, PyObject *args)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)self;
    PyObject *retval = PyInt_FromLong(obj->data->GetShowDuplicated()?1L:0L);
    return retval;
}

/*static*/ PyObject *
InverseGhostZoneAttributes_SetShowEnhancedConnectivity(PyObject *self, PyObject *args)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)self;

    PyObject *packaged_args = 0;

    // Handle args packaged into a tuple of size one
    // if we think the unpackaged args matches our needs
    if (PySequence_Check(args) && PySequence_Size(args) == 1)
    {
        packaged_args = PySequence_GetItem(args, 0);
        if (PyNumber_Check(packaged_args))
            args = packaged_args;
    }

    if (PySequence_Check(args))
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_TypeError, "expecting a single number arg");
    }

    if (!PyNumber_Check(args))
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_TypeError, "arg is not a number type");
    }

    long val = PyLong_AsLong(args);
    bool cval = bool(val);

    if (val == -1 && PyErr_Occurred())
    {
        Py_XDECREF(packaged_args);
        PyErr_Clear();
        return PyErr_Format(PyExc_TypeError, "arg not interpretable as C++ bool");
    }
    if (fabs(double(val))>1.5E-7 && fabs((double(long(cval))-double(val))/double(val))>1.5E-7)
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_ValueError, "arg not interpretable as C++ bool");
    }

    Py_XDECREF(packaged_args);

    // Set the showEnhancedConnectivity in the object.
    obj->data->SetShowEnhancedConnectivity(cval);

    Py_INCREF(Py_None);
    return Py_None;
}

/*static*/ PyObject *
InverseGhostZoneAttributes_GetShowEnhancedConnectivity(PyObject *self, PyObject *args)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)self;
    PyObject *retval = PyInt_FromLong(obj->data->GetShowEnhancedConnectivity()?1L:0L);
    return retval;
}

/*static*/ PyObject *
InverseGhostZoneAttributes_SetShowReducedConnectivity(PyObject *self, PyObject *args)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)self;

    PyObject *packaged_args = 0;

    // Handle args packaged into a tuple of size one
    // if we think the unpackaged args matches our needs
    if (PySequence_Check(args) && PySequence_Size(args) == 1)
    {
        packaged_args = PySequence_GetItem(args, 0);
        if (PyNumber_Check(packaged_args))
            args = packaged_args;
    }

    if (PySequence_Check(args))
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_TypeError, "expecting a single number arg");
    }

    if (!PyNumber_Check(args))
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_TypeError, "arg is not a number type");
    }

    long val = PyLong_AsLong(args);
    bool cval = bool(val);

    if (val == -1 && PyErr_Occurred())
    {
        Py_XDECREF(packaged_args);
        PyErr_Clear();
        return PyErr_Format(PyExc_TypeError, "arg not interpretable as C++ bool");
    }
    if (fabs(double(val))>1.5E-7 && fabs((double(long(cval))-double(val))/double(val))>1.5E-7)
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_ValueError, "arg not interpretable as C++ bool");
    }

    Py_XDECREF(packaged_args);

    // Set the showReducedConnectivity in the object.
    obj->data->SetShowReducedConnectivity(cval);

    Py_INCREF(Py_None);
    return Py_None;
}

/*static*/ PyObject *
InverseGhostZoneAttributes_GetShowReducedConnectivity(PyObject *self, PyObject *args)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)self;
    PyObject *retval = PyInt_FromLong(obj->data->GetShowReducedConnectivity()?1L:0L);
    return retval;
}

/*static*/ PyObject *
InverseGhostZoneAttributes_SetShowAMRRefined(PyObject *self, PyObject *args)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)self;

    PyObject *packaged_args = 0;

    // Handle args packaged into a tuple of size one
    // if we think the unpackaged args matches our needs
    if (PySequence_Check(args) && PySequence_Size(args) == 1)
    {
        packaged_args = PySequence_GetItem(args, 0);
        if (PyNumber_Check(packaged_args))
            args = packaged_args;
    }

    if (PySequence_Check(args))
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_TypeError, "expecting a single number arg");
    }

    if (!PyNumber_Check(args))
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_TypeError, "arg is not a number type");
    }

    long val = PyLong_AsLong(args);
    bool cval = bool(val);

    if (val == -1 && PyErr_Occurred())
    {
        Py_XDECREF(packaged_args);
        PyErr_Clear();
        return PyErr_Format(PyExc_TypeError, "arg not interpretable as C++ bool");
    }
    if (fabs(double(val))>1.5E-7 && fabs((double(long(cval))-double(val))/double(val))>1.5E-7)
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_ValueError, "arg not interpretable as C++ bool");
    }

    Py_XDECREF(packaged_args);

    // Set the showAMRRefined in the object.
    obj->data->SetShowAMRRefined(cval);

    Py_INCREF(Py_None);
    return Py_None;
}

/*static*/ PyObject *
InverseGhostZoneAttributes_GetShowAMRRefined(PyObject *self, PyObject *args)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)self;
    PyObject *retval = PyInt_FromLong(obj->data->GetShowAMRRefined()?1L:0L);
    return retval;
}

/*static*/ PyObject *
InverseGhostZoneAttributes_SetShowExterior(PyObject *self, PyObject *args)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)self;

    PyObject *packaged_args = 0;

    // Handle args packaged into a tuple of size one
    // if we think the unpackaged args matches our needs
    if (PySequence_Check(args) && PySequence_Size(args) == 1)
    {
        packaged_args = PySequence_GetItem(args, 0);
        if (PyNumber_Check(packaged_args))
            args = packaged_args;
    }

    if (PySequence_Check(args))
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_TypeError, "expecting a single number arg");
    }

    if (!PyNumber_Check(args))
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_TypeError, "arg is not a number type");
    }

    long val = PyLong_AsLong(args);
    bool cval = bool(val);

    if (val == -1 && PyErr_Occurred())
    {
        Py_XDECREF(packaged_args);
        PyErr_Clear();
        return PyErr_Format(PyExc_TypeError, "arg not interpretable as C++ bool");
    }
    if (fabs(double(val))>1.5E-7 && fabs((double(long(cval))-double(val))/double(val))>1.5E-7)
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_ValueError, "arg not interpretable as C++ bool");
    }

    Py_XDECREF(packaged_args);

    // Set the showExterior in the object.
    obj->data->SetShowExterior(cval);

    Py_INCREF(Py_None);
    return Py_None;
}

/*static*/ PyObject *
InverseGhostZoneAttributes_GetShowExterior(PyObject *self, PyObject *args)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)self;
    PyObject *retval = PyInt_FromLong(obj->data->GetShowExterior()?1L:0L);
    return retval;
}

/*static*/ PyObject *
InverseGhostZoneAttributes_SetShowNotApplicable(PyObject *self, PyObject *args)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)self;

    PyObject *packaged_args = 0;

    // Handle args packaged into a tuple of size one
    // if we think the unpackaged args matches our needs
    if (PySequence_Check(args) && PySequence_Size(args) == 1)
    {
        packaged_args = PySequence_GetItem(args, 0);
        if (PyNumber_Check(packaged_args))
            args = packaged_args;
    }

    if (PySequence_Check(args))
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_TypeError, "expecting a single number arg");
    }

    if (!PyNumber_Check(args))
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_TypeError, "arg is not a number type");
    }

    long val = PyLong_AsLong(args);
    bool cval = bool(val);

    if (val == -1 && PyErr_Occurred())
    {
        Py_XDECREF(packaged_args);
        PyErr_Clear();
        return PyErr_Format(PyExc_TypeError, "arg not interpretable as C++ bool");
    }
    if (fabs(double(val))>1.5E-7 && fabs((double(long(cval))-double(val))/double(val))>1.5E-7)
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_ValueError, "arg not interpretable as C++ bool");
    }

    Py_XDECREF(packaged_args);

    // Set the showNotApplicable in the object.
    obj->data->SetShowNotApplicable(cval);

    Py_INCREF(Py_None);
    return Py_None;
}

/*static*/ PyObject *
InverseGhostZoneAttributes_GetShowNotApplicable(PyObject *self, PyObject *args)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)self;
    PyObject *retval = PyInt_FromLong(obj->data->GetShowNotApplicable()?1L:0L);
    return retval;
}



PyMethodDef PyInverseGhostZoneAttributes_methods[INVERSEGHOSTZONEATTRIBUTES_NMETH] = {
    {"Notify", InverseGhostZoneAttributes_Notify, METH_VARARGS},
    {"SetRequestGhostZones", InverseGhostZoneAttributes_SetRequestGhostZones, METH_VARARGS},
    {"GetRequestGhostZones", InverseGhostZoneAttributes_GetRequestGhostZones, METH_VARARGS},
    {"SetShowDuplicated", InverseGhostZoneAttributes_SetShowDuplicated, METH_VARARGS},
    {"GetShowDuplicated", InverseGhostZoneAttributes_GetShowDuplicated, METH_VARARGS},
    {"SetShowEnhancedConnectivity", InverseGhostZoneAttributes_SetShowEnhancedConnectivity, METH_VARARGS},
    {"GetShowEnhancedConnectivity", InverseGhostZoneAttributes_GetShowEnhancedConnectivity, METH_VARARGS},
    {"SetShowReducedConnectivity", InverseGhostZoneAttributes_SetShowReducedConnectivity, METH_VARARGS},
    {"GetShowReducedConnectivity", InverseGhostZoneAttributes_GetShowReducedConnectivity, METH_VARARGS},
    {"SetShowAMRRefined", InverseGhostZoneAttributes_SetShowAMRRefined, METH_VARARGS},
    {"GetShowAMRRefined", InverseGhostZoneAttributes_GetShowAMRRefined, METH_VARARGS},
    {"SetShowExterior", InverseGhostZoneAttributes_SetShowExterior, METH_VARARGS},
    {"GetShowExterior", InverseGhostZoneAttributes_GetShowExterior, METH_VARARGS},
    {"SetShowNotApplicable", InverseGhostZoneAttributes_SetShowNotApplicable, METH_VARARGS},
    {"GetShowNotApplicable", InverseGhostZoneAttributes_GetShowNotApplicable, METH_VARARGS},
    {NULL, NULL}
};

//
// Type functions
//

static void
InverseGhostZoneAttributes_dealloc(PyObject *v)
{
   InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)v;
   if(obj->parent != 0)
       Py_DECREF(obj->parent);
   if(obj->owns)
       delete obj->data;
}

static PyObject *InverseGhostZoneAttributes_richcompare(PyObject *self, PyObject *other, int op);
PyObject *
PyInverseGhostZoneAttributes_getattr(PyObject *self, char *name)
{
    if(strcmp(name, "requestGhostZones") == 0)
        return InverseGhostZoneAttributes_GetRequestGhostZones(self, NULL);
    if(strcmp(name, "showDuplicated") == 0)
        return InverseGhostZoneAttributes_GetShowDuplicated(self, NULL);
    if(strcmp(name, "showEnhancedConnectivity") == 0)
        return InverseGhostZoneAttributes_GetShowEnhancedConnectivity(self, NULL);
    if(strcmp(name, "showReducedConnectivity") == 0)
        return InverseGhostZoneAttributes_GetShowReducedConnectivity(self, NULL);
    if(strcmp(name, "showAMRRefined") == 0)
        return InverseGhostZoneAttributes_GetShowAMRRefined(self, NULL);
    if(strcmp(name, "showExterior") == 0)
        return InverseGhostZoneAttributes_GetShowExterior(self, NULL);
    if(strcmp(name, "showNotApplicable") == 0)
        return InverseGhostZoneAttributes_GetShowNotApplicable(self, NULL);


    // Add a __dict__ answer so that dir() works
    if (!strcmp(name, "__dict__"))
    {
        PyObject *result = PyDict_New();
        for (int i = 0; PyInverseGhostZoneAttributes_methods[i].ml_meth; i++)
            PyDict_SetItem(result,
                PyString_FromString(PyInverseGhostZoneAttributes_methods[i].ml_name),
                PyString_FromString(PyInverseGhostZoneAttributes_methods[i].ml_name));
        return result;
    }

    return Py_FindMethod(PyInverseGhostZoneAttributes_methods, self, name);
}

int
PyInverseGhostZoneAttributes_setattr(PyObject *self, char *name, PyObject *args)
{
    PyObject NULL_PY_OBJ;
    PyObject *obj = &NULL_PY_OBJ;

    if(strcmp(name, "requestGhostZones") == 0)
        obj = InverseGhostZoneAttributes_SetRequestGhostZones(self, args);
    else if(strcmp(name, "showDuplicated") == 0)
        obj = InverseGhostZoneAttributes_SetShowDuplicated(self, args);
    else if(strcmp(name, "showEnhancedConnectivity") == 0)
        obj = InverseGhostZoneAttributes_SetShowEnhancedConnectivity(self, args);
    else if(strcmp(name, "showReducedConnectivity") == 0)
        obj = InverseGhostZoneAttributes_SetShowReducedConnectivity(self, args);
    else if(strcmp(name, "showAMRRefined") == 0)
        obj = InverseGhostZoneAttributes_SetShowAMRRefined(self, args);
    else if(strcmp(name, "showExterior") == 0)
        obj = InverseGhostZoneAttributes_SetShowExterior(self, args);
    else if(strcmp(name, "showNotApplicable") == 0)
        obj = InverseGhostZoneAttributes_SetShowNotApplicable(self, args);

    if (obj != NULL && obj != &NULL_PY_OBJ)
        Py_DECREF(obj);

    if (obj == &NULL_PY_OBJ)
    {
        obj = NULL;
        PyErr_Format(PyExc_NameError, "name '%s' is not defined", name);
    }
    else if (obj == NULL && !PyErr_Occurred())
        PyErr_Format(PyExc_RuntimeError, "unknown problem with '%s'", name);

    return (obj != NULL) ? 0 : -1;
}

static int
InverseGhostZoneAttributes_print(PyObject *v, FILE *fp, int flags)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)v;
    fprintf(fp, "%s", PyInverseGhostZoneAttributes_ToString(obj->data, "").c_str());
    return 0;
}

PyObject *
InverseGhostZoneAttributes_str(PyObject *v)
{
    InverseGhostZoneAttributesObject *obj = (InverseGhostZoneAttributesObject *)v;
    return PyString_FromString(PyInverseGhostZoneAttributes_ToString(obj->data,"").c_str());
}

//
// The doc string for the class.
//
#if PY_MAJOR_VERSION > 2 || (PY_MAJOR_VERSION == 2 && PY_MINOR_VERSION >= 5)
static const char *InverseGhostZoneAttributes_Purpose = "This class contains attributes for the inverse ghost zone operator.";
#else
static char *InverseGhostZoneAttributes_Purpose = "This class contains attributes for the inverse ghost zone operator.";
#endif

//
// Python Type Struct Def Macro from Py2and3Support.h
//
//         VISIT_PY_TYPE_OBJ( VPY_TYPE,
//                            VPY_NAME,
//                            VPY_OBJECT,
//                            VPY_DEALLOC,
//                            VPY_PRINT,
//                            VPY_GETATTR,
//                            VPY_SETATTR,
//                            VPY_STR,
//                            VPY_PURPOSE,
//                            VPY_RICHCOMP,
//                            VPY_AS_NUMBER)

//
// The type description structure
//

VISIT_PY_TYPE_OBJ(InverseGhostZoneAttributesType,         \
                  "InverseGhostZoneAttributes",           \
                  InverseGhostZoneAttributesObject,       \
                  InverseGhostZoneAttributes_dealloc,     \
                  InverseGhostZoneAttributes_print,       \
                  PyInverseGhostZoneAttributes_getattr,   \
                  PyInverseGhostZoneAttributes_setattr,   \
                  InverseGhostZoneAttributes_str,         \
                  InverseGhostZoneAttributes_Purpose,     \
                  InverseGhostZoneAttributes_richcompare, \
                  0); /* as_number*/

//
// Helper function for comparing.
//
static PyObject *
InverseGhostZoneAttributes_richcompare(PyObject *self, PyObject *other, int op)
{
    // only compare against the same type 
    if ( Py_TYPE(self) != &InverseGhostZoneAttributesType
         || Py_TYPE(other) != &InverseGhostZoneAttributesType)
    {
        Py_INCREF(Py_NotImplemented);
        return Py_NotImplemented;
    }

    PyObject *res = NULL;
    InverseGhostZoneAttributes *a = ((InverseGhostZoneAttributesObject *)self)->data;
    InverseGhostZoneAttributes *b = ((InverseGhostZoneAttributesObject *)other)->data;

    switch (op)
    {
       case Py_EQ:
           res = (*a == *b) ? Py_True : Py_False;
           break;
       case Py_NE:
           res = (*a != *b) ? Py_True : Py_False;
           break;
       default:
           res = Py_NotImplemented;
           break;
    }

    Py_INCREF(res);
    return res;
}

//
// Helper functions for object allocation.
//

static InverseGhostZoneAttributes *defaultAtts = 0;
static InverseGhostZoneAttributes *currentAtts = 0;

static PyObject *
NewInverseGhostZoneAttributes(int useCurrent)
{
    InverseGhostZoneAttributesObject *newObject;
    newObject = PyObject_NEW(InverseGhostZoneAttributesObject, &InverseGhostZoneAttributesType);
    if(newObject == NULL)
        return NULL;
    if(useCurrent && currentAtts != 0)
        newObject->data = new InverseGhostZoneAttributes(*currentAtts);
    else if(defaultAtts != 0)
        newObject->data = new InverseGhostZoneAttributes(*defaultAtts);
    else
        newObject->data = new InverseGhostZoneAttributes;
    newObject->owns = true;
    newObject->parent = 0;
    return (PyObject *)newObject;
}

static PyObject *
WrapInverseGhostZoneAttributes(const InverseGhostZoneAttributes *attr)
{
    InverseGhostZoneAttributesObject *newObject;
    newObject = PyObject_NEW(InverseGhostZoneAttributesObject, &InverseGhostZoneAttributesType);
    if(newObject == NULL)
        return NULL;
    newObject->data = (InverseGhostZoneAttributes *)attr;
    newObject->owns = false;
    newObject->parent = 0;
    return (PyObject *)newObject;
}

///////////////////////////////////////////////////////////////////////////////
//
// Interface that is exposed to the VisIt module.
//
///////////////////////////////////////////////////////////////////////////////

PyObject *
InverseGhostZoneAttributes_new(PyObject *self, PyObject *args)
{
    int useCurrent = 0;
    if (!PyArg_ParseTuple(args, "i", &useCurrent))
    {
        if (!PyArg_ParseTuple(args, ""))
            return NULL;
        else
            PyErr_Clear();
    }

    return (PyObject *)NewInverseGhostZoneAttributes(useCurrent);
}

//
// Plugin method table. These methods are added to the visitmodule's methods.
//
static PyMethodDef InverseGhostZoneAttributesMethods[] = {
    {"InverseGhostZoneAttributes", InverseGhostZoneAttributes_new, METH_VARARGS},
    {NULL,      NULL}        /* Sentinel */
};

static Observer *InverseGhostZoneAttributesObserver = 0;

std::string
PyInverseGhostZoneAttributes_GetLogString()
{
    std::string s("InverseGhostZoneAtts = InverseGhostZoneAttributes()\n");
    if(currentAtts != 0)
        s += PyInverseGhostZoneAttributes_ToString(currentAtts, "InverseGhostZoneAtts.");
    return s;
}

static void
PyInverseGhostZoneAttributes_CallLogRoutine(Subject *subj, void *data)
{
    typedef void (*logCallback)(const std::string &);
    logCallback cb = (logCallback)data;

    if(cb != 0)
    {
        std::string s("InverseGhostZoneAtts = InverseGhostZoneAttributes()\n");
        s += PyInverseGhostZoneAttributes_ToString(currentAtts, "InverseGhostZoneAtts.");
        cb(s);
    }
}

void
PyInverseGhostZoneAttributes_StartUp(InverseGhostZoneAttributes *subj, void *data)
{
    if(subj == 0)
        return;

    currentAtts = subj;
    PyInverseGhostZoneAttributes_SetDefaults(subj);

    //
    // Create the observer that will be notified when the attributes change.
    //
    if(InverseGhostZoneAttributesObserver == 0)
    {
        InverseGhostZoneAttributesObserver = new ObserverToCallback(subj,
            PyInverseGhostZoneAttributes_CallLogRoutine, (void *)data);
    }

}

void
PyInverseGhostZoneAttributes_CloseDown()
{
    delete defaultAtts;
    defaultAtts = 0;
    delete InverseGhostZoneAttributesObserver;
    InverseGhostZoneAttributesObserver = 0;
}

PyMethodDef *
PyInverseGhostZoneAttributes_GetMethodTable(int *nMethods)
{
    *nMethods = 1;
    return InverseGhostZoneAttributesMethods;
}

bool
PyInverseGhostZoneAttributes_Check(PyObject *obj)
{
    return (obj->ob_type == &InverseGhostZoneAttributesType);
}

InverseGhostZoneAttributes *
PyInverseGhostZoneAttributes_FromPyObject(PyObject *obj)
{
    InverseGhostZoneAttributesObject *obj2 = (InverseGhostZoneAttributesObject *)obj;
    return obj2->data;
}

PyObject *
PyInverseGhostZoneAttributes_New()
{
    return NewInverseGhostZoneAttributes(0);
}

PyObject *
PyInverseGhostZoneAttributes_Wrap(const InverseGhostZoneAttributes *attr)
{
    return WrapInverseGhostZoneAttributes(attr);
}

void
PyInverseGhostZoneAttributes_SetParent(PyObject *obj, PyObject *parent)
{
    InverseGhostZoneAttributesObject *obj2 = (InverseGhostZoneAttributesObject *)obj;
    obj2->parent = parent;
}

void
PyInverseGhostZoneAttributes_SetDefaults(const InverseGhostZoneAttributes *atts)
{
    if(defaultAtts)
        delete defaultAtts;

    defaultAtts = new InverseGhostZoneAttributes(*atts);
}

