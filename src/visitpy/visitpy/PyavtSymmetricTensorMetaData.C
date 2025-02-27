// Copyright (c) Lawrence Livermore National Security, LLC and other VisIt
// Project developers.  See the top-level LICENSE file for dates and other
// details.  No copyright assignment is required to contribute to VisIt.

#include <PyavtSymmetricTensorMetaData.h>
#include <ObserverToCallback.h>
#include <stdio.h>
#include <Py2and3Support.h>

// ****************************************************************************
// Module: PyavtSymmetricTensorMetaData
//
// Purpose:
//   Contains symmetricTensor metadata attributes
//
// Note:       Autogenerated by xml2python. Do not modify by hand!
//
// Programmer: xml2python
// Creation:   omitted
//
// ****************************************************************************

//
// This struct contains the Python type information and a avtSymmetricTensorMetaData.
//
struct avtSymmetricTensorMetaDataObject
{
    PyObject_HEAD
    avtSymmetricTensorMetaData *data;
    bool        owns;
    PyObject   *parent;
};

//
// Internal prototypes
//
static PyObject *NewavtSymmetricTensorMetaData(int);
std::string
PyavtSymmetricTensorMetaData_ToString(const avtSymmetricTensorMetaData *atts, const char *prefix)
{
    std::string str;
    char tmpStr[1000];

    str = PyavtVarMetaData_ToString(atts, prefix);

    snprintf(tmpStr, 1000, "%sdim = %d\n", prefix, atts->dim);
    str += tmpStr;
    return str;
}

static PyObject *
avtSymmetricTensorMetaData_Notify(PyObject *self, PyObject *args)
{
    avtSymmetricTensorMetaDataObject *obj = (avtSymmetricTensorMetaDataObject *)self;
    obj->data->Notify();
    Py_INCREF(Py_None);
    return Py_None;
}

/*static*/ PyObject *
avtSymmetricTensorMetaData_SetDim(PyObject *self, PyObject *args)
{
    avtSymmetricTensorMetaDataObject *obj = (avtSymmetricTensorMetaDataObject *)self;

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
    int cval = int(val);

    if (val == -1 && PyErr_Occurred())
    {
        Py_XDECREF(packaged_args);
        PyErr_Clear();
        return PyErr_Format(PyExc_TypeError, "arg not interpretable as C++ int");
    }
    if (fabs(double(val))>1.5E-7 && fabs((double(long(cval))-double(val))/double(val))>1.5E-7)
    {
        Py_XDECREF(packaged_args);
        return PyErr_Format(PyExc_ValueError, "arg not interpretable as C++ int");
    }

    Py_XDECREF(packaged_args);

    // Set the dim in the object.
    obj->data->dim = cval;

    Py_INCREF(Py_None);
    return Py_None;
}

/*static*/ PyObject *
avtSymmetricTensorMetaData_GetDim(PyObject *self, PyObject *args)
{
    avtSymmetricTensorMetaDataObject *obj = (avtSymmetricTensorMetaDataObject *)self;
    PyObject *retval = PyInt_FromLong(long(obj->data->dim));
    return retval;
}



PyMethodDef PyavtSymmetricTensorMetaData_methods[AVTSYMMETRICTENSORMETADATA_NMETH] = {
    {"Notify", avtSymmetricTensorMetaData_Notify, METH_VARARGS},
    {"SetDim", avtSymmetricTensorMetaData_SetDim, METH_VARARGS},
    {"GetDim", avtSymmetricTensorMetaData_GetDim, METH_VARARGS},
    {NULL, NULL}
};

static void PyavtSymmetricTensorMetaData_ExtendSetGetMethodTable()
{
    static bool extended = false;
    if (extended) return;
    extended = true;

    int i = 0;
    while (PyavtSymmetricTensorMetaData_methods[i].ml_name)
        i++;
    int n = i;
    while (PyavtVarMetaData_methods[i-n+1].ml_name)
    {
        PyavtSymmetricTensorMetaData_methods[i] = PyavtVarMetaData_methods[i-n+1];
        i++;
    }

    PyMethodDef nullMethod = {NULL, NULL};
    PyavtSymmetricTensorMetaData_methods[i] = nullMethod;
}

//
// Type functions
//

static void
avtSymmetricTensorMetaData_dealloc(PyObject *v)
{
   avtSymmetricTensorMetaDataObject *obj = (avtSymmetricTensorMetaDataObject *)v;
   if(obj->parent != 0)
       Py_DECREF(obj->parent);
   if(obj->owns)
       delete obj->data;
}

static PyObject *avtSymmetricTensorMetaData_richcompare(PyObject *self, PyObject *other, int op);
PyObject *
PyavtSymmetricTensorMetaData_getattr(PyObject *self, char *name)
{
    if(strcmp(name, "dim") == 0)
        return avtSymmetricTensorMetaData_GetDim(self, NULL);

    if(strcmp(name, "__methods__") != 0)
    {
        PyObject *retval = PyavtVarMetaData_getattr(self, name);
        if (retval) return retval;
    }

    PyavtSymmetricTensorMetaData_ExtendSetGetMethodTable();

    // Add a __dict__ answer so that dir() works
    if (!strcmp(name, "__dict__"))
    {
        PyObject *result = PyDict_New();
        for (int i = 0; PyavtSymmetricTensorMetaData_methods[i].ml_meth; i++)
            PyDict_SetItem(result,
                PyString_FromString(PyavtSymmetricTensorMetaData_methods[i].ml_name),
                PyString_FromString(PyavtSymmetricTensorMetaData_methods[i].ml_name));
        return result;
    }

    return Py_FindMethod(PyavtSymmetricTensorMetaData_methods, self, name);
}

int
PyavtSymmetricTensorMetaData_setattr(PyObject *self, char *name, PyObject *args)
{
    if (PyavtVarMetaData_setattr(self, name, args) != -1)
        return 0;
    else
        PyErr_Clear();

    PyObject NULL_PY_OBJ;
    PyObject *obj = &NULL_PY_OBJ;

    if(strcmp(name, "dim") == 0)
        obj = avtSymmetricTensorMetaData_SetDim(self, args);

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
avtSymmetricTensorMetaData_print(PyObject *v, FILE *fp, int flags)
{
    avtSymmetricTensorMetaDataObject *obj = (avtSymmetricTensorMetaDataObject *)v;
    fprintf(fp, "%s", PyavtSymmetricTensorMetaData_ToString(obj->data, "").c_str());
    return 0;
}

PyObject *
avtSymmetricTensorMetaData_str(PyObject *v)
{
    avtSymmetricTensorMetaDataObject *obj = (avtSymmetricTensorMetaDataObject *)v;
    return PyString_FromString(PyavtSymmetricTensorMetaData_ToString(obj->data,"").c_str());
}

//
// The doc string for the class.
//
#if PY_MAJOR_VERSION > 2 || (PY_MAJOR_VERSION == 2 && PY_MINOR_VERSION >= 5)
static const char *avtSymmetricTensorMetaData_Purpose = "Contains symmetricTensor metadata attributes";
#else
static char *avtSymmetricTensorMetaData_Purpose = "Contains symmetricTensor metadata attributes";
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

VISIT_PY_TYPE_OBJ(avtSymmetricTensorMetaDataType,         \
                  "avtSymmetricTensorMetaData",           \
                  avtSymmetricTensorMetaDataObject,       \
                  avtSymmetricTensorMetaData_dealloc,     \
                  avtSymmetricTensorMetaData_print,       \
                  PyavtSymmetricTensorMetaData_getattr,   \
                  PyavtSymmetricTensorMetaData_setattr,   \
                  avtSymmetricTensorMetaData_str,         \
                  avtSymmetricTensorMetaData_Purpose,     \
                  avtSymmetricTensorMetaData_richcompare, \
                  0); /* as_number*/

//
// Helper function for comparing.
//
static PyObject *
avtSymmetricTensorMetaData_richcompare(PyObject *self, PyObject *other, int op)
{
    // only compare against the same type 
    if ( Py_TYPE(self) != &avtSymmetricTensorMetaDataType
         || Py_TYPE(other) != &avtSymmetricTensorMetaDataType)
    {
        Py_INCREF(Py_NotImplemented);
        return Py_NotImplemented;
    }

    PyObject *res = NULL;
    avtSymmetricTensorMetaData *a = ((avtSymmetricTensorMetaDataObject *)self)->data;
    avtSymmetricTensorMetaData *b = ((avtSymmetricTensorMetaDataObject *)other)->data;

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

static avtSymmetricTensorMetaData *defaultAtts = 0;
static avtSymmetricTensorMetaData *currentAtts = 0;

static PyObject *
NewavtSymmetricTensorMetaData(int useCurrent)
{
    avtSymmetricTensorMetaDataObject *newObject;
    newObject = PyObject_NEW(avtSymmetricTensorMetaDataObject, &avtSymmetricTensorMetaDataType);
    if(newObject == NULL)
        return NULL;
    if(useCurrent && currentAtts != 0)
        newObject->data = new avtSymmetricTensorMetaData(*currentAtts);
    else if(defaultAtts != 0)
        newObject->data = new avtSymmetricTensorMetaData(*defaultAtts);
    else
        newObject->data = new avtSymmetricTensorMetaData;
    newObject->owns = true;
    newObject->parent = 0;
    return (PyObject *)newObject;
}

static PyObject *
WrapavtSymmetricTensorMetaData(const avtSymmetricTensorMetaData *attr)
{
    avtSymmetricTensorMetaDataObject *newObject;
    newObject = PyObject_NEW(avtSymmetricTensorMetaDataObject, &avtSymmetricTensorMetaDataType);
    if(newObject == NULL)
        return NULL;
    newObject->data = (avtSymmetricTensorMetaData *)attr;
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
avtSymmetricTensorMetaData_new(PyObject *self, PyObject *args)
{
    int useCurrent = 0;
    if (!PyArg_ParseTuple(args, "i", &useCurrent))
    {
        if (!PyArg_ParseTuple(args, ""))
            return NULL;
        else
            PyErr_Clear();
    }

    return (PyObject *)NewavtSymmetricTensorMetaData(useCurrent);
}

//
// Plugin method table. These methods are added to the visitmodule's methods.
//
static PyMethodDef avtSymmetricTensorMetaDataMethods[] = {
    {"avtSymmetricTensorMetaData", avtSymmetricTensorMetaData_new, METH_VARARGS},
    {NULL,      NULL}        /* Sentinel */
};

static Observer *avtSymmetricTensorMetaDataObserver = 0;

std::string
PyavtSymmetricTensorMetaData_GetLogString()
{
    std::string s("avtSymmetricTensorMetaData = avtSymmetricTensorMetaData()\n");
    if(currentAtts != 0)
        s += PyavtSymmetricTensorMetaData_ToString(currentAtts, "avtSymmetricTensorMetaData.");
    return s;
}

static void
PyavtSymmetricTensorMetaData_CallLogRoutine(Subject *subj, void *data)
{
    typedef void (*logCallback)(const std::string &);
    logCallback cb = (logCallback)data;

    if(cb != 0)
    {
        std::string s("avtSymmetricTensorMetaData = avtSymmetricTensorMetaData()\n");
        s += PyavtSymmetricTensorMetaData_ToString(currentAtts, "avtSymmetricTensorMetaData.");
        cb(s);
    }
}

void
PyavtSymmetricTensorMetaData_StartUp(avtSymmetricTensorMetaData *subj, void *data)
{
    if(subj == 0)
        return;

    currentAtts = subj;
    PyavtSymmetricTensorMetaData_SetDefaults(subj);

    //
    // Create the observer that will be notified when the attributes change.
    //
    if(avtSymmetricTensorMetaDataObserver == 0)
    {
        avtSymmetricTensorMetaDataObserver = new ObserverToCallback(subj,
            PyavtSymmetricTensorMetaData_CallLogRoutine, (void *)data);
    }

}

void
PyavtSymmetricTensorMetaData_CloseDown()
{
    delete defaultAtts;
    defaultAtts = 0;
    delete avtSymmetricTensorMetaDataObserver;
    avtSymmetricTensorMetaDataObserver = 0;
}

PyMethodDef *
PyavtSymmetricTensorMetaData_GetMethodTable(int *nMethods)
{
    *nMethods = 1;
    return avtSymmetricTensorMetaDataMethods;
}

bool
PyavtSymmetricTensorMetaData_Check(PyObject *obj)
{
    return (obj->ob_type == &avtSymmetricTensorMetaDataType);
}

avtSymmetricTensorMetaData *
PyavtSymmetricTensorMetaData_FromPyObject(PyObject *obj)
{
    avtSymmetricTensorMetaDataObject *obj2 = (avtSymmetricTensorMetaDataObject *)obj;
    return obj2->data;
}

PyObject *
PyavtSymmetricTensorMetaData_New()
{
    return NewavtSymmetricTensorMetaData(0);
}

PyObject *
PyavtSymmetricTensorMetaData_Wrap(const avtSymmetricTensorMetaData *attr)
{
    return WrapavtSymmetricTensorMetaData(attr);
}

void
PyavtSymmetricTensorMetaData_SetParent(PyObject *obj, PyObject *parent)
{
    avtSymmetricTensorMetaDataObject *obj2 = (avtSymmetricTensorMetaDataObject *)obj;
    obj2->parent = parent;
}

void
PyavtSymmetricTensorMetaData_SetDefaults(const avtSymmetricTensorMetaData *atts)
{
    if(defaultAtts)
        delete defaultAtts;

    defaultAtts = new avtSymmetricTensorMetaData(*atts);
}

