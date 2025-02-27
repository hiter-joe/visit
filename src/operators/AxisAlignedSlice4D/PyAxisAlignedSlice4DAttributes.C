// Copyright (c) Lawrence Livermore National Security, LLC and other VisIt
// Project developers.  See the top-level LICENSE file for dates and other
// details.  No copyright assignment is required to contribute to VisIt.

#include <PyAxisAlignedSlice4DAttributes.h>
#include <ObserverToCallback.h>
#include <stdio.h>
#include <Py2and3Support.h>

// ****************************************************************************
// Module: PyAxisAlignedSlice4DAttributes
//
// Purpose:
//   Attributes for AxisAlignedSlice4D
//
// Note:       Autogenerated by xml2python. Do not modify by hand!
//
// Programmer: xml2python
// Creation:   omitted
//
// ****************************************************************************

//
// This struct contains the Python type information and a AxisAlignedSlice4DAttributes.
//
struct AxisAlignedSlice4DAttributesObject
{
    PyObject_HEAD
    AxisAlignedSlice4DAttributes *data;
    bool        owns;
    PyObject   *parent;
};

//
// Internal prototypes
//
static PyObject *NewAxisAlignedSlice4DAttributes(int);
std::string
PyAxisAlignedSlice4DAttributes_ToString(const AxisAlignedSlice4DAttributes *atts, const char *prefix)
{
    std::string str;
    char tmpStr[1000];

    {   const intVector &I = atts->GetI();
        snprintf(tmpStr, 1000, "%sI = (", prefix);
        str += tmpStr;
        for(size_t i = 0; i < I.size(); ++i)
        {
            snprintf(tmpStr, 1000, "%d", I[i]);
            str += tmpStr;
            if(i < I.size() - 1)
            {
                snprintf(tmpStr, 1000, ", ");
                str += tmpStr;
            }
        }
        snprintf(tmpStr, 1000, ")\n");
        str += tmpStr;
    }
    {   const intVector &J = atts->GetJ();
        snprintf(tmpStr, 1000, "%sJ = (", prefix);
        str += tmpStr;
        for(size_t i = 0; i < J.size(); ++i)
        {
            snprintf(tmpStr, 1000, "%d", J[i]);
            str += tmpStr;
            if(i < J.size() - 1)
            {
                snprintf(tmpStr, 1000, ", ");
                str += tmpStr;
            }
        }
        snprintf(tmpStr, 1000, ")\n");
        str += tmpStr;
    }
    {   const intVector &K = atts->GetK();
        snprintf(tmpStr, 1000, "%sK = (", prefix);
        str += tmpStr;
        for(size_t i = 0; i < K.size(); ++i)
        {
            snprintf(tmpStr, 1000, "%d", K[i]);
            str += tmpStr;
            if(i < K.size() - 1)
            {
                snprintf(tmpStr, 1000, ", ");
                str += tmpStr;
            }
        }
        snprintf(tmpStr, 1000, ")\n");
        str += tmpStr;
    }
    {   const intVector &L = atts->GetL();
        snprintf(tmpStr, 1000, "%sL = (", prefix);
        str += tmpStr;
        for(size_t i = 0; i < L.size(); ++i)
        {
            snprintf(tmpStr, 1000, "%d", L[i]);
            str += tmpStr;
            if(i < L.size() - 1)
            {
                snprintf(tmpStr, 1000, ", ");
                str += tmpStr;
            }
        }
        snprintf(tmpStr, 1000, ")\n");
        str += tmpStr;
    }
    return str;
}

static PyObject *
AxisAlignedSlice4DAttributes_Notify(PyObject *self, PyObject *args)
{
    AxisAlignedSlice4DAttributesObject *obj = (AxisAlignedSlice4DAttributesObject *)self;
    obj->data->Notify();
    Py_INCREF(Py_None);
    return Py_None;
}

/*static*/ PyObject *
AxisAlignedSlice4DAttributes_SetI(PyObject *self, PyObject *args)
{
    AxisAlignedSlice4DAttributesObject *obj = (AxisAlignedSlice4DAttributesObject *)self;

    intVector vec;

    if (PyNumber_Check(args))
    {
        long val = PyLong_AsLong(args);
        int cval = int(val);
        if (val == -1 && PyErr_Occurred())
        {
            PyErr_Clear();
            return PyErr_Format(PyExc_TypeError, "number not interpretable as C++ int");
        }
        if (fabs(double(val))>1.5E-7 && fabs((double(long(cval))-double(val))/double(val))>1.5E-7)
            return PyErr_Format(PyExc_ValueError, "number not interpretable as C++ int");
        vec.resize(1);
        vec[0] = cval;
    }
    else if (PySequence_Check(args) && !PyUnicode_Check(args))
    {
        vec.resize(PySequence_Size(args));
        for (Py_ssize_t i = 0; i < PySequence_Size(args); i++)
        {
            PyObject *item = PySequence_GetItem(args, i);

            if (!PyNumber_Check(item))
            {
                Py_DECREF(item);
                return PyErr_Format(PyExc_TypeError, "arg %d is not a number type", (int) i);
            }

            long val = PyLong_AsLong(item);
            int cval = int(val);

            if (val == -1 && PyErr_Occurred())
            {
                Py_DECREF(item);
                PyErr_Clear();
                return PyErr_Format(PyExc_TypeError, "arg %d not interpretable as C++ int", (int) i);
            }
            if (fabs(double(val))>1.5E-7 && fabs((double(long(cval))-double(val))/double(val))>1.5E-7)
            {
                Py_DECREF(item);
                return PyErr_Format(PyExc_ValueError, "arg %d not interpretable as C++ int", (int) i);
            }
            Py_DECREF(item);

            vec[i] = cval;
        }
    }
    else
        return PyErr_Format(PyExc_TypeError, "arg(s) must be one or more ints");

    obj->data->GetI() = vec;
    // Mark the I in the object as modified.
    obj->data->SelectI();

    Py_INCREF(Py_None);
    return Py_None;
}

/*static*/ PyObject *
AxisAlignedSlice4DAttributes_GetI(PyObject *self, PyObject *args)
{
    AxisAlignedSlice4DAttributesObject *obj = (AxisAlignedSlice4DAttributesObject *)self;
    // Allocate a tuple the with enough entries to hold the I.
    const intVector &I = obj->data->GetI();
    PyObject *retval = PyTuple_New(I.size());
    for(size_t i = 0; i < I.size(); ++i)
        PyTuple_SET_ITEM(retval, i, PyInt_FromLong(long(I[i])));
    return retval;
}

/*static*/ PyObject *
AxisAlignedSlice4DAttributes_SetJ(PyObject *self, PyObject *args)
{
    AxisAlignedSlice4DAttributesObject *obj = (AxisAlignedSlice4DAttributesObject *)self;

    intVector vec;

    if (PyNumber_Check(args))
    {
        long val = PyLong_AsLong(args);
        int cval = int(val);
        if (val == -1 && PyErr_Occurred())
        {
            PyErr_Clear();
            return PyErr_Format(PyExc_TypeError, "number not interpretable as C++ int");
        }
        if (fabs(double(val))>1.5E-7 && fabs((double(long(cval))-double(val))/double(val))>1.5E-7)
            return PyErr_Format(PyExc_ValueError, "number not interpretable as C++ int");
        vec.resize(1);
        vec[0] = cval;
    }
    else if (PySequence_Check(args) && !PyUnicode_Check(args))
    {
        vec.resize(PySequence_Size(args));
        for (Py_ssize_t i = 0; i < PySequence_Size(args); i++)
        {
            PyObject *item = PySequence_GetItem(args, i);

            if (!PyNumber_Check(item))
            {
                Py_DECREF(item);
                return PyErr_Format(PyExc_TypeError, "arg %d is not a number type", (int) i);
            }

            long val = PyLong_AsLong(item);
            int cval = int(val);

            if (val == -1 && PyErr_Occurred())
            {
                Py_DECREF(item);
                PyErr_Clear();
                return PyErr_Format(PyExc_TypeError, "arg %d not interpretable as C++ int", (int) i);
            }
            if (fabs(double(val))>1.5E-7 && fabs((double(long(cval))-double(val))/double(val))>1.5E-7)
            {
                Py_DECREF(item);
                return PyErr_Format(PyExc_ValueError, "arg %d not interpretable as C++ int", (int) i);
            }
            Py_DECREF(item);

            vec[i] = cval;
        }
    }
    else
        return PyErr_Format(PyExc_TypeError, "arg(s) must be one or more ints");

    obj->data->GetJ() = vec;
    // Mark the J in the object as modified.
    obj->data->SelectJ();

    Py_INCREF(Py_None);
    return Py_None;
}

/*static*/ PyObject *
AxisAlignedSlice4DAttributes_GetJ(PyObject *self, PyObject *args)
{
    AxisAlignedSlice4DAttributesObject *obj = (AxisAlignedSlice4DAttributesObject *)self;
    // Allocate a tuple the with enough entries to hold the J.
    const intVector &J = obj->data->GetJ();
    PyObject *retval = PyTuple_New(J.size());
    for(size_t i = 0; i < J.size(); ++i)
        PyTuple_SET_ITEM(retval, i, PyInt_FromLong(long(J[i])));
    return retval;
}

/*static*/ PyObject *
AxisAlignedSlice4DAttributes_SetK(PyObject *self, PyObject *args)
{
    AxisAlignedSlice4DAttributesObject *obj = (AxisAlignedSlice4DAttributesObject *)self;

    intVector vec;

    if (PyNumber_Check(args))
    {
        long val = PyLong_AsLong(args);
        int cval = int(val);
        if (val == -1 && PyErr_Occurred())
        {
            PyErr_Clear();
            return PyErr_Format(PyExc_TypeError, "number not interpretable as C++ int");
        }
        if (fabs(double(val))>1.5E-7 && fabs((double(long(cval))-double(val))/double(val))>1.5E-7)
            return PyErr_Format(PyExc_ValueError, "number not interpretable as C++ int");
        vec.resize(1);
        vec[0] = cval;
    }
    else if (PySequence_Check(args) && !PyUnicode_Check(args))
    {
        vec.resize(PySequence_Size(args));
        for (Py_ssize_t i = 0; i < PySequence_Size(args); i++)
        {
            PyObject *item = PySequence_GetItem(args, i);

            if (!PyNumber_Check(item))
            {
                Py_DECREF(item);
                return PyErr_Format(PyExc_TypeError, "arg %d is not a number type", (int) i);
            }

            long val = PyLong_AsLong(item);
            int cval = int(val);

            if (val == -1 && PyErr_Occurred())
            {
                Py_DECREF(item);
                PyErr_Clear();
                return PyErr_Format(PyExc_TypeError, "arg %d not interpretable as C++ int", (int) i);
            }
            if (fabs(double(val))>1.5E-7 && fabs((double(long(cval))-double(val))/double(val))>1.5E-7)
            {
                Py_DECREF(item);
                return PyErr_Format(PyExc_ValueError, "arg %d not interpretable as C++ int", (int) i);
            }
            Py_DECREF(item);

            vec[i] = cval;
        }
    }
    else
        return PyErr_Format(PyExc_TypeError, "arg(s) must be one or more ints");

    obj->data->GetK() = vec;
    // Mark the K in the object as modified.
    obj->data->SelectK();

    Py_INCREF(Py_None);
    return Py_None;
}

/*static*/ PyObject *
AxisAlignedSlice4DAttributes_GetK(PyObject *self, PyObject *args)
{
    AxisAlignedSlice4DAttributesObject *obj = (AxisAlignedSlice4DAttributesObject *)self;
    // Allocate a tuple the with enough entries to hold the K.
    const intVector &K = obj->data->GetK();
    PyObject *retval = PyTuple_New(K.size());
    for(size_t i = 0; i < K.size(); ++i)
        PyTuple_SET_ITEM(retval, i, PyInt_FromLong(long(K[i])));
    return retval;
}

/*static*/ PyObject *
AxisAlignedSlice4DAttributes_SetL(PyObject *self, PyObject *args)
{
    AxisAlignedSlice4DAttributesObject *obj = (AxisAlignedSlice4DAttributesObject *)self;

    intVector vec;

    if (PyNumber_Check(args))
    {
        long val = PyLong_AsLong(args);
        int cval = int(val);
        if (val == -1 && PyErr_Occurred())
        {
            PyErr_Clear();
            return PyErr_Format(PyExc_TypeError, "number not interpretable as C++ int");
        }
        if (fabs(double(val))>1.5E-7 && fabs((double(long(cval))-double(val))/double(val))>1.5E-7)
            return PyErr_Format(PyExc_ValueError, "number not interpretable as C++ int");
        vec.resize(1);
        vec[0] = cval;
    }
    else if (PySequence_Check(args) && !PyUnicode_Check(args))
    {
        vec.resize(PySequence_Size(args));
        for (Py_ssize_t i = 0; i < PySequence_Size(args); i++)
        {
            PyObject *item = PySequence_GetItem(args, i);

            if (!PyNumber_Check(item))
            {
                Py_DECREF(item);
                return PyErr_Format(PyExc_TypeError, "arg %d is not a number type", (int) i);
            }

            long val = PyLong_AsLong(item);
            int cval = int(val);

            if (val == -1 && PyErr_Occurred())
            {
                Py_DECREF(item);
                PyErr_Clear();
                return PyErr_Format(PyExc_TypeError, "arg %d not interpretable as C++ int", (int) i);
            }
            if (fabs(double(val))>1.5E-7 && fabs((double(long(cval))-double(val))/double(val))>1.5E-7)
            {
                Py_DECREF(item);
                return PyErr_Format(PyExc_ValueError, "arg %d not interpretable as C++ int", (int) i);
            }
            Py_DECREF(item);

            vec[i] = cval;
        }
    }
    else
        return PyErr_Format(PyExc_TypeError, "arg(s) must be one or more ints");

    obj->data->GetL() = vec;
    // Mark the L in the object as modified.
    obj->data->SelectL();

    Py_INCREF(Py_None);
    return Py_None;
}

/*static*/ PyObject *
AxisAlignedSlice4DAttributes_GetL(PyObject *self, PyObject *args)
{
    AxisAlignedSlice4DAttributesObject *obj = (AxisAlignedSlice4DAttributesObject *)self;
    // Allocate a tuple the with enough entries to hold the L.
    const intVector &L = obj->data->GetL();
    PyObject *retval = PyTuple_New(L.size());
    for(size_t i = 0; i < L.size(); ++i)
        PyTuple_SET_ITEM(retval, i, PyInt_FromLong(long(L[i])));
    return retval;
}



PyMethodDef PyAxisAlignedSlice4DAttributes_methods[AXISALIGNEDSLICE4DATTRIBUTES_NMETH] = {
    {"Notify", AxisAlignedSlice4DAttributes_Notify, METH_VARARGS},
    {"SetI", AxisAlignedSlice4DAttributes_SetI, METH_VARARGS},
    {"GetI", AxisAlignedSlice4DAttributes_GetI, METH_VARARGS},
    {"SetJ", AxisAlignedSlice4DAttributes_SetJ, METH_VARARGS},
    {"GetJ", AxisAlignedSlice4DAttributes_GetJ, METH_VARARGS},
    {"SetK", AxisAlignedSlice4DAttributes_SetK, METH_VARARGS},
    {"GetK", AxisAlignedSlice4DAttributes_GetK, METH_VARARGS},
    {"SetL", AxisAlignedSlice4DAttributes_SetL, METH_VARARGS},
    {"GetL", AxisAlignedSlice4DAttributes_GetL, METH_VARARGS},
    {NULL, NULL}
};

//
// Type functions
//

static void
AxisAlignedSlice4DAttributes_dealloc(PyObject *v)
{
   AxisAlignedSlice4DAttributesObject *obj = (AxisAlignedSlice4DAttributesObject *)v;
   if(obj->parent != 0)
       Py_DECREF(obj->parent);
   if(obj->owns)
       delete obj->data;
}

static PyObject *AxisAlignedSlice4DAttributes_richcompare(PyObject *self, PyObject *other, int op);
PyObject *
PyAxisAlignedSlice4DAttributes_getattr(PyObject *self, char *name)
{
    if(strcmp(name, "I") == 0)
        return AxisAlignedSlice4DAttributes_GetI(self, NULL);
    if(strcmp(name, "J") == 0)
        return AxisAlignedSlice4DAttributes_GetJ(self, NULL);
    if(strcmp(name, "K") == 0)
        return AxisAlignedSlice4DAttributes_GetK(self, NULL);
    if(strcmp(name, "L") == 0)
        return AxisAlignedSlice4DAttributes_GetL(self, NULL);


    // Add a __dict__ answer so that dir() works
    if (!strcmp(name, "__dict__"))
    {
        PyObject *result = PyDict_New();
        for (int i = 0; PyAxisAlignedSlice4DAttributes_methods[i].ml_meth; i++)
            PyDict_SetItem(result,
                PyString_FromString(PyAxisAlignedSlice4DAttributes_methods[i].ml_name),
                PyString_FromString(PyAxisAlignedSlice4DAttributes_methods[i].ml_name));
        return result;
    }

    return Py_FindMethod(PyAxisAlignedSlice4DAttributes_methods, self, name);
}

int
PyAxisAlignedSlice4DAttributes_setattr(PyObject *self, char *name, PyObject *args)
{
    PyObject NULL_PY_OBJ;
    PyObject *obj = &NULL_PY_OBJ;

    if(strcmp(name, "I") == 0)
        obj = AxisAlignedSlice4DAttributes_SetI(self, args);
    else if(strcmp(name, "J") == 0)
        obj = AxisAlignedSlice4DAttributes_SetJ(self, args);
    else if(strcmp(name, "K") == 0)
        obj = AxisAlignedSlice4DAttributes_SetK(self, args);
    else if(strcmp(name, "L") == 0)
        obj = AxisAlignedSlice4DAttributes_SetL(self, args);

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
AxisAlignedSlice4DAttributes_print(PyObject *v, FILE *fp, int flags)
{
    AxisAlignedSlice4DAttributesObject *obj = (AxisAlignedSlice4DAttributesObject *)v;
    fprintf(fp, "%s", PyAxisAlignedSlice4DAttributes_ToString(obj->data, "").c_str());
    return 0;
}

PyObject *
AxisAlignedSlice4DAttributes_str(PyObject *v)
{
    AxisAlignedSlice4DAttributesObject *obj = (AxisAlignedSlice4DAttributesObject *)v;
    return PyString_FromString(PyAxisAlignedSlice4DAttributes_ToString(obj->data,"").c_str());
}

//
// The doc string for the class.
//
#if PY_MAJOR_VERSION > 2 || (PY_MAJOR_VERSION == 2 && PY_MINOR_VERSION >= 5)
static const char *AxisAlignedSlice4DAttributes_Purpose = "Attributes for AxisAlignedSlice4D";
#else
static char *AxisAlignedSlice4DAttributes_Purpose = "Attributes for AxisAlignedSlice4D";
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

VISIT_PY_TYPE_OBJ(AxisAlignedSlice4DAttributesType,         \
                  "AxisAlignedSlice4DAttributes",           \
                  AxisAlignedSlice4DAttributesObject,       \
                  AxisAlignedSlice4DAttributes_dealloc,     \
                  AxisAlignedSlice4DAttributes_print,       \
                  PyAxisAlignedSlice4DAttributes_getattr,   \
                  PyAxisAlignedSlice4DAttributes_setattr,   \
                  AxisAlignedSlice4DAttributes_str,         \
                  AxisAlignedSlice4DAttributes_Purpose,     \
                  AxisAlignedSlice4DAttributes_richcompare, \
                  0); /* as_number*/

//
// Helper function for comparing.
//
static PyObject *
AxisAlignedSlice4DAttributes_richcompare(PyObject *self, PyObject *other, int op)
{
    // only compare against the same type 
    if ( Py_TYPE(self) != &AxisAlignedSlice4DAttributesType
         || Py_TYPE(other) != &AxisAlignedSlice4DAttributesType)
    {
        Py_INCREF(Py_NotImplemented);
        return Py_NotImplemented;
    }

    PyObject *res = NULL;
    AxisAlignedSlice4DAttributes *a = ((AxisAlignedSlice4DAttributesObject *)self)->data;
    AxisAlignedSlice4DAttributes *b = ((AxisAlignedSlice4DAttributesObject *)other)->data;

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

static AxisAlignedSlice4DAttributes *defaultAtts = 0;
static AxisAlignedSlice4DAttributes *currentAtts = 0;

static PyObject *
NewAxisAlignedSlice4DAttributes(int useCurrent)
{
    AxisAlignedSlice4DAttributesObject *newObject;
    newObject = PyObject_NEW(AxisAlignedSlice4DAttributesObject, &AxisAlignedSlice4DAttributesType);
    if(newObject == NULL)
        return NULL;
    if(useCurrent && currentAtts != 0)
        newObject->data = new AxisAlignedSlice4DAttributes(*currentAtts);
    else if(defaultAtts != 0)
        newObject->data = new AxisAlignedSlice4DAttributes(*defaultAtts);
    else
        newObject->data = new AxisAlignedSlice4DAttributes;
    newObject->owns = true;
    newObject->parent = 0;
    return (PyObject *)newObject;
}

static PyObject *
WrapAxisAlignedSlice4DAttributes(const AxisAlignedSlice4DAttributes *attr)
{
    AxisAlignedSlice4DAttributesObject *newObject;
    newObject = PyObject_NEW(AxisAlignedSlice4DAttributesObject, &AxisAlignedSlice4DAttributesType);
    if(newObject == NULL)
        return NULL;
    newObject->data = (AxisAlignedSlice4DAttributes *)attr;
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
AxisAlignedSlice4DAttributes_new(PyObject *self, PyObject *args)
{
    int useCurrent = 0;
    if (!PyArg_ParseTuple(args, "i", &useCurrent))
    {
        if (!PyArg_ParseTuple(args, ""))
            return NULL;
        else
            PyErr_Clear();
    }

    return (PyObject *)NewAxisAlignedSlice4DAttributes(useCurrent);
}

//
// Plugin method table. These methods are added to the visitmodule's methods.
//
static PyMethodDef AxisAlignedSlice4DAttributesMethods[] = {
    {"AxisAlignedSlice4DAttributes", AxisAlignedSlice4DAttributes_new, METH_VARARGS},
    {NULL,      NULL}        /* Sentinel */
};

static Observer *AxisAlignedSlice4DAttributesObserver = 0;

std::string
PyAxisAlignedSlice4DAttributes_GetLogString()
{
    std::string s("AxisAlignedSlice4DAtts = AxisAlignedSlice4DAttributes()\n");
    if(currentAtts != 0)
        s += PyAxisAlignedSlice4DAttributes_ToString(currentAtts, "AxisAlignedSlice4DAtts.");
    return s;
}

static void
PyAxisAlignedSlice4DAttributes_CallLogRoutine(Subject *subj, void *data)
{
    typedef void (*logCallback)(const std::string &);
    logCallback cb = (logCallback)data;

    if(cb != 0)
    {
        std::string s("AxisAlignedSlice4DAtts = AxisAlignedSlice4DAttributes()\n");
        s += PyAxisAlignedSlice4DAttributes_ToString(currentAtts, "AxisAlignedSlice4DAtts.");
        cb(s);
    }
}

void
PyAxisAlignedSlice4DAttributes_StartUp(AxisAlignedSlice4DAttributes *subj, void *data)
{
    if(subj == 0)
        return;

    currentAtts = subj;
    PyAxisAlignedSlice4DAttributes_SetDefaults(subj);

    //
    // Create the observer that will be notified when the attributes change.
    //
    if(AxisAlignedSlice4DAttributesObserver == 0)
    {
        AxisAlignedSlice4DAttributesObserver = new ObserverToCallback(subj,
            PyAxisAlignedSlice4DAttributes_CallLogRoutine, (void *)data);
    }

}

void
PyAxisAlignedSlice4DAttributes_CloseDown()
{
    delete defaultAtts;
    defaultAtts = 0;
    delete AxisAlignedSlice4DAttributesObserver;
    AxisAlignedSlice4DAttributesObserver = 0;
}

PyMethodDef *
PyAxisAlignedSlice4DAttributes_GetMethodTable(int *nMethods)
{
    *nMethods = 1;
    return AxisAlignedSlice4DAttributesMethods;
}

bool
PyAxisAlignedSlice4DAttributes_Check(PyObject *obj)
{
    return (obj->ob_type == &AxisAlignedSlice4DAttributesType);
}

AxisAlignedSlice4DAttributes *
PyAxisAlignedSlice4DAttributes_FromPyObject(PyObject *obj)
{
    AxisAlignedSlice4DAttributesObject *obj2 = (AxisAlignedSlice4DAttributesObject *)obj;
    return obj2->data;
}

PyObject *
PyAxisAlignedSlice4DAttributes_New()
{
    return NewAxisAlignedSlice4DAttributes(0);
}

PyObject *
PyAxisAlignedSlice4DAttributes_Wrap(const AxisAlignedSlice4DAttributes *attr)
{
    return WrapAxisAlignedSlice4DAttributes(attr);
}

void
PyAxisAlignedSlice4DAttributes_SetParent(PyObject *obj, PyObject *parent)
{
    AxisAlignedSlice4DAttributesObject *obj2 = (AxisAlignedSlice4DAttributesObject *)obj;
    obj2->parent = parent;
}

void
PyAxisAlignedSlice4DAttributes_SetDefaults(const AxisAlignedSlice4DAttributes *atts)
{
    if(defaultAtts)
        delete defaultAtts;

    defaultAtts = new AxisAlignedSlice4DAttributes(*atts);
}

