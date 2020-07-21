/****************************************************************************
** Meta object code from reading C++ file 'markerNode.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/ros_gui/markerNode.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'markerNode.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_Ui__MarkerNode_t {
    QByteArrayData data[8];
    char stringdata0[44];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Ui__MarkerNode_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Ui__MarkerNode_t qt_meta_stringdata_Ui__MarkerNode = {
    {
QT_MOC_LITERAL(0, 0, 14), // "Ui::MarkerNode"
QT_MOC_LITERAL(1, 15, 5), // "power"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 1), // "p"
QT_MOC_LITERAL(4, 24, 10), // "power_flag"
QT_MOC_LITERAL(5, 35, 1), // "f"
QT_MOC_LITERAL(6, 37, 4), // "temp"
QT_MOC_LITERAL(7, 42, 1) // "t"

    },
    "Ui::MarkerNode\0power\0\0p\0power_flag\0f\0"
    "temp\0t"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Ui__MarkerNode[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   29,    2, 0x06 /* Public */,
       4,    1,   32,    2, 0x06 /* Public */,
       6,    1,   35,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Float,    3,
    QMetaType::Void, QMetaType::Float,    5,
    QMetaType::Void, QMetaType::Float,    7,

       0        // eod
};

void Ui::MarkerNode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MarkerNode *_t = static_cast<MarkerNode *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->power((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 1: _t->power_flag((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 2: _t->temp((*reinterpret_cast< float(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (MarkerNode::*_t)(float );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MarkerNode::power)) {
                *result = 0;
            }
        }
        {
            typedef void (MarkerNode::*_t)(float );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MarkerNode::power_flag)) {
                *result = 1;
            }
        }
        {
            typedef void (MarkerNode::*_t)(float );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MarkerNode::temp)) {
                *result = 2;
            }
        }
    }
}

const QMetaObject Ui::MarkerNode::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_Ui__MarkerNode.data,
      qt_meta_data_Ui__MarkerNode,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *Ui::MarkerNode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Ui::MarkerNode::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_Ui__MarkerNode.stringdata0))
        return static_cast<void*>(const_cast< MarkerNode*>(this));
    return QThread::qt_metacast(_clname);
}

int Ui::MarkerNode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void Ui::MarkerNode::power(float _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Ui::MarkerNode::power_flag(float _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void Ui::MarkerNode::temp(float _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
