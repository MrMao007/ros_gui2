/****************************************************************************
** Meta object code from reading C++ file 'mapNode.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/ros_gui/mapNode.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mapNode.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_Ui__MapNode_t {
    QByteArrayData data[24];
    char stringdata0[288];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Ui__MapNode_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Ui__MapNode_t qt_meta_stringdata_Ui__MapNode = {
    {
QT_MOC_LITERAL(0, 0, 11), // "Ui::MapNode"
QT_MOC_LITERAL(1, 12, 13), // "startpUpdated"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 1), // "x"
QT_MOC_LITERAL(4, 29, 1), // "y"
QT_MOC_LITERAL(5, 31, 11), // "endpUpdated"
QT_MOC_LITERAL(6, 43, 18), // "line_startpUpdated"
QT_MOC_LITERAL(7, 62, 16), // "line_endpUpdated"
QT_MOC_LITERAL(8, 79, 13), // "pointpUpdated"
QT_MOC_LITERAL(9, 93, 16), // "semanticpUpdated"
QT_MOC_LITERAL(10, 110, 11), // "marker_slot"
QT_MOC_LITERAL(11, 122, 16), // "line_marker_slot"
QT_MOC_LITERAL(12, 139, 17), // "point_marker_slot"
QT_MOC_LITERAL(13, 157, 6), // "radius"
QT_MOC_LITERAL(14, 164, 11), // "delete_slot"
QT_MOC_LITERAL(15, 176, 2), // "id"
QT_MOC_LITERAL(16, 179, 13), // "semantic_slot"
QT_MOC_LITERAL(17, 193, 11), // "std::string"
QT_MOC_LITERAL(18, 205, 8), // "semantic"
QT_MOC_LITERAL(19, 214, 14), // "multigoal_slot"
QT_MOC_LITERAL(20, 229, 16), // "record_path_slot"
QT_MOC_LITERAL(21, 246, 14), // "save_path_slot"
QT_MOC_LITERAL(22, 261, 10), // "track_slot"
QT_MOC_LITERAL(23, 272, 15) // "track_shut_slot"

    },
    "Ui::MapNode\0startpUpdated\0\0x\0y\0"
    "endpUpdated\0line_startpUpdated\0"
    "line_endpUpdated\0pointpUpdated\0"
    "semanticpUpdated\0marker_slot\0"
    "line_marker_slot\0point_marker_slot\0"
    "radius\0delete_slot\0id\0semantic_slot\0"
    "std::string\0semantic\0multigoal_slot\0"
    "record_path_slot\0save_path_slot\0"
    "track_slot\0track_shut_slot"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Ui__MapNode[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   94,    2, 0x06 /* Public */,
       5,    2,   99,    2, 0x06 /* Public */,
       6,    2,  104,    2, 0x06 /* Public */,
       7,    2,  109,    2, 0x06 /* Public */,
       8,    2,  114,    2, 0x06 /* Public */,
       9,    2,  119,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      10,    0,  124,    2, 0x0a /* Public */,
      11,    0,  125,    2, 0x0a /* Public */,
      12,    1,  126,    2, 0x0a /* Public */,
      14,    1,  129,    2, 0x0a /* Public */,
      16,    1,  132,    2, 0x0a /* Public */,
      19,    0,  135,    2, 0x0a /* Public */,
      20,    0,  136,    2, 0x0a /* Public */,
      21,    0,  137,    2, 0x0a /* Public */,
      22,    0,  138,    2, 0x0a /* Public */,
      23,    0,  139,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double,   13,
    QMetaType::Void, QMetaType::Int,   15,
    QMetaType::Void, 0x80000000 | 17,   18,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void Ui::MapNode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MapNode *_t = static_cast<MapNode *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->startpUpdated((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 1: _t->endpUpdated((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 2: _t->line_startpUpdated((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 3: _t->line_endpUpdated((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 4: _t->pointpUpdated((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 5: _t->semanticpUpdated((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 6: _t->marker_slot(); break;
        case 7: _t->line_marker_slot(); break;
        case 8: _t->point_marker_slot((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 9: _t->delete_slot((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->semantic_slot((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 11: _t->multigoal_slot(); break;
        case 12: _t->record_path_slot(); break;
        case 13: _t->save_path_slot(); break;
        case 14: _t->track_slot(); break;
        case 15: _t->track_shut_slot(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (MapNode::*_t)(double , double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MapNode::startpUpdated)) {
                *result = 0;
            }
        }
        {
            typedef void (MapNode::*_t)(double , double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MapNode::endpUpdated)) {
                *result = 1;
            }
        }
        {
            typedef void (MapNode::*_t)(double , double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MapNode::line_startpUpdated)) {
                *result = 2;
            }
        }
        {
            typedef void (MapNode::*_t)(double , double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MapNode::line_endpUpdated)) {
                *result = 3;
            }
        }
        {
            typedef void (MapNode::*_t)(double , double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MapNode::pointpUpdated)) {
                *result = 4;
            }
        }
        {
            typedef void (MapNode::*_t)(double , double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MapNode::semanticpUpdated)) {
                *result = 5;
            }
        }
    }
}

const QMetaObject Ui::MapNode::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_Ui__MapNode.data,
      qt_meta_data_Ui__MapNode,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *Ui::MapNode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Ui::MapNode::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_Ui__MapNode.stringdata0))
        return static_cast<void*>(const_cast< MapNode*>(this));
    return QThread::qt_metacast(_clname);
}

int Ui::MapNode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 16)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 16;
    }
    return _id;
}

// SIGNAL 0
void Ui::MapNode::startpUpdated(double _t1, double _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Ui::MapNode::endpUpdated(double _t1, double _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void Ui::MapNode::line_startpUpdated(double _t1, double _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void Ui::MapNode::line_endpUpdated(double _t1, double _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void Ui::MapNode::pointpUpdated(double _t1, double _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void Ui::MapNode::semanticpUpdated(double _t1, double _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_END_MOC_NAMESPACE
