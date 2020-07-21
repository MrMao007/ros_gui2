/****************************************************************************
** Meta object code from reading C++ file 'multinode.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/ros_gui/multinode.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'multinode.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_Ui__Multinode_t {
    QByteArrayData data[3];
    char stringdata0[30];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Ui__Multinode_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Ui__Multinode_t qt_meta_stringdata_Ui__Multinode = {
    {
QT_MOC_LITERAL(0, 0, 13), // "Ui::Multinode"
QT_MOC_LITERAL(1, 14, 14), // "multigoal_slot"
QT_MOC_LITERAL(2, 29, 0) // ""

    },
    "Ui::Multinode\0multigoal_slot\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Ui__Multinode[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   19,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void Ui::Multinode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Multinode *_t = static_cast<Multinode *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->multigoal_slot(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject Ui::Multinode::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_Ui__Multinode.data,
      qt_meta_data_Ui__Multinode,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *Ui::Multinode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Ui::Multinode::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_Ui__Multinode.stringdata0))
        return static_cast<void*>(const_cast< Multinode*>(this));
    return QThread::qt_metacast(_clname);
}

int Ui::Multinode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
