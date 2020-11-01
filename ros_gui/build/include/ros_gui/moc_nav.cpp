/****************************************************************************
** Meta object code from reading C++ file 'nav.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/ros_gui/nav.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'nav.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_Nav_t {
    QByteArrayData data[13];
    char stringdata0[209];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Nav_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Nav_t qt_meta_stringdata_Nav = {
    {
QT_MOC_LITERAL(0, 0, 3), // "Nav"
QT_MOC_LITERAL(1, 4, 17), // "door_front_signal"
QT_MOC_LITERAL(2, 22, 0), // ""
QT_MOC_LITERAL(3, 23, 14), // "door_in_signal"
QT_MOC_LITERAL(4, 38, 15), // "door_out_signal"
QT_MOC_LITERAL(5, 54, 12), // "vicon_signal"
QT_MOC_LITERAL(6, 67, 10), // "tea_signal"
QT_MOC_LITERAL(7, 78, 21), // "door_front_ready_slot"
QT_MOC_LITERAL(8, 100, 18), // "door_in_ready_slot"
QT_MOC_LITERAL(9, 119, 19), // "door_out_ready_slot"
QT_MOC_LITERAL(10, 139, 21), // "on_pushButton_clicked"
QT_MOC_LITERAL(11, 161, 23), // "on_pushButton_2_clicked"
QT_MOC_LITERAL(12, 185, 23) // "on_pushButton_3_clicked"

    },
    "Nav\0door_front_signal\0\0door_in_signal\0"
    "door_out_signal\0vicon_signal\0tea_signal\0"
    "door_front_ready_slot\0door_in_ready_slot\0"
    "door_out_ready_slot\0on_pushButton_clicked\0"
    "on_pushButton_2_clicked\0on_pushButton_3_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Nav[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   69,    2, 0x06 /* Public */,
       3,    0,   70,    2, 0x06 /* Public */,
       4,    0,   71,    2, 0x06 /* Public */,
       5,    0,   72,    2, 0x06 /* Public */,
       6,    0,   73,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    0,   74,    2, 0x0a /* Public */,
       8,    0,   75,    2, 0x0a /* Public */,
       9,    0,   76,    2, 0x0a /* Public */,
      10,    0,   77,    2, 0x08 /* Private */,
      11,    0,   78,    2, 0x08 /* Private */,
      12,    0,   79,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void Nav::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Nav *_t = static_cast<Nav *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->door_front_signal(); break;
        case 1: _t->door_in_signal(); break;
        case 2: _t->door_out_signal(); break;
        case 3: _t->vicon_signal(); break;
        case 4: _t->tea_signal(); break;
        case 5: _t->door_front_ready_slot(); break;
        case 6: _t->door_in_ready_slot(); break;
        case 7: _t->door_out_ready_slot(); break;
        case 8: _t->on_pushButton_clicked(); break;
        case 9: _t->on_pushButton_2_clicked(); break;
        case 10: _t->on_pushButton_3_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (Nav::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Nav::door_front_signal)) {
                *result = 0;
            }
        }
        {
            typedef void (Nav::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Nav::door_in_signal)) {
                *result = 1;
            }
        }
        {
            typedef void (Nav::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Nav::door_out_signal)) {
                *result = 2;
            }
        }
        {
            typedef void (Nav::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Nav::vicon_signal)) {
                *result = 3;
            }
        }
        {
            typedef void (Nav::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Nav::tea_signal)) {
                *result = 4;
            }
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject Nav::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_Nav.data,
      qt_meta_data_Nav,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *Nav::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Nav::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_Nav.stringdata0))
        return static_cast<void*>(const_cast< Nav*>(this));
    return QDialog::qt_metacast(_clname);
}

int Nav::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void Nav::door_front_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void Nav::door_in_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void Nav::door_out_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 2, Q_NULLPTR);
}

// SIGNAL 3
void Nav::vicon_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 3, Q_NULLPTR);
}

// SIGNAL 4
void Nav::tea_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 4, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
