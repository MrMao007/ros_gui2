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
    QByteArrayData data[11];
    char stringdata0[185];
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
QT_MOC_LITERAL(5, 54, 21), // "door_front_ready_slot"
QT_MOC_LITERAL(6, 76, 18), // "door_in_ready_slot"
QT_MOC_LITERAL(7, 95, 19), // "door_out_ready_slot"
QT_MOC_LITERAL(8, 115, 21), // "on_pushButton_clicked"
QT_MOC_LITERAL(9, 137, 23), // "on_pushButton_2_clicked"
QT_MOC_LITERAL(10, 161, 23) // "on_pushButton_3_clicked"

    },
    "Nav\0door_front_signal\0\0door_in_signal\0"
    "door_out_signal\0door_front_ready_slot\0"
    "door_in_ready_slot\0door_out_ready_slot\0"
    "on_pushButton_clicked\0on_pushButton_2_clicked\0"
    "on_pushButton_3_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Nav[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   59,    2, 0x06 /* Public */,
       3,    0,   60,    2, 0x06 /* Public */,
       4,    0,   61,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,   62,    2, 0x0a /* Public */,
       6,    0,   63,    2, 0x0a /* Public */,
       7,    0,   64,    2, 0x0a /* Public */,
       8,    0,   65,    2, 0x08 /* Private */,
       9,    0,   66,    2, 0x08 /* Private */,
      10,    0,   67,    2, 0x08 /* Private */,

 // signals: parameters
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
        case 3: _t->door_front_ready_slot(); break;
        case 4: _t->door_in_ready_slot(); break;
        case 5: _t->door_out_ready_slot(); break;
        case 6: _t->on_pushButton_clicked(); break;
        case 7: _t->on_pushButton_2_clicked(); break;
        case 8: _t->on_pushButton_3_clicked(); break;
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
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 9;
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
QT_END_MOC_NAMESPACE
