/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/ros_gui/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[58];
    char stringdata0[1058];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 18), // "record_path_signal"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 16), // "save_path_signal"
QT_MOC_LITERAL(4, 48, 12), // "track_signal"
QT_MOC_LITERAL(5, 61, 17), // "track_shut_signal"
QT_MOC_LITERAL(6, 79, 17), // "door_front_signal"
QT_MOC_LITERAL(7, 97, 14), // "door_in_signal"
QT_MOC_LITERAL(8, 112, 15), // "door_out_signal"
QT_MOC_LITERAL(9, 128, 11), // "dock_signal"
QT_MOC_LITERAL(10, 140, 19), // "demostration_signal"
QT_MOC_LITERAL(11, 160, 10), // "power_slot"
QT_MOC_LITERAL(12, 171, 1), // "p"
QT_MOC_LITERAL(13, 173, 9), // "flag_slot"
QT_MOC_LITERAL(14, 183, 1), // "f"
QT_MOC_LITERAL(15, 185, 9), // "temp_slot"
QT_MOC_LITERAL(16, 195, 1), // "t"
QT_MOC_LITERAL(17, 197, 11), // "startp_slot"
QT_MOC_LITERAL(18, 209, 9), // "endp_slot"
QT_MOC_LITERAL(19, 219, 17), // "markersignal_slot"
QT_MOC_LITERAL(20, 237, 16), // "line_startp_slot"
QT_MOC_LITERAL(21, 254, 14), // "line_endp_slot"
QT_MOC_LITERAL(22, 269, 22), // "line_markersignal_slot"
QT_MOC_LITERAL(23, 292, 11), // "pointp_slot"
QT_MOC_LITERAL(24, 304, 23), // "point_markersignal_slot"
QT_MOC_LITERAL(25, 328, 14), // "semanticp_slot"
QT_MOC_LITERAL(26, 343, 26), // "semantic_markersignal_slot"
QT_MOC_LITERAL(27, 370, 12), // "setgoal_slot"
QT_MOC_LITERAL(28, 383, 21), // "door_front_ready_slot"
QT_MOC_LITERAL(29, 405, 18), // "door_in_ready_slot"
QT_MOC_LITERAL(30, 424, 19), // "door_out_ready_slot"
QT_MOC_LITERAL(31, 444, 15), // "dock_ready_slot"
QT_MOC_LITERAL(32, 460, 16), // "dock_ready_slot2"
QT_MOC_LITERAL(33, 477, 23), // "demostration_ready_slot"
QT_MOC_LITERAL(34, 501, 22), // "on_radioButton_toggled"
QT_MOC_LITERAL(35, 524, 5), // "state"
QT_MOC_LITERAL(36, 530, 24), // "on_radioButton_2_toggled"
QT_MOC_LITERAL(37, 555, 24), // "on_radioButton_3_toggled"
QT_MOC_LITERAL(38, 580, 24), // "on_radioButton_4_toggled"
QT_MOC_LITERAL(39, 605, 24), // "on_radioButton_5_toggled"
QT_MOC_LITERAL(40, 630, 24), // "on_radioButton_6_toggled"
QT_MOC_LITERAL(41, 655, 24), // "on_radioButton_7_toggled"
QT_MOC_LITERAL(42, 680, 24), // "on_radioButton_8_toggled"
QT_MOC_LITERAL(43, 705, 24), // "on_radioButton_9_toggled"
QT_MOC_LITERAL(44, 730, 25), // "on_radioButton_10_toggled"
QT_MOC_LITERAL(45, 756, 21), // "on_pushButton_clicked"
QT_MOC_LITERAL(46, 778, 23), // "on_pushButton_2_clicked"
QT_MOC_LITERAL(47, 802, 23), // "on_pushButton_3_clicked"
QT_MOC_LITERAL(48, 826, 24), // "on_pushButton_23_clicked"
QT_MOC_LITERAL(49, 851, 24), // "on_pushButton_24_clicked"
QT_MOC_LITERAL(50, 876, 24), // "on_pushButton_25_clicked"
QT_MOC_LITERAL(51, 901, 24), // "on_pushButton_26_clicked"
QT_MOC_LITERAL(52, 926, 24), // "on_pushButton_27_clicked"
QT_MOC_LITERAL(53, 951, 24), // "on_pushButton_28_clicked"
QT_MOC_LITERAL(54, 976, 24), // "on_pushButton_29_clicked"
QT_MOC_LITERAL(55, 1001, 24), // "on_pushButton_30_clicked"
QT_MOC_LITERAL(56, 1026, 24), // "on_pushButton_31_clicked"
QT_MOC_LITERAL(57, 1051, 6) // "reshow"

    },
    "MainWindow\0record_path_signal\0\0"
    "save_path_signal\0track_signal\0"
    "track_shut_signal\0door_front_signal\0"
    "door_in_signal\0door_out_signal\0"
    "dock_signal\0demostration_signal\0"
    "power_slot\0p\0flag_slot\0f\0temp_slot\0t\0"
    "startp_slot\0endp_slot\0markersignal_slot\0"
    "line_startp_slot\0line_endp_slot\0"
    "line_markersignal_slot\0pointp_slot\0"
    "point_markersignal_slot\0semanticp_slot\0"
    "semantic_markersignal_slot\0setgoal_slot\0"
    "door_front_ready_slot\0door_in_ready_slot\0"
    "door_out_ready_slot\0dock_ready_slot\0"
    "dock_ready_slot2\0demostration_ready_slot\0"
    "on_radioButton_toggled\0state\0"
    "on_radioButton_2_toggled\0"
    "on_radioButton_3_toggled\0"
    "on_radioButton_4_toggled\0"
    "on_radioButton_5_toggled\0"
    "on_radioButton_6_toggled\0"
    "on_radioButton_7_toggled\0"
    "on_radioButton_8_toggled\0"
    "on_radioButton_9_toggled\0"
    "on_radioButton_10_toggled\0"
    "on_pushButton_clicked\0on_pushButton_2_clicked\0"
    "on_pushButton_3_clicked\0"
    "on_pushButton_23_clicked\0"
    "on_pushButton_24_clicked\0"
    "on_pushButton_25_clicked\0"
    "on_pushButton_26_clicked\0"
    "on_pushButton_27_clicked\0"
    "on_pushButton_28_clicked\0"
    "on_pushButton_29_clicked\0"
    "on_pushButton_30_clicked\0"
    "on_pushButton_31_clicked\0reshow"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      52,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       9,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,  274,    2, 0x06 /* Public */,
       3,    0,  275,    2, 0x06 /* Public */,
       4,    0,  276,    2, 0x06 /* Public */,
       5,    0,  277,    2, 0x06 /* Public */,
       6,    0,  278,    2, 0x06 /* Public */,
       7,    0,  279,    2, 0x06 /* Public */,
       8,    0,  280,    2, 0x06 /* Public */,
       9,    0,  281,    2, 0x06 /* Public */,
      10,    0,  282,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      11,    1,  283,    2, 0x0a /* Public */,
      13,    1,  286,    2, 0x0a /* Public */,
      15,    1,  289,    2, 0x0a /* Public */,
      17,    0,  292,    2, 0x0a /* Public */,
      18,    0,  293,    2, 0x0a /* Public */,
      19,    0,  294,    2, 0x0a /* Public */,
      20,    0,  295,    2, 0x0a /* Public */,
      21,    0,  296,    2, 0x0a /* Public */,
      22,    0,  297,    2, 0x0a /* Public */,
      23,    0,  298,    2, 0x0a /* Public */,
      24,    0,  299,    2, 0x0a /* Public */,
      25,    0,  300,    2, 0x0a /* Public */,
      26,    0,  301,    2, 0x0a /* Public */,
      27,    0,  302,    2, 0x0a /* Public */,
      28,    0,  303,    2, 0x0a /* Public */,
      29,    0,  304,    2, 0x0a /* Public */,
      30,    0,  305,    2, 0x0a /* Public */,
      31,    0,  306,    2, 0x0a /* Public */,
      32,    0,  307,    2, 0x0a /* Public */,
      33,    0,  308,    2, 0x0a /* Public */,
      34,    1,  309,    2, 0x08 /* Private */,
      36,    1,  312,    2, 0x08 /* Private */,
      37,    1,  315,    2, 0x08 /* Private */,
      38,    1,  318,    2, 0x08 /* Private */,
      39,    1,  321,    2, 0x08 /* Private */,
      40,    1,  324,    2, 0x08 /* Private */,
      41,    1,  327,    2, 0x08 /* Private */,
      42,    1,  330,    2, 0x08 /* Private */,
      43,    1,  333,    2, 0x08 /* Private */,
      44,    1,  336,    2, 0x08 /* Private */,
      45,    0,  339,    2, 0x08 /* Private */,
      46,    0,  340,    2, 0x08 /* Private */,
      47,    0,  341,    2, 0x08 /* Private */,
      48,    0,  342,    2, 0x08 /* Private */,
      49,    0,  343,    2, 0x08 /* Private */,
      50,    0,  344,    2, 0x08 /* Private */,
      51,    0,  345,    2, 0x08 /* Private */,
      52,    0,  346,    2, 0x08 /* Private */,
      53,    0,  347,    2, 0x08 /* Private */,
      54,    0,  348,    2, 0x08 /* Private */,
      55,    0,  349,    2, 0x08 /* Private */,
      56,    0,  350,    2, 0x08 /* Private */,
      57,    0,  351,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::Float,   12,
    QMetaType::Void, QMetaType::Float,   14,
    QMetaType::Void, QMetaType::Float,   16,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   35,
    QMetaType::Void, QMetaType::Bool,   35,
    QMetaType::Void, QMetaType::Bool,   35,
    QMetaType::Void, QMetaType::Bool,   35,
    QMetaType::Void, QMetaType::Bool,   35,
    QMetaType::Void, QMetaType::Bool,   35,
    QMetaType::Void, QMetaType::Bool,   35,
    QMetaType::Void, QMetaType::Bool,   35,
    QMetaType::Void, QMetaType::Bool,   35,
    QMetaType::Void, QMetaType::Bool,   35,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->record_path_signal(); break;
        case 1: _t->save_path_signal(); break;
        case 2: _t->track_signal(); break;
        case 3: _t->track_shut_signal(); break;
        case 4: _t->door_front_signal(); break;
        case 5: _t->door_in_signal(); break;
        case 6: _t->door_out_signal(); break;
        case 7: _t->dock_signal(); break;
        case 8: _t->demostration_signal(); break;
        case 9: _t->power_slot((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 10: _t->flag_slot((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 11: _t->temp_slot((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 12: _t->startp_slot(); break;
        case 13: _t->endp_slot(); break;
        case 14: _t->markersignal_slot(); break;
        case 15: _t->line_startp_slot(); break;
        case 16: _t->line_endp_slot(); break;
        case 17: _t->line_markersignal_slot(); break;
        case 18: _t->pointp_slot(); break;
        case 19: _t->point_markersignal_slot(); break;
        case 20: _t->semanticp_slot(); break;
        case 21: _t->semantic_markersignal_slot(); break;
        case 22: _t->setgoal_slot(); break;
        case 23: _t->door_front_ready_slot(); break;
        case 24: _t->door_in_ready_slot(); break;
        case 25: _t->door_out_ready_slot(); break;
        case 26: _t->dock_ready_slot(); break;
        case 27: _t->dock_ready_slot2(); break;
        case 28: _t->demostration_ready_slot(); break;
        case 29: _t->on_radioButton_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 30: _t->on_radioButton_2_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 31: _t->on_radioButton_3_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 32: _t->on_radioButton_4_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 33: _t->on_radioButton_5_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 34: _t->on_radioButton_6_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 35: _t->on_radioButton_7_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 36: _t->on_radioButton_8_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 37: _t->on_radioButton_9_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 38: _t->on_radioButton_10_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 39: _t->on_pushButton_clicked(); break;
        case 40: _t->on_pushButton_2_clicked(); break;
        case 41: _t->on_pushButton_3_clicked(); break;
        case 42: _t->on_pushButton_23_clicked(); break;
        case 43: _t->on_pushButton_24_clicked(); break;
        case 44: _t->on_pushButton_25_clicked(); break;
        case 45: _t->on_pushButton_26_clicked(); break;
        case 46: _t->on_pushButton_27_clicked(); break;
        case 47: _t->on_pushButton_28_clicked(); break;
        case 48: _t->on_pushButton_29_clicked(); break;
        case 49: _t->on_pushButton_30_clicked(); break;
        case 50: _t->on_pushButton_31_clicked(); break;
        case 51: _t->reshow(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::record_path_signal)) {
                *result = 0;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::save_path_signal)) {
                *result = 1;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::track_signal)) {
                *result = 2;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::track_shut_signal)) {
                *result = 3;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::door_front_signal)) {
                *result = 4;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::door_in_signal)) {
                *result = 5;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::door_out_signal)) {
                *result = 6;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::dock_signal)) {
                *result = 7;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::demostration_signal)) {
                *result = 8;
            }
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 52)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 52;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 52)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 52;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::record_path_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void MainWindow::save_path_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void MainWindow::track_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 2, Q_NULLPTR);
}

// SIGNAL 3
void MainWindow::track_shut_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 3, Q_NULLPTR);
}

// SIGNAL 4
void MainWindow::door_front_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 4, Q_NULLPTR);
}

// SIGNAL 5
void MainWindow::door_in_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 5, Q_NULLPTR);
}

// SIGNAL 6
void MainWindow::door_out_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 6, Q_NULLPTR);
}

// SIGNAL 7
void MainWindow::dock_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 7, Q_NULLPTR);
}

// SIGNAL 8
void MainWindow::demostration_signal()
{
    QMetaObject::activate(this, &staticMetaObject, 8, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
