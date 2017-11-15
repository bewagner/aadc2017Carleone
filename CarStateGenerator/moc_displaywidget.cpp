/****************************************************************************
** Meta object code from reading C++ file 'displaywidget.h'
**
** Created: Thu Aug 31 20:47:03 2017
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "stdafx.h"
#include "/home/aadc/ADTF/src/aadcUser/CarStateGenerator/displaywidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'displaywidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_DisplayWidget[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: signature, parameters, type, tag, flags
      15,   14,   14,   14, 0x05,
      31,   14,   14,   14, 0x05,
      46,   14,   14,   14, 0x05,
      61,   14,   14,   14, 0x05,
      78,   14,   14,   14, 0x05,
      94,   14,   14,   14, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_DisplayWidget[] = {
    "DisplayWidget\0\0sendValueZero()\0"
    "sendValueOne()\0sendValueTwo()\0"
    "sendValueThree()\0sendValueFour()\0"
    "sendValueFive()\0"
};

const QMetaObject DisplayWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_DisplayWidget,
      qt_meta_data_DisplayWidget, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &DisplayWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *DisplayWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *DisplayWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_DisplayWidget))
        return static_cast<void*>(const_cast< DisplayWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int DisplayWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: sendValueZero(); break;
        case 1: sendValueOne(); break;
        case 2: sendValueTwo(); break;
        case 3: sendValueThree(); break;
        case 4: sendValueFour(); break;
        case 5: sendValueFive(); break;
        default: ;
        }
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void DisplayWidget::sendValueZero()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void DisplayWidget::sendValueOne()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void DisplayWidget::sendValueTwo()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}

// SIGNAL 3
void DisplayWidget::sendValueThree()
{
    QMetaObject::activate(this, &staticMetaObject, 3, 0);
}

// SIGNAL 4
void DisplayWidget::sendValueFour()
{
    QMetaObject::activate(this, &staticMetaObject, 4, 0);
}

// SIGNAL 5
void DisplayWidget::sendValueFive()
{
    QMetaObject::activate(this, &staticMetaObject, 5, 0);
}
QT_END_MOC_NAMESPACE
