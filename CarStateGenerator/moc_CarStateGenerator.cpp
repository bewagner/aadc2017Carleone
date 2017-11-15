/****************************************************************************
** Meta object code from reading C++ file 'CarStateGenerator.h'
**
** Created: Thu Aug 31 20:47:03 2017
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "stdafx.h"
#include "/home/aadc/ADTF/src/aadcUser/CarStateGenerator/CarStateGenerator.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'CarStateGenerator.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_carStateGenerator[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      19,   18,   18,   18, 0x0a,
      41,   18,   18,   18, 0x0a,
      62,   18,   18,   18, 0x0a,
      83,   18,   18,   18, 0x0a,
     106,   18,   18,   18, 0x0a,
     128,   18,   18,   18, 0x0a,
     156,  150,   18,   18, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_carStateGenerator[] = {
    "carStateGenerator\0\0OnTransmitValueZero()\0"
    "OnTransmitValueOne()\0OnTransmitValueTwo()\0"
    "OnTransmitValueThree()\0OnTransmitValueFour()\0"
    "OnTransmitValueFive()\0value\0"
    "Transmit(tFloat32)\0"
};

const QMetaObject carStateGenerator::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_carStateGenerator,
      qt_meta_data_carStateGenerator, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &carStateGenerator::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *carStateGenerator::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *carStateGenerator::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_carStateGenerator))
        return static_cast<void*>(const_cast< carStateGenerator*>(this));
    if (!strcmp(_clname, "cBaseQtFilter"))
        return static_cast< cBaseQtFilter*>(const_cast< carStateGenerator*>(this));
    return QObject::qt_metacast(_clname);
}

int carStateGenerator::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: OnTransmitValueZero(); break;
        case 1: OnTransmitValueOne(); break;
        case 2: OnTransmitValueTwo(); break;
        case 3: OnTransmitValueThree(); break;
        case 4: OnTransmitValueFour(); break;
        case 5: OnTransmitValueFive(); break;
        case 6: Transmit((*reinterpret_cast< tFloat32(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 7;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
