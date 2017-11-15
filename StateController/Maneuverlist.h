#ifndef MANEUVERLIST_H
#define MANEUVERLIST_H

struct tAADC_Maneuver
{
    int id;
    cString action;
};

struct tSector
{
    int id;
    std::vector<tAADC_Maneuver> maneuverList;
};

#endif