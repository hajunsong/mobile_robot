#pragma once

#include <math.h>
#include <iostream>

using namespace std;

class PathTrackingMR{
public:
    PathTrackingMR();
    ~PathTrackingMR();
    int move(double Px, double Py, double *vx, double *wz);
private:
    double WPcur[2], WPnxt[2];
    double dn, u_wp, yaw_wp;
    double Xd, Yd, Lpv, Xpv, Ypv, yaw_d;
    int wp_size, wp_indx;
    double min_vel, max_vel;
};