#include "path_tracking.hpp"

double path1_temp[6][4] = {
    -0.261, 0.249,0.980, 0.200,
    -0.230, 0.211,0.967, 0.255,
    -0.632, 0.459,0.950, 0.313,
    -0.927, 0.684,0.923, 0.385,
    -1.217, 1.053,0.879, 0.476,
    // -1.307, 1.138,0.829, 0.559,
    -1.386, 1.333,0.779, 0.627
};

PathTrackingMR::PathTrackingMR(){
    wp_indx = 0;
    min_vel = 0.1;
    max_vel = 0.2;
    wp_size = 6;
}

PathTrackingMR::~PathTrackingMR(){}

int PathTrackingMR::move(double Px, double Py, double *vx, double *wz){
    WPcur[0] = path1_temp[wp_indx][0];
    WPcur[1] = path1_temp[wp_indx][1];
    WPnxt[0] = path1_temp[wp_indx + 1][0];
    WPnxt[1] = path1_temp[wp_indx + 1][1];
    dn = sqrt(pow((WPnxt[0] - WPcur[0]), 2) + pow((WPnxt[1] - WPcur[1]), 2));
    u_wp = ((Px - WPcur[0])*(WPnxt[0] - WPcur[0]) + (Py - WPcur[1])*(WPnxt[1] - WPcur[1]))/(dn*dn);
    yaw_wp = atan2((WPnxt[1] - WPcur[1]), (WPnxt[0] - WPcur[0]));
    while(u_wp >= 1){
        wp_indx++;
        if (wp_indx >= wp_size) break;
        WPcur[0] = path1_temp[wp_indx][0];
        WPcur[1] = path1_temp[wp_indx][1];
        WPnxt[0] = path1_temp[wp_indx + 1][0];
        WPnxt[1] = path1_temp[wp_indx + 1][1];
        dn = sqrt(pow((WPnxt[0] - WPcur[0]), 2) + pow((WPnxt[1] - WPcur[1]), 2));
        u_wp = ((Px - WPcur[0])*(WPnxt[0] - WPcur[0]) + (Py - WPcur[1])*(WPnxt[1] - WPcur[1]))/(dn*dn);
        yaw_wp = atan2((WPnxt[1] - WPcur[1]), (WPnxt[0] - WPcur[0]));
    }
    Xd = WPcur[0] + u_wp*(WPnxt[0] - WPcur[0]);
    Yd = WPcur[1] + u_wp*(WPnxt[1] - WPcur[1]);
    Lpv = 0.05;
    Xpv = Xd + Lpv*cos(yaw_wp);
    Ypv = Yd + Lpv*sin(yaw_wp);
    yaw_d = atan2(Ypv - Py, Xpv - Px);

    cout << "Px : " << Px << ", " << "Py : " << Py << endl;
    cout << "yaw_d : " << yaw_d << endl;
    cout << "wp_indx : " << wp_indx << endl;

    *vx = 0.1;
    if (yaw_d < 0){
        *wz = 0.2;
    }
    else{
        *wz = -0.2;
    }

    if (wp_indx >= wp_size - 1){
        return 1;
    }
    else{
        return 0;
    }
}