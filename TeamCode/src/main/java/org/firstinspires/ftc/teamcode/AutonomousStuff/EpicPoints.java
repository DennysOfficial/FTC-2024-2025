package org.firstinspires.ftc.teamcode.AutonomousStuff;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.pathgen.Point;

@Config
public class EpicPoints {




    public static double aaa_hangX = 20;//30;

    //public static double initalWait = .5;
    public static double AAA_initialArmSettleWait = .2;

    public static DashboardPoint aa_rungPoint1 = new DashboardPoint(aaa_hangX, 74.35, Point.CARTESIAN);
    public static DashboardPoint ab_rungPoint3 = new DashboardPoint(aaa_hangX, 71, Point.CARTESIAN);
    public static DashboardPoint ac_rungPoint2 = new DashboardPoint(aaa_hangX, 73, Point.CARTESIAN);
    public static DashboardPoint ad_rungPoint4 = new DashboardPoint(aaa_hangX, 69, Point.CARTESIAN);

    public static DashboardPoint ba_rungApproachPoint1 = new DashboardPoint(18, 73, Point.CARTESIAN);
    public static DashboardPoint bb_rungApproachPoint2 = new DashboardPoint(18, 71, Point.CARTESIAN);
    public static DashboardPoint bc_rungApproachPoint3 = new DashboardPoint(18, 69, Point.CARTESIAN);
    public static DashboardPoint bd_rungApproachPoint4 = new DashboardPoint(18, 67, Point.CARTESIAN);

    public static DashboardPoint ca_scorePathControlPoint1 = new DashboardPoint(15, 30, Point.CARTESIAN);
    public static DashboardPoint cb_scorePathControlPoint2 = new DashboardPoint(15, 50, Point.CARTESIAN);

    public static DashboardPoint da_pickupPointApproach = new DashboardPoint(14, 26, Point.CARTESIAN);
    public static DashboardPoint db_pickupPointAtWallDefault = new DashboardPoint(10, 26, Point.CARTESIAN);
    public static DashboardPoint dc_pickupPointFirstWallPickup = new DashboardPoint(10, 26, Point.CARTESIAN);


    public static DashboardPoint sampleCurvePoint1 = new DashboardPoint(17, 20, Point.CARTESIAN);
    public static DashboardPoint sampleCurvePoint2 = new DashboardPoint(66, 48, Point.CARTESIAN);
    public static DashboardPoint sampleCurvePoint3 = new DashboardPoint(66, 30, Point.CARTESIAN);
    public static DashboardPoint sampleCurvePoint4 = new DashboardPoint(66, 20, Point.CARTESIAN);

    public static DashboardPoint samplepoint1 = new DashboardPoint(54, 26.5, Point.CARTESIAN);
    public static DashboardPoint samplepoint2 = new DashboardPoint(54, 17.5, Point.CARTESIAN);
    public static DashboardPoint samplepoint3 = new DashboardPoint(56, 12, Point.CARTESIAN);

    public static DashboardPoint linepoint1 = new DashboardPoint(30, 25, Point.CARTESIAN);
    public static DashboardPoint linepoint2 = new DashboardPoint(30, 17.7, Point.CARTESIAN);
    public static DashboardPoint linepoint3 = new DashboardPoint(28, 11.85, Point.CARTESIAN);



}
