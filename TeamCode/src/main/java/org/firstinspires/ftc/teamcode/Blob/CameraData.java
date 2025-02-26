package org.firstinspires.ftc.teamcode.Blob;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class CameraData {
    int xResolution;
    int yResolution;
    Vector3D positionOnRobot;
    double headingAngleOffset;
    double pitchAngle;
    double HFOV = 70.42;
    double VFOV = 43.3;

    double liftAngle;

    double liftExtension;

    double CameraOffsetAngle;
    double xOffset ; // a set distance from otos to arm and camera

    double yOffset ; // based off extenston of lift and angle
    double zOffset ; // based off extenston of lift and angle


}
