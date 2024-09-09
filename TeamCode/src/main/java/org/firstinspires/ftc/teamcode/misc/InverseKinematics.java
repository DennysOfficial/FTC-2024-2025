package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.individual_components.Lift;
import org.firstinspires.ftc.teamcode.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.misc.PID_Stuff.CustomPID;
import org.firstinspires.ftc.teamcode.profiles_and_base_settings.Settings;

/**
 * look for the picture kinematics.jpg in the misc folder for reference
 */
public class InverseKinematics {

    Pose pose = new Pose();
    Pivot pivot;
    Lift lift;
    OpMode opMode;
    Settings settings;
    double pivotArmLength = 12;

    public InverseKinematics(Pivot pivot, Lift lift, OpMode opMode, Settings settings) {
        this.pivot = pivot;
        this.lift = lift;
        this.opMode = opMode;
        this.settings = settings;

    }

    public void updatePose() {
        double liftXComponent = lift.getPositionInch() / 2;
        double pivotXComponent = Math.sin(Math.toRadians(pivot.getTargetPositionDeg()));

        pose.currentX = pivotXComponent + liftXComponent;

        double liftYComponent = lift.getPositionInch() / (2 * Math.sqrt(3));
        double pivotYComponent = -Math.cos(Math.toRadians(pivot.getTargetPositionDeg()));

        pose.currentY = pivotYComponent + liftYComponent;
    }


    public void setPosition(double x, double y) {

        pose.targetY = y;
        pose.targetX = x;

        double angleA = Math.toRadians(60) - Math.atan(y / x);
        double side_b = Math.sqrt(x * x + y * y);

        double fistPivotSolution = Math.toDegrees(Math.asin((side_b * Math.sin(angleA)) / pivotArmLength));
        double secondPivotSolution = 180 - fistPivotSolution;

        double pivotAngle  = ((fistPivotSolution- pivot.getTargetPositionDeg()) < Math.abs(secondPivotSolution- pivot.getTargetPositionDeg())) ? fistPivotSolution : secondPivotSolution;

        double angleC = 180 - angleA - pivotAngle;
        angleC = Math.toRadians(angleC);

        double liftExtension = Math.toDegrees(Math.sin(angleC) * pivotArmLength / Math.sin(angleA));

        pivot.setTargetPositionDeg(pivotAngle);
        lift.setPositionInch(liftExtension);
    }
}
