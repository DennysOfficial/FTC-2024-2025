package org.firstinspires.ftc.teamcode.Config.SubConfigs;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DeviceConfig {
    public final String frontRightDrive = "FR";
    public final DcMotorSimple.Direction frontRightDriveDir = DcMotorSimple.Direction.FORWARD;


    public final String frontLeftDrive = "FL";
    public final DcMotorSimple.Direction frontLeftDriveDir = DcMotorSimple.Direction.FORWARD;


    public final String backRightDrive = "BR";
    public final DcMotorSimple.Direction backRightDriveDir = DcMotorSimple.Direction.FORWARD;


    public final String backLeftDrive = "BL";
    public final DcMotorSimple.Direction backLeftDriveDir = DcMotorSimple.Direction.REVERSE;


    public final String leftPivot = "PivotL";

    public final String rightPivot = "PivotR";

    public final String leftLift = "LiftL";

    public final String rightLift = "LiftR";

    public final String grabberServo = "Pinch";

    public final String flapServo = "Flap";

}
