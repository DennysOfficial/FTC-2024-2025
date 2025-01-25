package org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DeviceConfig {
    public final String frontRightDrive = "FR";
    public final DcMotorSimple.Direction frontRightDriveDir = DcMotorSimple.Direction.FORWARD;


    public final String frontLeftDrive = "FL";
    public final DcMotorSimple.Direction frontLeftDriveDir = DcMotorSimple.Direction.REVERSE;


    public final String backRightDrive = "BR";
    public final DcMotorSimple.Direction backRightDriveDir = DcMotorSimple.Direction.FORWARD;


    public final String backLeftDrive = "BL";
    public final DcMotorSimple.Direction backLeftDriveDir = DcMotorSimple.Direction.REVERSE;


    public final String leftPivot = "PivotL";
    public final String leftPivotServo1 = "PivotL1";
    public final String leftPivotServo2 = "PivotL2";

    public final String leftPivotEncoder = frontRightDrive;

    public final String rightPivot = "PivotR";

    public final String leftLift = "LiftL";

    public final String rightLift = "LiftR";

    public final String intakeServo = "suck";
    public final String intakeMotor = "suck";

    public final String flapServo = "Flap";
    public final String wristServo = "Wrist";

    public final String elbowServo = "Elbow";
    public final String spWristServo = "SpWrist";


}
