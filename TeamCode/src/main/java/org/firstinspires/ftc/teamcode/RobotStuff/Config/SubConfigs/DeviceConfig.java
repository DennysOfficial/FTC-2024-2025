package org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DeviceConfig {

    ///drive
    public final String frontRightDrive = "FR";
    public final DcMotorSimple.Direction frontRightDriveDir = DcMotorSimple.Direction.FORWARD;


    public final String frontLeftDrive = "FL";
    public final DcMotorSimple.Direction frontLeftDriveDir = DcMotorSimple.Direction.FORWARD;


    public final String backRightDrive = "BR";
    public final DcMotorSimple.Direction backRightDriveDir = DcMotorSimple.Direction.FORWARD;


    public final String backLeftDrive = "BL";
    public final DcMotorSimple.Direction backLeftDriveDir = DcMotorSimple.Direction.REVERSE;

    /// pivots and lifts ##############################################################################################################
    public final String leftPivot = "PivotL";
    public final String rightPivot = "PivotR";


    public final String leftLift = "LiftL";

    public final String rightLift = "LiftR";


    /// specimen claw ##############################################################################################################
    public final String specimenWristServo = "SpWrist";
    public final String specimenClawServo = "SpClaw";


    /// harpoon ##############################################################################################################
    public final String harpoonGrabServo = "hg"; // harpoon grab
    public final String harpoonDepositWristServo = "hd"; // harpoon deposit
    public final String harpoonBlockAlignmentWristServo = "hba"; // harpoon block alignment zxdzxd4


    /// stuff ##############################################################################################################

    public final String elbowServo = "Elbow";
    public final String intakeServo = "suck";
    public final String intakeMotor = "suck";


    public final String flapServo = "Flap";
    //public final String supportServo = "R";



}
