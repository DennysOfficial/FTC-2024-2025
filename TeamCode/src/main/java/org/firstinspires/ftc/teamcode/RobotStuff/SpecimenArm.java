package org.firstinspires.ftc.teamcode.RobotStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftPivot;

public class SpecimenArm {


    OpMode opmode;
    RobotConfig config;

    LeftLift leftLift;

    LeftPivot leftPivot;


    SpecimenArm(OpMode opMode, RobotConfig config) {
        this.opmode = opMode;
        this.config = config;

        leftLift = new LeftLift(ControlAxis.ControlMode.positionControl, opmode, config);
        leftPivot = new LeftPivot(ControlAxis.ControlMode.positionControl, opmode, config);

        leftPivot.assignLift(leftLift);
        leftLift.assignPivot(leftPivot);
    }


}
