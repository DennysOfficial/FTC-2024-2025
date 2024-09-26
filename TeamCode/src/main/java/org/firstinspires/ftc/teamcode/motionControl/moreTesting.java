package org.firstinspires.ftc.teamcode.motionControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;

public class moreTesting {

    LinearOpMode opMode;
    ElapsedTime runtime;
    RobotConfig config;

    public moreTesting(LinearOpMode opMode, ElapsedTime runtime, RobotConfig config) {
        this.opMode = opMode;
        this.runtime = runtime;
        this.config = config;
    }

    double deltaTime;
}
