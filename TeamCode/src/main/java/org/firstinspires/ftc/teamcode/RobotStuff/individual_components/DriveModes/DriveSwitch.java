package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

import java.util.ArrayList;
import java.util.List;

public class DriveSwitch {

    List<DriveModeBase> driveModes = new ArrayList<>();

    DriveModeBase defaultDriveMode = driveModes[0];

    OpMode opmode;
    RobotConfig config;

    public DriveSwitch(OpMode opmode, RobotConfig config) {
        this.opmode = opmode;
        this.config = config;
        DriveModeBase defaultDriveMode = driveModes[0];
    }
    public void addDriveMode(DriveModeBase driveMode) {

    }
}
