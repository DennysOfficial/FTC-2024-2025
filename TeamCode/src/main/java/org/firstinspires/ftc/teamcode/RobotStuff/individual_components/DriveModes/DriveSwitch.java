package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

import java.util.ArrayList;
import java.util.List;

public class DriveSwitch {

    private List<DriveModeBase> driveModes = new ArrayList<>();

    private int activeDriveIndex = 0;
    private DriveModeBase activeDrive;
    private String activeDriveName;

    OpMode opmode;
    RobotConfig config;

    public DriveSwitch(OpMode opmode, RobotConfig config) {
        this.opmode = opmode;
        this.config = config;
        DriveModeBase defaultDriveMode = driveModes.get(0);
    }
    public void addDriveMode(DriveModeBase driveMode) {
        driveModes.add(driveMode);
        activeDrive = driveMode;

    }

    public void setActiveDrive(DriveModeBase activateDriveMode) {

    }
}
