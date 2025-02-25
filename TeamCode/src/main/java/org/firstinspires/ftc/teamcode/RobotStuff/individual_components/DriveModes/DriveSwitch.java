package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

import java.util.ArrayList;
import java.util.List;

public class DriveSwitch extends DriveModeBase{

    private List<DriveModeBase> driveModes = new ArrayList<>();

    private int activeDriveIndex = 0;
    private DriveModeBase activeDrive;
    private String modeName;

    OpMode opmode;
    RobotConfig config;

    public DriveSwitch(OpMode opmode, RobotConfig config) {
        super(opmode, config);
    }
    public void addDriveMode(DriveModeBase driveMode) {
        driveModes.add(driveMode);
        activeDrive = driveMode;
    }

    public void setActiveDrive(int modeIndex) {
        activeDrive = driveModes.get(modeIndex);
        modeName = activeDrive.getClass().getSimpleName();
    }

    @Override
    public void updateDrive(double deltaTime) {
        if (config.playerOne.driveModeUp.getRisingState()) {
            activeDriveIndex = (activeDriveIndex + 1 == driveModes.size()) ? 0 : activeDriveIndex + 1;
        }

        if (config.playerOne.driveModeDown.getRisingState()) {
            activeDriveIndex = (activeDriveIndex - 1 < 0) ? driveModes.size() - 1 : activeDriveIndex - 1;
        }

        setActiveDrive(activeDriveIndex);

        opmode.telemetry.addData("Drive Mode", modeName);

        activeDrive.updateDrive(deltaTime);
    }
}
