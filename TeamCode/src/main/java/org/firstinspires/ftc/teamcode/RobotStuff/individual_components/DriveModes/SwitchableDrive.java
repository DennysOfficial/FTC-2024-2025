package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

import java.util.ArrayList;
import java.util.List;

public class SwitchableDrive extends DriveModeBase {

    private List<DriveModeBase> driveModes = new ArrayList<>();

    private int activeDriveIndex;
    private DriveModeBase activeDrive;
    private String activeDriveName;

    public void addDriveMode(DriveModeBase driveMode) {
        driveModes.add(driveMode);
        activeDriveIndex = driveModes.size() - 1;
        setActiveDrive(driveMode);
    }

    public void setActiveDrive(DriveModeBase driveMode) {
        activeDrive = driveMode;
        activeDriveName = activeDrive.getClass().getSimpleName();
    }


    public SwitchableDrive(OpMode opMode, RobotConfig config) {
        super(opMode, config);
    }

    boolean previous
    @Override
    public void updateDrive(double deltaTime) {
        if (config.inputMap.getDriveModeCycleRightButton()) {
            if(activeDriveIndex + 1 < driveModes.size())
            activeDriveIndex
        }

        activeDrive.updateDrive(deltaTime);
    }

    @Override
    public void resetDrive() {
        activeDrive.resetDrive();
    }


}
