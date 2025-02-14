package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ButtonEdgeDetector;

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
        activeDrive.resetDrive();
    }

    public void setActiveDrive(int driveIndex) {
        activeDrive = driveModes.get(driveIndex);
        activeDriveName = activeDrive.getClass().getSimpleName();
        activeDrive.resetDrive();
    }

    ButtonEdgeDetector cycleRightButton;
    ButtonEdgeDetector cycleLeftButton;

    public SwitchableDrive(OpMode opMode, RobotConfig config) {
        super(opMode, config);
        cycleRightButton = new ButtonEdgeDetector(config.inputMap.getDriveModeCycleRightButton());
        cycleLeftButton = new ButtonEdgeDetector(config.inputMap.getDriveModeCycleLeftButton());
    }


    @Override
    public void updateDrive(double deltaTime) {
        if (cycleRightButton.getButtonDown(config.inputMap.getDriveModeCycleRightButton())) {
            activeDriveIndex = (activeDriveIndex + 1 < driveModes.size()) ? activeDriveIndex + 1 : 0;
            setActiveDrive(activeDriveIndex);
        }

        if (cycleLeftButton.getButtonDown(config.inputMap.getDriveModeCycleLeftButton())) {
            activeDriveIndex = (activeDriveIndex > 0) ? activeDriveIndex - 1 : 0;
            setActiveDrive(activeDriveIndex);
        }

        activeDrive.updateDrive(deltaTime);

        opMode.telemetry.addData("activeDriveMode:", activeDriveName);
    }

    @Override
    public void resetDrive() {
        activeDrive.resetDrive();
    }


}
