package org.firstinspires.ftc.teamcode.motionControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;
import org.firstinspires.ftc.teamcode.individual_components.Pivot.PivotBasic;

public class testing {

    LinearOpMode opMode;
    ElapsedTime runtime;
    RobotConfig config;

    PivotBasic spinyBoi;

    motionState Status = motionState.DirectControl;

    public testing(LinearOpMode opMode, ElapsedTime runtime, RobotConfig config) {
        this.opMode = opMode;
        this.runtime = runtime;
        this.config = config;

        spinyBoi = new PivotBasic(opMode, config);
    }

    public enum motionState {
        DirectControl,
        Moving,
        Idle,

    }


    public void update(double deltaTime) {

        switch (Status){
            case DirectControl:
                spinyBoi.directControlStockPID(deltaTime);
                break;

            case Moving:

                break;

            case Idle:

                break;
        }

    }
}
