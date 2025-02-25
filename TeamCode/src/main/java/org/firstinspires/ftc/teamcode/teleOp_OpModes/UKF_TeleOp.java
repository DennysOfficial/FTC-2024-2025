package org.firstinspires.ftc.teamcode.teleOp_OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories.UKF_Localization;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class UKF_TeleOp extends LinearOpMode {
    private UKF_Localization ukf;
    private BNO055IMU imu;
    private double lastTime;
    private double theta = 0;  // Estimated heading
    private double omega = 0;  // Angular velocity

    @Override
    public void runOpMode() {
        ukf = new UKF_Localization();
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

        waitForStart();
        lastTime = getRuntime();

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            // Read IMU data
            theta = imu.getAngularOrientation().firstAngle;
            omega = imu.getAngularVelocity().zRotationRate;

            // Simulated OTOS and Pinpoint data
            double otosX = 1.0, otosY = 1.0;
            double pinX = 1.2, pinY = 1.1;

            // Predict with IMU heading & angular velocity
            ukf.predict(dt, theta, omega);
            ukf.update(otosX, otosY, pinX, pinY);

            telemetry.addData("X", ukf.getX());
            telemetry.addData("Y", ukf.getY());
            telemetry.addData("Theta", ukf.getTheta());
            telemetry.update();
        }
    }
}
