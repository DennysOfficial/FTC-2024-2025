import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories.UKF;

@TeleOp(name = "UKF OpMode")
public class UkfOpMode extends LinearOpMode {
    private UKF ukf;
    private BNO055IMU imu;
    private double lastTime;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        ukf = new UKF(0, 0, 0);
        lastTime = getRuntime();

        waitForStart();
        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double deltaT = currentTime - lastTime;
            lastTime = currentTime;

            double imuTheta = imu.getAngularOrientation().firstAngle;
            double imuOmega = imu.getAngularVelocity().zRotationRate;

            double otosX = getOtosX();
            double otosY = getOtosY();
            double pinpointX = getPinpointX();
            double pinpointY = getPinpointY();

            ukf.predict(imuTheta, imuOmega, deltaT);
            ukf.update(otosX, otosY, pinpointX, pinpointY);

            telemetry.addData("X", ukf.getX());
            telemetry.addData("Y", ukf.getY());
            telemetry.addData("Theta", ukf.getTheta());
            telemetry.update();
        }
    }

    private double getOtosX() {
        return 0; // Replace with OTOS sensor retrieval
    }

    private double getOtosY() {
        return 0; // Replace with OTOS sensor retrieval
    }

    private double getPinpointX() {
        return 0; // Replace with Pinpoint sensor retrieval
    }

    private double getPinpointY() {
        return 0; // Replace with Pinpoint sensor retrieval
    }
}
