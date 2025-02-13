package org.firstinspires.ftc.teamcode.teleOp_OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.EKF;
import org.firstinspires.ftc.vision.VisionPortal;

public class EKF_Localization extends LinearOpMode {
    private EKF ekf = new EKF();
    private OdometrySensor odometry;
    private IMU imu;
    private VisionPortal aprilTagDetector; // make a class that sets up the apriltagdetector
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        odometry = hardwareMap.get(OdometrySensor.class, "odometry");
        imu = hardwareMap.get(IMU.class, "imu");

        waitForStart();
        double lastTime = timer.seconds();

        while (opModeIsActive()) {
            double currentTime = timer.seconds();
            double deltaTime = currentTime - lastTime;
            lastTime = currentTime;

            // Read odometry
            double[] odoPos = odometry.getPosition();  // [vx, vy, omega]
            ekf.predict(odoPos[0], odoPos[1], odoPos[2], deltaTime);

            // Read IMU heading
            double imuTheta = imu.getAngularOrientation().firstAngle;
            ekf.updateWithIMU(imuTheta);


            // Read AprilTags (simulate getting a detection)
            double[] aprilTagPose = getAprilTagPosition();
            if (aprilTagPose != null) {
                double confidence = computeAprilTagConfidence(aprilTagPose[0], aprilTagPose[1], aprilTagPose[2]);
                ekf.updateWithAprilTag(aprilTagPose[0], aprilTagPose[1], aprilTagPose[2], confidence);
            }

            // Get EKF state estimate
            double[] state = ekf.getState();
            telemetry.addData("EKF Position", "(%.2f, %.2f)", state[0], state[1]);
            telemetry.addData("EKF Heading", "%.2f°", Math.toDegrees(state[2]));
            telemetry.update();

            sleep(10);  // Loop control
        }
    }

    private double[] getAprilTagPosition() {
        // Simulated data (replace with actual FTC AprilTag detection)
        return new double[]{Math.random(), Math.random(), Math.random()};
    }

    private double computeAprilTagConfidence(double lastX, double lastY, double lastTheta) {
        double dx = Math.abs(lastX - ekf.getState()[0]);
        double dy = Math.abs(lastY - ekf.getState()[1]);
        double dTheta = Math.abs(lastTheta - ekf.getState()[2]);
        double positionChange = Math.sqrt(dx * dx + dy * dy);
        double thetaChange = Math.toDegrees(dTheta);

        return Math.max(0.0, Math.min(1.0, 1.0 - (positionChange / 1.0 + thetaChange / 15.0)));
    }
}
