package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

import org.ejml.simple.SimpleMatrix;

public class EKF {
    private SimpleMatrix x;  // State [x, y, theta]
    private SimpleMatrix P;  // Covariance matrix
    private double lastUpdateTime = -1;

    public EKF() {
        x = new SimpleMatrix(3, 1);  // [x, y, theta]
        P = SimpleMatrix.identity(3).scale(0.1);
    }

    public void predict(double vx, double vy, double omega, double deltaTime) {
        if (deltaTime <= 0) return; // Skip if time is invalid

        // State transition model (motion model)
        double theta = x.get(2);
        double dx = vx * Math.cos(theta) * deltaTime - vy * Math.sin(theta) * deltaTime;
        double dy = vx * Math.sin(theta) * deltaTime + vy * Math.cos(theta) * deltaTime;
        double dTheta = omega * deltaTime;

        x.set(0, x.get(0) + dx);
        x.set(1, x.get(1) + dy);
        x.set(2, x.get(2) + dTheta);

        // Jacobian of motion model
        SimpleMatrix F = new SimpleMatrix(3, 3, true, new double[]{
                1, 0, -vx * Math.sin(theta) * deltaTime - vy * Math.cos(theta) * deltaTime,
                0, 1,  vx * Math.cos(theta) * deltaTime - vy * Math.sin(theta) * deltaTime,
                0, 0, 1
        });

        // Process noise
        SimpleMatrix Q = SimpleMatrix.diag(0.01, 0.01, 0.001).scale(deltaTime);

        // Update covariance
        P = F.mult(P).mult(F.transpose()).plus(Q);
    }

    public void updateWithAprilTag(double tagX, double tagY, double tagTheta, double confidence) {
        double alpha = 0.2 + 0.5 * confidence;  // Adaptive filtering

        // Low-pass filter
        double filteredX = alpha * tagX + (1 - alpha) * x.get(0);
        double filteredY = alpha * tagY + (1 - alpha) * x.get(1);
        double filteredTheta = alpha * tagTheta + (1 - alpha) * x.get(2);

        SimpleMatrix z = new SimpleMatrix(3, 1, true, new double[]{filteredX, filteredY, filteredTheta});
        SimpleMatrix H = SimpleMatrix.identity(3);
        SimpleMatrix R = SimpleMatrix.diag(0.2, 0.2, 0.1);

        SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);
        SimpleMatrix K = P.mult(H.transpose()).mult(S.invert());

        x = x.plus(K.mult(z.minus(H.mult(x))));
        P = (SimpleMatrix.identity(3).minus(K.mult(H))).mult(P);
    }

    public void updateWithIMU(double imuTheta) {
        SimpleMatrix z = new SimpleMatrix(1, 1, true, new double[]{imuTheta});
        SimpleMatrix H = new SimpleMatrix(1, 3, true, new double[]{0, 0, 1});
        SimpleMatrix R = new SimpleMatrix(1, 1, true, new double[]{0.05});

        SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);
        SimpleMatrix K = P.mult(H.transpose()).mult(S.invert());

        x = x.plus(K.mult(z.minus(H.mult(x))));
        P = (SimpleMatrix.identity(3).minus(K.mult(H))).mult(P);
    }

    public double[] getState() {
        return new double[]{x.get(0), x.get(1), x.get(2)};
    }
}
