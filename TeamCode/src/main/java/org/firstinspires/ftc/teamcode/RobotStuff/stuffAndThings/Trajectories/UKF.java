package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories;

import org.ejml.simple.SimpleMatrix;

public class UKF {
    private SimpleMatrix state;
    private SimpleMatrix covariance;
    private final double alpha = 1e-3;
    private final double beta = 2;
    private final double kappa = 0;
    private final int n = 6; // State dimension
    private final double lambda = alpha * alpha * (n + kappa) - n;
    private final double[] weightsMean;
    private final double[] weightsCov;

    public UKF(double initialX, double initialY, double initialTheta) {
        state = new SimpleMatrix(new double[][]{{initialX}, {initialY}, {initialTheta}, {0}, {0}, {0}});
        covariance = SimpleMatrix.identity(n).scale(0.1);
        weightsMean = new double[2 * n + 1];
        weightsCov = new double[2 * n + 1];

        weightsMean[0] = lambda / (n + lambda);
        weightsCov[0] = lambda / (n + lambda) + (1 - alpha * alpha + beta);
        for (int i = 1; i < 2 * n + 1; i++) {
            weightsMean[i] = weightsCov[i] = 1.0 / (2 * (n + lambda));
        }
    }

    private SimpleMatrix computeSigmaPoints() {
        SimpleMatrix sqrtMatrix = covariance.plus(SimpleMatrix.identity(n).scale(1e-3)).svd().getU().scale(Math.sqrt(n + lambda));
        SimpleMatrix sigmaPoints = new SimpleMatrix(n, 2 * n + 1);

        sigmaPoints.setColumn(0, 0, state.getDDRM().getData());
        for (int i = 0; i < n; i++) {
            sigmaPoints.setColumn(i + 1, 0, state.plus(sqrtMatrix.extractVector(false, i)).getDDRM().getData());
            sigmaPoints.setColumn(i + 1 + n, 0, state.minus(sqrtMatrix.extractVector(false, i)).getDDRM().getData());
        }
        return sigmaPoints;
    }

    public void predict(double imuTheta, double imuOmega, double deltaT) {
        SimpleMatrix sigmaPoints = computeSigmaPoints();
        for (int i = 0; i < sigmaPoints.numCols(); i++) {
            double x = sigmaPoints.get(0, i);
            double y = sigmaPoints.get(1, i);
            double theta = imuTheta;
            double v_x = sigmaPoints.get(3, i);
            double v_y = sigmaPoints.get(4, i);
            double omega = imuOmega;

            x += v_x * deltaT;
            y += v_y * deltaT;
            theta += omega * deltaT;

            sigmaPoints.setColumn(i, 0, new double[]{x, y, theta, v_x, v_y, omega});
        }
        updateStateAndCovariance(sigmaPoints);
    }

    private void updateStateAndCovariance(SimpleMatrix sigmaPoints) {
        state.zero();
        for (int i = 0; i < sigmaPoints.numCols(); i++) {
            state = state.plus(sigmaPoints.extractVector(false, i).scale(weightsMean[i]));
        }
        covariance.zero();
        for (int i = 0; i < sigmaPoints.numCols(); i++) {
            SimpleMatrix diff = sigmaPoints.extractVector(false, i).minus(state);
            covariance = covariance.plus(diff.mult(diff.transpose()).scale(weightsCov[i]));
        }
    }

    public void update(double otosX, double otosY, double pinpointX, double pinpointY) {
        SimpleMatrix z = new SimpleMatrix(new double[][]{
                { pinpointX },
                { pinpointY },
                { otosX },
                { otosY }
        });
        SimpleMatrix H = new SimpleMatrix(new double[][]{
                { 1, 0, 0, 0, 0, 0 },
                { 0, 1, 0, 0, 0, 0 },
                { 1, 0, 0, 0, 0, 0 },
                { 0, 1, 0, 0, 0, 0 }
        });
        SimpleMatrix R = new SimpleMatrix(new double[][]{
                { 0.0001, 0, 0, 0 },
                { 0, 0.0001, 0, 0 },
                { 0, 0, 0.05, 0 },
                { 0, 0, 0, 0.05 }
        });
        SimpleMatrix S = H.mult(covariance).mult(H.transpose()).plus(R);
        SimpleMatrix K = covariance.mult(H.transpose()).mult(S.invert());
        SimpleMatrix y = z.minus(H.mult(state));
        state = state.plus(K.mult(y));
        SimpleMatrix I = SimpleMatrix.identity(state.numRows());
        covariance = (I.minus(K.mult(H))).mult(covariance);
    }

    public double getX() { return state.get(0); }
    public double getY() { return state.get(1); }
    public double getTheta() { return state.get(2); }
}
