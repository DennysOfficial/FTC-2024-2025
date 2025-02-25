package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.CholeskyDecomposition_F64;
import org.ejml.simple.SimpleMatrix;

public class UKF_Localization {
    private static final int STATE_DIM = 5;  // [x, y, θ, v, ω]
    private static final int SIGMA_POINT_COUNT = 2 * STATE_DIM + 1;

    private SimpleMatrix state;  // State vector [x, y, θ, v, ω]
    private SimpleMatrix covariance;  // Covariance matrix
    private SimpleMatrix processNoise;  // Process noise matrix
    private SimpleMatrix measurementNoise;  // Noise for OTOS & Pinpoint

    private double lambda;
    private SimpleMatrix sigmaPoints;

    public UKF_Localization() {
        state = new SimpleMatrix(STATE_DIM, 1);
        covariance = SimpleMatrix.identity(STATE_DIM).scale(0.1);
        processNoise = SimpleMatrix.identity(STATE_DIM).scale(0.05);

        measurementNoise = new SimpleMatrix(new double[][]{
                {0.002, 0},
                {0, 0.00009}
        });

        lambda = 3 - STATE_DIM;
    }

    private void generateSigmaPoints() {
        CholeskyDecomposition_F64<DMatrixRMaj> cholesky = DecompositionFactory_DDRM.chol(STATE_DIM, true);

        if (!cholesky.decompose(covariance.getMatrix())) {
            throw new RuntimeException("Cholesky Decomposition failed!");
        }

        SimpleMatrix sqrtP = SimpleMatrix.wrap(cholesky.getT(null)).scale(Math.sqrt(lambda + STATE_DIM));
        sigmaPoints = new SimpleMatrix(STATE_DIM, SIGMA_POINT_COUNT);
        sigmaPoints.insertIntoThis(0, 0, state);

        for (int i = 0; i < STATE_DIM; i++) {
            SimpleMatrix col = sqrtP.extractVector(false, i);
            sigmaPoints.insertIntoThis(0, i + 1, state.plus(col));
            sigmaPoints.insertIntoThis(0, i + 1 + STATE_DIM, state.minus(col));
        }
    }


    public void predict(double dt, double imuTheta, double imuOmega) {
        generateSigmaPoints();

        for (int i = 0; i < SIGMA_POINT_COUNT; i++) {
            double x = sigmaPoints.get(0, i);
            double y = sigmaPoints.get(1, i);
            double theta = sigmaPoints.get(2, i);
            double v = sigmaPoints.get(3, i);
            double omega = sigmaPoints.get(4, i);

            // Update theta using IMU if available
            theta = imuTheta;
            omega = imuOmega;

            if (Math.abs(omega) > 1e-5) {
                x += (v / omega) * (Math.sin(theta + omega * dt) - Math.sin(theta));
                y += (v / omega) * (Math.cos(theta) - Math.cos(theta + omega * dt));
            } else {
                x += v * Math.cos(theta) * dt;
                y += v * Math.sin(theta) * dt;
            }
            theta += omega * dt;

            sigmaPoints.set(0, i, x);
            sigmaPoints.set(1, i, y);
            sigmaPoints.set(2, i, theta);
        }

        computeMeanAndCovariance();
    }


    private void computeMeanAndCovariance() {
        state.zero();
        for (int i = 0; i < SIGMA_POINT_COUNT; i++) {
            double weight = (i == 0) ? lambda / (lambda + STATE_DIM) : 1 / (2 * (lambda + STATE_DIM));
            state = state.plus(sigmaPoints.extractVector(false, i).scale(weight));
        }

        covariance.zero();
        for (int i = 0; i < SIGMA_POINT_COUNT; i++) {
            double weight = (i == 0) ? lambda / (lambda + STATE_DIM) : 1 / (2 * (lambda + STATE_DIM));
            SimpleMatrix diff = sigmaPoints.extractVector(false, i).minus(state);
            covariance = covariance.plus(diff.mult(diff.transpose()).scale(weight));
        }

        covariance = covariance.plus(processNoise);
    }

    public void update(double otosX, double otosY, double pinX, double pinY) {
        SimpleMatrix measurement = new SimpleMatrix(new double[][]{
                {otosX}, {otosY}, {pinX}, {pinY}
        });

        SimpleMatrix measurementSigma = sigmaPoints.extractMatrix(0, 2, 0, SIGMA_POINT_COUNT);
        SimpleMatrix measurementMean = measurementSigma.extractVector(false, 0);

        SimpleMatrix innovationCov = measurementNoise.plus(measurementSigma.mult(measurementSigma.transpose()));
        SimpleMatrix kalmanGain = covariance.mult(innovationCov.invert());

        SimpleMatrix innovation = measurement.minus(measurementMean);
        state = state.plus(kalmanGain.mult(innovation));
        covariance = covariance.minus(kalmanGain.mult(innovationCov).mult(kalmanGain.transpose()));
    }

    public double getX() { return state.get(0, 0); }
    public double getY() { return state.get(1, 0); }
    public double getTheta() { return state.get(2, 0); }
}
