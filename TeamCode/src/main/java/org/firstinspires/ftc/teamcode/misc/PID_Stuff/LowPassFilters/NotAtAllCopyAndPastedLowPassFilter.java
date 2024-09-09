package org.firstinspires.ftc.teamcode.misc.PID_Stuff.LowPassFilters;

public class NotAtAllCopyAndPastedLowPassFilter {
    int order = 2;
    double[] a = new double[order];
    double[] b = new double[order + 1];
    double omega0;
    double dt;
    boolean adapt;
    double tn1 = 0;
    double[] x = new double[order + 1]; // Raw values
    double[] y = new double[order + 1]; // Filtered values

    public NotAtAllCopyAndPastedLowPassFilter(int order, double cutoffFrequency, double estimatedSampleFrequency, boolean adaptive, double deltaTime) {
        // cutoffFrequency: cutoff frequency (Hz)
        // estimatedSampleFrequency: sample frequency (Hz)
        // adaptive: boolean flag, if set to 1, the code will automatically set
        // the sample frequency based on the time history.
        this.order = order;

        omega0 = 6.28318530718 * cutoffFrequency;
        dt = 1.0 / estimatedSampleFrequency;
        adapt = adaptive;
        tn1 = -dt;
        for (int k = 0; k < order + 1; k++) {
            x[k] = 0;
            y[k] = 0;
        }
        setCoef(deltaTime);
    }

    void setCoef(double deltaTime) {
        if (adapt) {
            double t = deltaTime;
            dt = t - tn1;
            tn1 = t;
        }

        double alpha = omega0 * dt;
        if (order == 1) {
            a[0] = -(alpha - 2.0) / (alpha + 2.0);
            b[0] = alpha / (alpha + 2.0);
            b[1] = alpha / (alpha + 2.0);
        }
        if (order == 2) {
            double alphaSq = alpha * alpha;
            double[] beta = {1, Math.sqrt(2), 1};
            double D = alphaSq * beta[0] + 2 * alpha * beta[1] + 4 * beta[2];
            b[0] = alphaSq / D;
            b[1] = 2 * b[0];
            b[2] = b[0];
            a[0] = -(2 * alphaSq * beta[0] - 8 * beta[2]) / D;
            a[1] = -(beta[0] * alphaSq - 2 * beta[1] * alpha + 4 * beta[2]) / D;
        }
    }

    public double filter(double xn, double runtime) {
        // Provide me with the current raw value: x
        // I will give you the current filtered value: y
        if (adapt) {
            setCoef(runtime); // Update coefficients if necessary
        }
        y[0] = 0;
        x[0] = xn;
        // Compute the filtered values
        for (int k = 0; k < order; k++) {
            y[0] += a[k] * y[k + 1] + b[k] * x[k];
        }
        y[0] += b[order] * x[order];

        // Save the historical values
        for (int k = order; k > 0; k--) {
            y[k] = y[k - 1];
            x[k] = x[k - 1];
        }

        // Return the filtered value    
        return y[0];
    }
}

