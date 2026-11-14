package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;

/** Live diagnostic controls only; copy confirmed values into Constants to save them. */
@Configurable
public class BrakeTuning {
    public static volatile double forwardLinearBrake = 0.10;
    public static volatile double forwardQuadraticBrake = 0.0;
    public static volatile double headingKP = 2.5;
    // Requested path speed in inches/second, not a motor-power limit.
    public static volatile double maxVelocity = 25.0;
    // Requested slowdown profile, inches/second squared. Lower means gentler.
    public static volatile double maxDeceleration = 20.0;
    // Captured on the next A press; never moves endpoints during an active run.
    public static volatile double pathDistance = 48.0;

    @IgnoreConfigurable
    private static double acceptedDistance = 48.0;

    @IgnoreConfigurable
    private static double appliedHeadingKP = Double.NaN;
    @IgnoreConfigurable
    private static Controller appliedHeadingController;

    @IgnoreConfigurable
    private static String status = "Not applied";

    public static String status() {
        return status;
    }

    public static double appliedLinear() {
        return Constants.foresightConfig.linearBrakeCoefficients.get().get(0, 0);
    }

    public static double appliedQuadratic() {
        return Constants.foresightConfig.quadraticBrakeCoefficients.get().get(0, 0);
    }

    public static double appliedMaxVelocity() {
        return Constants.foresightConfig.maxVelocityConstraint.get();
    }

    public static double appliedMaxDeceleration() {
        return Constants.foresightConfig.maxDecelerationConstraint.get();
    }

    public static double nextPathDistance() {
        return acceptedDistance;
    }

    public static double appliedHeadingKP() {
        return appliedHeadingKP;
    }

    /** Call on the OpMode thread before follower.update(). */
    public static void apply() {
        double linear = forwardLinearBrake;
        double quadratic = forwardQuadraticBrake;
        double velocity = maxVelocity;
        double deceleration = maxDeceleration;
        double distance = pathDistance;
        double heading = headingKP;
        if (!Double.isFinite(heading) || heading < 0) {
            status = "Rejected: headingKP must be finite and nonnegative; previous values active";
            return;
        }
        if (!Double.isFinite(linear) || linear < 0
                || !Double.isFinite(quadratic) || quadratic < 0) {
            status = "Rejected: use finite nonnegative values; previous values active";
            return;
        }
        if (!Double.isFinite(velocity) || velocity <= 0) {
            status = "Rejected: maxVelocity must be finite and positive; previous values active";
            return;
        }
        if (!Double.isFinite(deceleration) || deceleration <= 0
                || !Double.isFinite(distance) || distance <= 0) {
            status = "Rejected: deceleration and distance must be finite and positive; previous values active";
            return;
        }
        Matrix oldLinear = Constants.foresightConfig.linearBrakeCoefficients.get();
        Matrix oldQuadratic = Constants.foresightConfig.quadraticBrakeCoefficients.get();
        if (linear != oldLinear.get(0, 0)) {
            Constants.foresightConfig.linearBrakeCoefficients.set(withForward(oldLinear, linear));
        }
        if (quadratic != oldQuadratic.get(0, 0)) {
            Constants.foresightConfig.quadraticBrakeCoefficients.set(withForward(oldQuadratic, quadratic));
        }
        Constants.foresightConfig.maxVelocityConstraint.set(velocity);
        Constants.foresightConfig.maxDecelerationConstraint.set(deceleration);
        if (heading != appliedHeadingKP
                || Constants.foresightConfig.headingFeedback.get() != appliedHeadingController) {
            appliedHeadingController = Controller.proportional(heading);
            Constants.foresightConfig.headingFeedback.set(appliedHeadingController);
            appliedHeadingKP = heading;
        }
        acceptedDistance = distance;
        status = "Applied";
    }

    private static Matrix withForward(Matrix old, double forward) {
        return new Matrix(new double[][] {
                {forward, old.get(0, 1)},
                {old.get(1, 0), old.get(1, 1)}
        });
    }
}
