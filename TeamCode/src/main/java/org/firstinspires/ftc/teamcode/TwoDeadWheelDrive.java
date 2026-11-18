package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Hardware and reusable movements for the standalone two dead-wheel auto. */
public final class TwoDeadWheelDrive {
    // Copied from the current pod calibration; changing Pedro's calibration is not required.
    private static final double FORWARD_INCHES_PER_TICK = 0.002973619489200135;
    private static final double SIDEWAYS_INCHES_PER_TICK = 0.002947316713741864;
    private static final double MAX_TRANSLATION_POWER = 0.25;
    private static final double MAX_CROSS_AXIS_DRIFT_INCHES = 3.0;
    private static final double MAX_HEADING_ERROR_DEGREES = 12.0;
    private static final double HEADING_KP = 0.015;
    private static final double MAX_HEADING_CORRECTION = 0.10;
    private static final double MAX_TURN_POWER = 0.20;
    private static final double TURN_TOLERANCE_DEGREES = 3.0;
    private final DcMotorEx fl, bl, fr, br;
    private final IMU imu;

    public TwoDeadWheelDrive(HardwareMap map) {
        fl = map.get(DcMotorEx.class, "fl");
        bl = map.get(DcMotorEx.class, "bl");
        fr = map.get(DcMotorEx.class, "fr");
        br = map.get(DcMotorEx.class, "br");
        imu = map.get(IMU.class, "imu");

        // Same motor directions as BasicDrive. Do not reset shared encoder ports.
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
    }

    /** Parallel pod on br; read relative to a caller-saved baseline. */
    public int forwardTicks() { return br.getCurrentPosition(); }

    /** Perpendicular pod on fl; read relative to a caller-saved baseline. */
    public int sidewaysTicks() { return fl.getCurrentPosition(); }

    public double headingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /** Signed shortest angle from current heading to target, in degrees. */
    public static double headingError(double target, double current) {
        double error = (target - current) % 360.0;
        if (error > 180.0) error -= 360.0;
        if (error < -180.0) error += 360.0;
        return error;
    }

    public static double clip(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }

    /** Robot-relative mecanum powers, using the same signs as BasicDrive. */
    public void drive(double forward, double sideways, double turn) {
        double flPower = forward + sideways + turn;
        double frPower = forward - sideways - turn;
        double blPower = forward - sideways + turn;
        double brPower = forward + sideways - turn;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower))));
        fl.setPower(flPower / max);
        fr.setPower(frPower / max);
        bl.setPower(blPower / max);
        br.setPower(brPower / max);
    }

    public void stop() {
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

    /** Drive forward while holding the heading measured at the start of this call. */
    public String driveForward(LinearOpMode opMode, double inches, double power, double timeoutSeconds) {
        if (!Double.isFinite(inches) || inches <= 0) {
            throw new IllegalArgumentException("Forward distance must be positive and finite");
        }
        return translate(opMode, inches, power, timeoutSeconds, false);
    }

    /** Strafe right for positive inches or left for negative inches. */
    public String strafe(LinearOpMode opMode, double inches, double power, double timeoutSeconds) {
        if (!Double.isFinite(inches) || inches == 0) {
            throw new IllegalArgumentException("Strafe distance must be nonzero and finite");
        }
        return translate(opMode, inches, power, timeoutSeconds, true);
    }

    private String translate(LinearOpMode opMode, double inches, double power,
                             double timeoutSeconds, boolean sideways) {
        if (!Double.isFinite(power) || power <= 0 || power > MAX_TRANSLATION_POWER ||
                !Double.isFinite(timeoutSeconds) || timeoutSeconds <= 0) {
            throw new IllegalArgumentException("Power must be 0..0.25 and timeout must be positive");
        }
        int forwardStart = forwardTicks();
        int sidewaysStart = sidewaysTicks();
        double headingTarget = headingDegrees();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        String result = "Stopped";
        try {
            while (opMode.opModeIsActive() && timer.seconds() < timeoutSeconds) {
                double forwardInches = Math.abs(forwardTicks() - forwardStart) * FORWARD_INCHES_PER_TICK;
                double sidewaysInches = Math.abs(sidewaysTicks() - sidewaysStart) * SIDEWAYS_INCHES_PER_TICK;
                double distance = sideways ? sidewaysInches : forwardInches;
                double crossAxisDrift = sideways ? forwardInches : sidewaysInches;
                double error = headingError(headingTarget, headingDegrees());
                if (crossAxisDrift >= MAX_CROSS_AXIS_DRIFT_INCHES ||
                        Math.abs(error) >= MAX_HEADING_ERROR_DEGREES) {
                    result = "Stopped: drift limit";
                    break;
                }
                if (distance >= Math.abs(inches)) {
                    result = "Distance reached";
                    break;
                }
                double correction = clip(HEADING_KP * error, MAX_HEADING_CORRECTION);
                drive(sideways ? 0 : power, sideways ? Math.copySign(power, inches) : 0, correction);
                opMode.telemetry.addData(sideways ? "Sideways (in)" : "Forward (in)",
                        "%.2f / %.2f", distance, Math.abs(inches));
                opMode.telemetry.addData("Cross-axis drift (in)", "%.2f", crossAxisDrift);
                opMode.telemetry.addData("Heading error (deg)", "%.1f", error);
                opMode.telemetry.update();
                opMode.idle();
            }
            if (opMode.opModeIsActive() && timer.seconds() >= timeoutSeconds) result = "Timed out";
        } finally {
            stop();
        }
        return result;
    }

    /** Turn relative to the current IMU heading; positive degrees is counterclockwise. */
    public String turn(LinearOpMode opMode, double degrees, double timeoutSeconds) {
        if (!Double.isFinite(degrees) || degrees == 0 || Math.abs(degrees) > 180 ||
                !Double.isFinite(timeoutSeconds) || timeoutSeconds <= 0) {
            throw new IllegalArgumentException("Turn must be within +/-180 degrees; timeout must be positive");
        }
        double target = headingDegrees() + degrees;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        String result = "Stopped";
        try {
            while (opMode.opModeIsActive() && timer.seconds() < timeoutSeconds) {
                double error = headingError(target, headingDegrees());
                if (Math.abs(error) <= TURN_TOLERANCE_DEGREES) {
                    result = "Heading reached";
                    break;
                }
                if (Math.abs(error) > Math.abs(degrees) + 10.0) {
                    result = "Stopped: heading moved wrong way";
                    break;
                }
                double power = Math.copySign(Math.max(0.07,
                        Math.min(MAX_TURN_POWER, 0.012 * Math.abs(error))), error);
                drive(0, 0, power);
                opMode.telemetry.addData("Turn error (deg)", "%.1f", error);
                opMode.telemetry.addData("Turn power", "%.2f", power);
                opMode.telemetry.update();
                opMode.idle();
            }
            if (opMode.opModeIsActive() && timer.seconds() >= timeoutSeconds) result = "Timed out";
        } finally {
            stop();
        }
        return result;
    }
}
