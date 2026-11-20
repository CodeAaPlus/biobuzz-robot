package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Hardware and reusable Ivy commands for the current TeleOp mechanisms. */
public final class RobotSubsystems {
    public final Drive drive;
    public final Intake intake;
    public final Shooter shooter;
    public final Feeder feeder;
    public final Latch latch;
    public final Odometry odometry;

    public RobotSubsystems(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        feeder = new Feeder(hardwareMap);
        latch = new Latch(hardwareMap);
        odometry = new Odometry(hardwareMap);
    }

    /** Stop powered mechanisms when the OpMode exits. The position servo holds its last command. */
    public void stop() {
        drive.stop();
        intake.stop();
        shooter.stop();
        feeder.stop();
        odometry.stop();
    }

    public static final class Drive {
        private static final double NORMAL_SPEED = 1.0;
        private static final double SLOW_SPEED = 0.5;
        private final DcMotor fl, bl, fr, br;
        private double frontLeft, backLeft, frontRight, backRight;

        private Drive(HardwareMap map) {
            fl = map.get(DcMotor.class, "fl");
            bl = map.get(DcMotor.class, "bl");
            fr = map.get(DcMotor.class, "fr");
            br = map.get(DcMotor.class, "br");
            // Preserve the directions in BasicDrive; verify on the robot before changing.
            fl.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.REVERSE);
            fr.setDirection(DcMotor.Direction.FORWARD);
            br.setDirection(DcMotor.Direction.FORWARD);
        }

        public Command manual(Gamepad gamepad) {
            return Command.build()
                    .setExecute(() -> drive(-gamepad.left_stick_y, gamepad.left_stick_x,
                            gamepad.right_stick_x, gamepad.b))
                    .setEnd(reason -> stop())
                    .requiring(this);
        }

        public void drive(double axial, double lateral, double yaw, boolean slow) {
            double flPower = axial + lateral + yaw;
            double frPower = axial - lateral - yaw;
            double blPower = axial - lateral + yaw;
            double brPower = axial + lateral - yaw;
            double max = Math.max(1.0, Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                    Math.max(Math.abs(blPower), Math.abs(brPower))));
            double scale = (slow ? SLOW_SPEED : NORMAL_SPEED) / max;
            frontLeft = flPower * scale;
            frontRight = frPower * scale;
            backLeft = blPower * scale;
            backRight = brPower * scale;
            fl.setPower(frontLeft);
            fr.setPower(frontRight);
            bl.setPower(backLeft);
            br.setPower(backRight);
        }

        public double frontLeftPower() { return frontLeft; }
        public double frontRightPower() { return frontRight; }
        public double backLeftPower() { return backLeft; }
        public double backRightPower() { return backRight; }

        public void stop() {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            frontLeft = frontRight = backLeft = backRight = 0;
        }
    }

    public static final class Intake {
        private static final double SWEEPER_POWER = 0.75;
        private static final double ESOPHAGUS_POWER = 1.0;
        private final DcMotor sweeper, esophagus;

        private Intake(HardwareMap map) {
            sweeper = map.get(DcMotor.class, "sweeper");
            esophagus = map.get(DcMotor.class, "esophagus");
        }

        /** Runs until cancelled; interruption and release both stop the motors. */
        public Command run(int direction) {
            return Command.build()
                    .setStart(() -> setDirection(direction))
                    .setEnd(reason -> stop())
                    .requiring(this);
        }

        private void setDirection(int direction) {
            sweeper.setPower(direction * SWEEPER_POWER);
            esophagus.setPower(direction * ESOPHAGUS_POWER);
        }

        public void stop() { setDirection(0); }
    }

    public static final class Shooter {
        private static final double TARGET_RPM = 1500;
        private static final double FEED_READY_FRACTION = 0.90;
        // Existing encoder assumption; validate gearing before treating target as flywheel RPM.
        private static final double MOTOR_TICKS_PER_REV = 103.8;
        private final DcMotorEx motor;
        private boolean enabled;

        private Shooter(HardwareMap map) {
            motor = map.get(DcMotorEx.class, "pollFly");
            motor.setDirection(DcMotorEx.Direction.REVERSE);
        }

        public Command toggle() {
            return Command.build().setStart(() -> setEnabled(!enabled))
                    .setDone(() -> true).requiring(this);
        }

        public void setEnabled(boolean enabled) {
            this.enabled = enabled;
            motor.setVelocity((enabled ? TARGET_RPM : 0) * MOTOR_TICKS_PER_REV / 60.0);
        }

        public boolean enabled() { return enabled; }
        public double targetRpm() { return enabled ? TARGET_RPM : 0; }
        /** Encoder-derived RPM, using the same unverified ticks/revolution as the target. */
        public double measuredRpm() { return motor.getVelocity() * 60.0 / MOTOR_TICKS_PER_REV; }
        public double feedReadyRpm() { return TARGET_RPM * FEED_READY_FRACTION; }
        public boolean readyToFeed() { return enabled && measuredRpm() >= feedReadyRpm(); }
        public void stop() { setEnabled(false); }
    }

    public static final class Feeder {
        private static final double POWER = 1.0;
        private final CRServo motor;
        private double commandedPower;
        private boolean feedStarted;

        private Feeder(HardwareMap map) {
            motor = map.get(CRServo.class, "feeder");
            stop();
        }

        /** Wait for shooter speed before starting; keep feeding through shot-induced speed dips. */
        public Command run(Shooter shooter) {
            return Command.build()
                    .setStart(() -> { feedStarted = false; stop(); })
                    .setExecute(() -> {
                        if (!shooter.enabled()) {
                            feedStarted = false;
                            stop();
                        } else if (!feedStarted && shooter.readyToFeed()) {
                            feedStarted = true;
                            setPower(POWER);
                        }
                    })
                    .setEnd(reason -> { feedStarted = false; stop(); })
                    .requiring(this);
        }

        private void setPower(double power) {
            commandedPower = power;
            motor.setPower(power);
        }

        public double commandedPower() { return commandedPower; }
        public void stop() { setPower(0); }
    }

    /**
     * Two dead-wheel + IMU odometry localizer with straight-line auto primitives.
     *
     * <p>Shares motor handles with {@link Drive} only by name; both subsystems request
     * independent hardware objects from the HardwareMap — do not change motor directions
     * here without updating Drive as well.
     *
     * <p>Pod assignment (matches {@code TwoDeadWheelDrive} and Pedro calibration):
     * <ul>
     *   <li>Forward pod  → {@code br} encoder</li>
     *   <li>Strafe pod   → {@code fl} encoder</li>
     *   <li>Heading      → REV Control Hub IMU (logo DOWN, USB RIGHT)</li>
     * </ul>
     */
    public static final class Odometry {
        // Calibrated in TwoWheelTuner; do not edit without re-running the tuner.
        private static final double FORWARD_INCHES_PER_TICK  = 0.002973619489200135;
        private static final double SIDEWAYS_INCHES_PER_TICK = 0.002947316713741864;
        // Safety limits — movement stops if these are exceeded mid-move.
        private static final double MAX_TRANSLATION_POWER      = 0.25;
        private static final double MAX_CROSS_AXIS_DRIFT_INCHES = 3.0;
        private static final double MAX_HEADING_ERROR_DEGREES  = 12.0;
        // Heading-hold P controller used during translation.
        private static final double HEADING_KP              = 0.015;
        private static final double MAX_HEADING_CORRECTION  = 0.10;
        // Turn controller limits.
        private static final double MAX_TURN_POWER          = 0.20;
        private static final double TURN_TOLERANCE_DEGREES  = 3.0;

        private final DcMotorEx fl, bl, fr, br;
        private final IMU imu;

        private Odometry(HardwareMap map) {
            fl = map.get(DcMotorEx.class, "fl");
            bl = map.get(DcMotorEx.class, "bl");
            fr = map.get(DcMotorEx.class, "fr");
            br = map.get(DcMotorEx.class, "br");
            // Same directions as Drive — do not change without updating both subsystems.
            fl.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.REVERSE);
            fr.setDirection(DcMotor.Direction.FORWARD);
            br.setDirection(DcMotor.Direction.FORWARD);
            imu = map.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        }

        /** Raw tick count from the forward (parallel) pod on br. */
        public int forwardTicks()  { return br.getCurrentPosition(); }
        /** Raw tick count from the strafe (perpendicular) pod on fl. */
        public int sidewaysTicks() { return fl.getCurrentPosition(); }

        /** Current IMU yaw in degrees. Positive is counter-clockwise. */
        public double headingDegrees() {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        /** Signed shortest angle (degrees) from {@code current} to {@code target}. */
        public static double headingError(double target, double current) {
            double error = (target - current) % 360.0;
            if (error >  180.0) error -= 360.0;
            if (error < -180.0) error += 360.0;
            return error;
        }

        private static double clip(double value, double limit) {
            return Math.max(-limit, Math.min(limit, value));
        }

        /** Robot-relative mecanum drive. Same sign convention as {@link Drive#drive}. */
        private void drive(double forward, double sideways, double turn) {
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
            fl.setPower(0); bl.setPower(0);
            fr.setPower(0); br.setPower(0);
        }

        /**
         * Drive straight forward while holding the heading captured at the start of the call.
         *
         * @param opMode          the running LinearOpMode (used for {@code opModeIsActive} and telemetry)
         * @param inches          positive distance to travel
         * @param power           drive power — must be in (0, {@value #MAX_TRANSLATION_POWER}]
         * @param timeoutSeconds  abort after this many seconds
         * @return a short human-readable result string (e.g. "Distance reached", "Timed out")
         */
        public String driveForward(LinearOpMode opMode, double inches, double power,
                                   double timeoutSeconds) {
            if (!Double.isFinite(inches) || inches <= 0)
                throw new IllegalArgumentException("Forward distance must be positive and finite");
            return translate(opMode, inches, power, timeoutSeconds, false);
        }

        /**
         * Strafe right (positive) or left (negative) while holding the starting heading.
         *
         * @param opMode          the running LinearOpMode
         * @param inches          signed strafe distance
         * @param power           drive power — must be in (0, {@value #MAX_TRANSLATION_POWER}]
         * @param timeoutSeconds  abort after this many seconds
         * @return a short human-readable result string
         */
        public String strafe(LinearOpMode opMode, double inches, double power,
                             double timeoutSeconds) {
            if (!Double.isFinite(inches) || inches == 0)
                throw new IllegalArgumentException("Strafe distance must be nonzero and finite");
            return translate(opMode, inches, power, timeoutSeconds, true);
        }

        private String translate(LinearOpMode opMode, double inches, double power,
                                 double timeoutSeconds, boolean sideways) {
            if (!Double.isFinite(power) || power <= 0 || power > MAX_TRANSLATION_POWER ||
                    !Double.isFinite(timeoutSeconds) || timeoutSeconds <= 0)
                throw new IllegalArgumentException(
                        "Power must be 0.." + MAX_TRANSLATION_POWER + " and timeout must be positive");
            int    forwardStart  = forwardTicks();
            int    sidewaysStart = sidewaysTicks();
            double headingTarget = headingDegrees();
            ElapsedTime timer = new ElapsedTime();
            String result = "Stopped";
            try {
                while (opMode.opModeIsActive() && timer.seconds() < timeoutSeconds) {
                    double forwardIn  = Math.abs(forwardTicks()  - forwardStart)  * FORWARD_INCHES_PER_TICK;
                    double sidewaysIn = Math.abs(sidewaysTicks() - sidewaysStart) * SIDEWAYS_INCHES_PER_TICK;
                    double distance      = sideways ? sidewaysIn : forwardIn;
                    double crossAxisDrift = sideways ? forwardIn  : sidewaysIn;
                    double error = headingError(headingTarget, headingDegrees());
                    if (crossAxisDrift >= MAX_CROSS_AXIS_DRIFT_INCHES ||
                            Math.abs(error) >= MAX_HEADING_ERROR_DEGREES) {
                        result = "Stopped: drift limit"; break;
                    }
                    if (distance >= Math.abs(inches)) {
                        result = "Distance reached"; break;
                    }
                    double correction = clip(HEADING_KP * error, MAX_HEADING_CORRECTION);
                    drive(sideways ? 0 : power,
                          sideways ? Math.copySign(power, inches) : 0,
                          correction);
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

        /**
         * Turn in place relative to the current IMU heading.
         *
         * @param opMode          the running LinearOpMode
         * @param degrees         signed turn — positive is counter-clockwise, range ±180
         * @param timeoutSeconds  abort after this many seconds
         * @return a short human-readable result string
         */
        public String turn(LinearOpMode opMode, double degrees, double timeoutSeconds) {
            if (!Double.isFinite(degrees) || degrees == 0 || Math.abs(degrees) > 180 ||
                    !Double.isFinite(timeoutSeconds) || timeoutSeconds <= 0)
                throw new IllegalArgumentException(
                        "Turn must be within ±180 degrees; timeout must be positive");
            double target = headingDegrees() + degrees;
            ElapsedTime timer = new ElapsedTime();
            String result = "Stopped";
            try {
                while (opMode.opModeIsActive() && timer.seconds() < timeoutSeconds) {
                    double error = headingError(target, headingDegrees());
                    if (Math.abs(error) <= TURN_TOLERANCE_DEGREES) {
                        result = "Heading reached"; break;
                    }
                    if (Math.abs(error) > Math.abs(degrees) + 10.0) {
                        result = "Stopped: heading moved wrong way"; break;
                    }
                    double power = Math.copySign(
                            Math.max(0.07, Math.min(MAX_TURN_POWER, 0.012 * Math.abs(error))), error);
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

    public static final class Latch {
        private static final double UP_POSITION = 0.730;
        private static final double DOWN_POSITION = 0.390;
        private final Servo servo;

        private Latch(HardwareMap map) {
            servo = map.get(Servo.class, "latch");
            // BasicDrive leaves the initial position unchanged; preserve that behavior.
        }

        public Command up() { return moveTo(UP_POSITION); }
        public Command down() { return moveTo(DOWN_POSITION); }

        private Command moveTo(double position) {
            return Command.build().setStart(() -> servo.setPosition(position))
                    .setDone(() -> true).requiring(this);
        }

        public double position() { return servo.getPosition(); }
    }
}
