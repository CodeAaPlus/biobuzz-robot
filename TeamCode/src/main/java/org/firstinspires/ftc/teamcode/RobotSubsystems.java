package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Hardware and reusable Ivy commands for the current TeleOp mechanisms. */
public final class RobotSubsystems {
    public final Drive drive;
    public final Intake intake;
    public final Shooter shooter;
    public final Feeder feeder;
    public final Latch latch;

    public RobotSubsystems(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        feeder = new Feeder(hardwareMap);
        latch = new Latch(hardwareMap);
    }

    /** Stop powered mechanisms when the OpMode exits. The position servo holds its last command. */
    public void stop() {
        drive.stop();
        intake.stop();
        shooter.stop();
        feeder.stop();
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
