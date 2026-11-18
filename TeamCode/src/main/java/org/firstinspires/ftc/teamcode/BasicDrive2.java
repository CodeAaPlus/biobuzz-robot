/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Robot-relative mecanum TeleOp: stick directions are relative to the robot,
 * not the field. No heading correction or autonomous path following is used.
 *
 * Gamepad 1: left stick drives/strafe, right stick X turns; hold B for half speed.
 * Gamepad 2: hold A to run the intake/feed motors forward, B to reverse them.
 * Each right-trigger press toggles the shooter velocity target on or off.
 * Hold gamepad 2 right bumper to run the feeder at full power; release to stop.
 * D-pad up/down selects the two latch positions; releasing keeps the last command.
 */

@Configurable
@TeleOp(name="test", group="Linear OpMode")
public class BasicDrive2 extends LinearOpMode {

    // Match timer, drivetrain motors (front/back, left/right), and mechanisms.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor bl = null;
    private DcMotor fr = null;
    private DcMotor br = null;
    private DcMotor sweeper = null;
    private Servo latch = null;
    private CRServo feeder = null;
    private DcMotor esophagus = null;
    private DcMotorEx pollFly = null;
    // Requested shooter state and previous trigger sample for press detection.
    private boolean toggleOutake = false;
    private boolean previousShooterTriggerPressed = false;
    private double speed;
    // Driver speed multipliers and mechanism power magnitudes.
    public static volatile double DRIVE_SPEED = 1.0;
    public static volatile double SLOW_DRIVE_SPEED = 0.5;
    public static volatile double SWEEPER_POWER = 1.0;
    public static volatile double ESOPHAGUS_POWER = 1.0;
    public static volatile double FEEDER_POWER = 1.0;
    // Velocity target; actual speed depends on the encoder conversion and hardware.
    public static volatile double SHOOTER_RPM = 4000;
    // Named for the controls until the physical latch positions are confirmed.
    public static volatile double LATCH_DPAD_UP_POSITION = 0.730;
    public static volatile double LATCH_DPAD_DOWN_POSITION = 0.390;
    // Assumed encoder ticks per revolution of the shaft whose RPM is commanded.
    // Verify the motor encoder and gearing before interpreting this as flywheel RPM.
    public static volatile double MOTOR_TICKS_PER_REV = 28;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        br = hardwareMap.get(DcMotor.class, "br");

        esophagus = hardwareMap.get(DcMotor.class, "esophagus");
        pollFly = hardwareMap.get(DcMotorEx.class, "pollFly");
        sweeper = hardwareMap.get(DcMotor.class, "sweeper");
        latch = hardwareMap.get(Servo.class, "latch");
        feeder = hardwareMap.get(CRServo.class, "feeder");
        // Continuous-rotation servos use power (-1 to 1), not position.
        feeder.setPower(0);
        speed = DRIVE_SPEED;
        // Current hardware convention: reverse the left drive motors and shooter.
        // Validate directions physically before changing this configuration.
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        pollFly.setDirection(DcMotorEx.Direction.REVERSE);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Robot-relative requests: forward/back, strafe, and turn.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Mix translation and rotation into the four mecanum wheel commands.

            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Scale all wheels equally when any magnitude exceeds 1.0.
            // This preserves command ratios; it does not guarantee physical tracking.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Apply the speed multiplier after normalization to preserve wheel ratios.
            boolean slowDrive = gamepad1.b;
            if (slowDrive) {
                speed = SLOW_DRIVE_SPEED;
            } else {
                speed = DRIVE_SPEED;
            }
            // Send calculated power to wheels
            fl.setPower(frontLeftPower * speed);
            bl.setPower(backLeftPower * speed);
            br.setPower(backRightPower * speed);
            fr.setPower(frontRightPower * speed);
            // Intake/feed are hold-to-run; A takes priority if both buttons are held.
            if (gamepad2.a) {
                sweeper.setPower(SWEEPER_POWER);
                esophagus.setPower(ESOPHAGUS_POWER);
            } else if (gamepad2.b){
                sweeper.setPower(-SWEEPER_POWER);
                esophagus.setPower(-ESOPHAGUS_POWER);
            } else {
                sweeper.setPower(0);
                esophagus.setPower(0);
            }
            // Toggle on a released-to-pressed transition, using the SDK trigger threshold.
            // A trigger already held at START also counts as the first press.
            boolean shooterTriggerPressed = gamepad2.right_trigger_pressed;
            if (shooterTriggerPressed && !previousShooterTriggerPressed) {
                outake(SHOOTER_RPM);
            }
            previousShooterTriggerPressed = shooterTriggerPressed;
            // Keep an enabled shooter in sync with live Panels RPM and encoder edits.
            if (toggleOutake) {
                setShooterRpm(SHOOTER_RPM);
            }

            // Hold-to-run feeder control, independent of the shooter toggle.
            double feederPower = gamepad2.right_bumper ? FEEDER_POWER : 0.0;
            feeder.setPower(feederPower);

            // No initial latch position is commanded by this OpMode.
            // Up wins if both directions are pressed; neither sends a new command.
            if (gamepad2.dpad_up) {
                latch.setPosition(LATCH_DPAD_UP_POSITION);
            } else if (gamepad2.dpad_down) {
                latch.setPosition(LATCH_DPAD_DOWN_POSITION);
            }
            // Show commanded outputs, not measured wheel speeds or shooter RPM.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive mode", slowDrive ? "Slow" : "Normal");
            telemetry.addData("Front power L/R", "%4.2f, %4.2f", frontLeftPower * speed, frontRightPower * speed);
            telemetry.addData("Back power L/R", "%4.2f, %4.2f", backLeftPower * speed, backRightPower * speed);
            telemetry.addData("Shooter", toggleOutake ? "Enabled" : "Disabled");
            telemetry.addData("Shooter target RPM", "%.0f", toggleOutake ? SHOOTER_RPM : 0.0);
            telemetry.addData("Latch commanded position", "%.3f", latch.getPosition());
            telemetry.addData("Feeder commanded power", "%.1f", feederPower);
            telemetry.update();
        }
    }
    /** Commands encoder velocity in ticks/second; zero requests a stopped shooter. */
    protected void setShooterRpm(double rpm) {
        double tps = rpmToTicksPerSec(rpm);
        pollFly.setVelocity(tps);
    }
    // Convert revolutions/minute to encoder ticks/second using the assumed ratio.
    private double rpmToTicksPerSec(double rpm) {
        return (rpm * MOTOR_TICKS_PER_REV) / 60.0;
    }
    // Issue the next velocity target, then record that requested on/off state.
    protected void outake(double rpm) {
        setShooterRpm(toggleOutake ? 0 : rpm);
        toggleOutake = !toggleOutake;
    }
}
