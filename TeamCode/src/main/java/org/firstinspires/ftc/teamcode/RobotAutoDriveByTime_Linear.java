/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Spins up the shooter, runs timed feeder blocks, then drives forward. */

@Configurable
@Autonomous(name="Robot: Auto Shoot By Time", group="Robot")
public class RobotAutoDriveByTime_Linear extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    public static volatile double SPIN_UP_SECONDS = 4.0;
    public static volatile double FEED_SECONDS = 0.3;
    public static volatile double FEED_PAUSE_SECONDS = 1.0;
    public static volatile int FEED_BLOCKS = 4;
    public static volatile double SHOOTER_RPM = 3800.0;
    public static volatile double FORWARD_SECONDS = 3.0;
    public static volatile double FORWARD_POWER = 0.3;

    @Override
    public void runOpMode() {
        DcMotorEx pollFly = hardwareMap.get(DcMotorEx.class, "pollFly");
        Servo feeder = hardwareMap.get(Servo.class, "feeder");
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        pollFly.setDirection(DcMotorEx.Direction.REVERSE);
        // Match the drivetrain directions used by BasicDrive.
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        telemetry.addData("Status", "Ready to shoot");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        feeder.setPosition(BasicDriveServoFeeder.FEEDER_RETRACTED_POSITION);
        try {
            // BasicDrive's encoder conversion remains configurable in Panels.
            pollFly.setVelocity(SHOOTER_RPM * BasicDrive.MOTOR_TICKS_PER_REV / 60.0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < SPIN_UP_SECONDS) {
                telemetry.addData("Status", "Spinning up: %.1f s", runtime.seconds());
                telemetry.update();
                idle();
            }

            for (int block = 0; block < FEED_BLOCKS && opModeIsActive(); block++) {
                feeder.setPosition(BasicDriveServoFeeder.FEEDER_EXTENDED_POSITION);
                runtime.reset();
                while (opModeIsActive() && runtime.seconds() < FEED_SECONDS) {
                    telemetry.addData("Status", "Feeding: %.1f s", runtime.seconds());
                    telemetry.update();
                    idle();
                }
                feeder.setPosition(BasicDriveServoFeeder.FEEDER_RETRACTED_POSITION);

                if (block < FEED_BLOCKS - 1 && opModeIsActive()) {
                    runtime.reset();
                    while (opModeIsActive() && runtime.seconds() < FEED_PAUSE_SECONDS) {
                        telemetry.addData("Status", "Pausing feeder: %.1f s", runtime.seconds());
                        telemetry.update();
                        idle();
                    }
                }
            }

            // Finish shooting before starting the forward drive.
            feeder.setPosition(BasicDriveServoFeeder.FEEDER_RETRACTED_POSITION);
            pollFly.setVelocity(0);
            if (opModeIsActive()) {
                fl.setPower(FORWARD_POWER);
                bl.setPower(FORWARD_POWER);
                fr.setPower(FORWARD_POWER);
                br.setPower(FORWARD_POWER);
                runtime.reset();
                while (opModeIsActive() && runtime.seconds() < FORWARD_SECONDS) {
                    telemetry.addData("Status", "Driving forward: %.1f s", runtime.seconds());
                    telemetry.update();
                    idle();
                }
            }
        } finally {
            feeder.setPosition(BasicDriveServoFeeder.FEEDER_RETRACTED_POSITION);
            pollFly.setVelocity(0);
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        }

        telemetry.addData("Status", "Complete");
        telemetry.update();
    }
}
