package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Ivy TeleOp with position-servo feeder toggled by right bumper. BasicDrive remains the manual reference. */
@TeleOp(name = "Command Drive", group = "Linear OpMode")
public class CommandDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Ivy's scheduler is static, so clear commands left by another OpMode.
        Scheduler.reset();
        RobotSubsystems robot = new RobotSubsystems(hardwareMap);
        Command manualDrive = robot.drive.manual(gamepad1);
        Command intakeForward = robot.intake.run(1);
        Command intakeReverse = robot.intake.run(-1);

        ElapsedTime runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) {
            robot.stop();
            return;
        }

        runtime.reset();
        Scheduler.schedule(manualDrive);
        boolean previousTrigger = false;
        int previousLatchDirection = 0;
        int previousIntakeDirection = 0;
        boolean previousFeederBumper = false;
        try {
            while (opModeIsActive()) {
                // A takes priority when both intake buttons are held.
                int intakeDirection = gamepad2.a ? 1 : gamepad2.b ? -1 : 0;
                if (intakeDirection != previousIntakeDirection) {
                    if (previousIntakeDirection == 1) Scheduler.cancel(intakeForward);
                    if (previousIntakeDirection == -1) Scheduler.cancel(intakeReverse);
                    if (intakeDirection == 1) Scheduler.schedule(intakeForward);
                    if (intakeDirection == -1) Scheduler.schedule(intakeReverse);
                    previousIntakeDirection = intakeDirection;
                }

                // Toggle feeder position on each right-bumper press (same as GameTeleop).
                boolean feederBumper = gamepad2.right_bumper;
                if (feederBumper && !previousFeederBumper) Scheduler.schedule(robot.feeder.toggle());
                previousFeederBumper = feederBumper;

                boolean trigger = gamepad2.right_trigger_pressed;
                if (trigger && !previousTrigger) Scheduler.schedule(robot.shooter.toggle());
                previousTrigger = trigger;

                // Up takes priority; a newly selected direction commands the servo once.
                int latchDirection = gamepad2.dpad_up ? 1 : gamepad2.dpad_down ? -1 : 0;
                if (latchDirection != previousLatchDirection) {
                    if (latchDirection == 1) Scheduler.schedule(robot.latch.up());
                    if (latchDirection == -1) Scheduler.schedule(robot.latch.down());
                    previousLatchDirection = latchDirection;
                }

                Scheduler.execute();
                telemetry.addData("Status", "Run Time: " + runtime);
                telemetry.addData("Drive mode", gamepad1.b ? "Slow" : "Normal");
                telemetry.addData("Front power L/R", "%4.2f, %4.2f",
                        robot.drive.frontLeftPower(), robot.drive.frontRightPower());
                telemetry.addData("Back power L/R", "%4.2f, %4.2f",
                        robot.drive.backLeftPower(), robot.drive.backRightPower());
                telemetry.addData("Shooter", robot.shooter.enabled() ? "Enabled" : "Disabled");
                telemetry.addData("Shooter target RPM", "%.0f", robot.shooter.targetRpm());
                telemetry.addData("Shooter measured RPM", "%.0f", robot.shooter.measuredRpm());
                telemetry.addData("Feed ready at RPM", "%.0f", robot.shooter.feedReadyRpm());
                telemetry.addData("Feed ready", robot.shooter.readyToFeed() ? "Yes" : "No");
                telemetry.addData("Latch commanded position", "%.3f", robot.latch.position());
                telemetry.addData("Feeder position", robot.feeder.isExtended() ? "Extended" : "Retracted");
                telemetry.update();
            }
        } finally {
            Scheduler.reset();
            robot.stop();
        }
    }
}
