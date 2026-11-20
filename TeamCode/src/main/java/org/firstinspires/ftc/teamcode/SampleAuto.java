package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.RobotSubsystems;

/**
 * Sample autonomous OpMode using RobotSubsystems.
 *
 * Routine:
 *   1. Spin up the shooter and wait for it to reach speed.
 *   2. Run the feeder for FEED_SECONDS to shoot.
 *   3. Stop shooter and feeder.
 *   4. Drive forward 24 inches (IMU holds heading).
 *   5. Turn 90 degrees counter-clockwise.
 *   6. Strafe right 12 inches.
 *
 * Logo DOWN, USB RIGHT. Clear the area and keep STOP ready.
 *
 * Tune FEED_SECONDS and distances before a real match run.
 * Do not change motor directions or encoder constants here;
 * those are owned by RobotSubsystems.
 */
@Autonomous(name = "Sample Auto", group = "Autonomous")
public class SampleAuto extends LinearOpMode {

    // --- Tunable constants ---
    private static final double SPIN_UP_TIMEOUT_SECONDS = 4.0;  // max wait for shooter to reach speed
    private static final double FEED_SECONDS            = 1.5;  // how long to run the feeder
    private static final double DRIVE_POWER             = 0.55; // must be <= 0.25 (odometry limit)

    @Override
    public void runOpMode() {
        RobotSubsystems robot = new RobotSubsystems(hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        telemetry.addLine("Sample Auto ready.");
        telemetry.addLine("Spins up shooter, feeds, then drives 24 in, turns 90, strafes 12.");
        telemetry.addLine("Logo DOWN, USB RIGHT. Clear the area.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            robot.stop();
            return;
        }

        // init() gives odometry access to opModeIsActive(), telemetry, and idle().
        // Call it once right after waitForStart().
        robot.odometry.init(this);

        try {
            // ----------------------------------------------------------------
            // Step 1: Spin up the shooter, wait until it reaches speed or timeout.
            // ----------------------------------------------------------------f



            // ----------------------------------------------------------------
            // Step 4: Drive forward 24 inches, IMU holds starting heading.
            // ----------------------------------------------------------------
            //String r1 = robot.odometry.driveForward(24.0, DRIVE_POWER, 6.0);
            //telemetry.addData("Drive forward", r1);
            telemetry.update();

            // ----------------------------------------------------------------
            // Step 5: Turn 90 degrees counter-clockwise.
            // ----------------------------------------------------------------
            //String r2 = robot.odometry.turn(90.0, 9.0);
            //telemetry.addData("Turn 90", r2);
            telemetry.update();

            // ----------------------------------------------------------------
            // Step 6: Strafe right 12 inches.
            // ----------------------------------------------------------------
            String r3 = robot.odometry.strafe(12.0, DRIVE_POWER, 4.0);
            telemetry.addData("Strafe 12", r3);
            telemetry.update();

            sleep(10000); // hold final position briefly so telemetry is readable

        } finally {
            // Always runs — catches STOP button, exceptions, and normal completion.
            robot.stop();
        }
    }
}
