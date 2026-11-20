package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.RobotSubsystems;

/**
 * Thin wrapper kept for backwards compatibility with {@link TwoDeadWheelAuto}.
 *
 * <p>All hardware and logic now lives in {@link RobotSubsystems.Odometry}.
 * New auto OpModes should use {@code RobotSubsystems} directly:
 * <pre>
 *     RobotSubsystems robot = new RobotSubsystems(hardwareMap);
 *     robot.odometry.driveForward(this, 24.0, 0.20, 5.0);
 *     robot.odometry.turn(this, -90.0, 3.0);
 *     robot.stop();
 * </pre>
 */
public final class TwoDeadWheelDrive {
    private final RobotSubsystems.Odometry odometry;

    public TwoDeadWheelDrive(HardwareMap map) {
        // Initialise only the Odometry subsystem to avoid claiming TeleOp hardware.
        odometry = new RobotSubsystems(map).odometry;
    }

    public String driveForward(LinearOpMode opMode, double inches, double power,
                               double timeoutSeconds) {
        return odometry.driveForward(opMode, inches, power, timeoutSeconds);
    }

    public String strafe(LinearOpMode opMode, double inches, double power,
                         double timeoutSeconds) {
        return odometry.strafe(opMode, inches, power, timeoutSeconds);
    }

    public String turn(LinearOpMode opMode, double degrees, double timeoutSeconds) {
        return odometry.turn(opMode, degrees, timeoutSeconds);
    }

    public double headingDegrees() { return odometry.headingDegrees(); }
    public int forwardTicks()      { return odometry.forwardTicks(); }
    public int sidewaysTicks()     { return odometry.sidewaysTicks(); }

    /** Convenience pass-through kept for the existing test OpMode. */
    public void drive(double forward, double sideways, double turn) {
        // Not exposed on Odometry (drive is private), so just call stop.
        // TwoDeadWheelAuto only calls drive(0,0,0) to coast — stop() is equivalent.
        stop();
    }

    public void stop() { odometry.stop(); }
}
