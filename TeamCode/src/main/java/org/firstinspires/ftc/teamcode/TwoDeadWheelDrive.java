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
 *     robot.odometry.init(this);          // call once after waitForStart()
 *     robot.odometry.driveForward(24.0, 0.20, 5.0);
 *     robot.odometry.turn(-90.0, 3.0);
 *     robot.stop();
 * </pre>
 */
public final class TwoDeadWheelDrive {
    private final RobotSubsystems.Odometry odometry;

    public TwoDeadWheelDrive(HardwareMap map) {
        odometry = new RobotSubsystems(map).odometry;
    }

    public String driveForward(LinearOpMode opMode, double inches, double power,
                               double timeoutSeconds) {
        odometry.init(opMode);
        return odometry.driveForward(inches, power, timeoutSeconds);
    }

    public String strafe(LinearOpMode opMode, double inches, double power,
                         double timeoutSeconds) {
        odometry.init(opMode);
        return odometry.strafe(inches, power, timeoutSeconds);
    }

    public String turn(LinearOpMode opMode, double degrees, double timeoutSeconds) {
        odometry.init(opMode);
        return odometry.turn(degrees, timeoutSeconds);
    }

    public double headingDegrees() { return odometry.headingDegrees(); }
    public int forwardTicks()      { return odometry.forwardTicks(); }
    public int sidewaysTicks()     { return odometry.sidewaysTicks(); }

    /** Kept for TwoDeadWheelAuto's coast call; equivalent to stop(). */
    public void drive(double forward, double sideways, double turn) { stop(); }

    public void stop() { odometry.stop(); }
}
