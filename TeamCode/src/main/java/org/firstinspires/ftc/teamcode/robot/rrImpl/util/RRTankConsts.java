package org.firstinspires.ftc.teamcode.robot.rrImpl.util;

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class RRTankConsts {

    public static final double TICKS_PER_REV = 560;
    public static final double MAX_RPM = 300;

    public static final boolean RUN_USING_ENCODER = true;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(4/*18*/, 3/*16*/, 4/*18*/, 4/*18*/); //P:18 I:0 D:0 F:17.5
    //getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));
    //P:4, I:3, D:4, F:4

    public static double WHEEL_RADIUS = 0.48/*1.19*/; // in
    public static double GEAR_RATIO = 19/*1.6756756756756757*/; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 1/*20.05*/; // in

    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            20, 20, 0.0,
            4, 4, 0
    );

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
