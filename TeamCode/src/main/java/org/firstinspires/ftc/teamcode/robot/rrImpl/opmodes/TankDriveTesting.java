package org.firstinspires.ftc.teamcode.robot.rrImpl.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.rrImpl.util.ModdedTankDrive;
import org.firstinspires.ftc.teamcode.tfodohm.TFMaths.TFMathExtension;
import org.firstinspires.ftc.teamcode.utils.LoopUtil;

@Config
@Autonomous(name="RRIMPLTest", group="Test")
public class TankDriveTesting extends LoopUtil {

    //basic roadrunner stuff
    public static ModdedTankDrive drive;
    public static Trajectory trajectory;
    public static FtcDashboard dashboard;
    public static TelemetryPacket packet;

    //poses
    public static Pose2d startingPose = new Pose2d(0, 0);
    public static double DISTANCE = 30;
    public static double trackedHeading = 0;
    public static double angleTurn = -90;

    @Override
    public void opInit() {
        setUpdateCap(30);

        //roadrunner stuffs
        drive = new ModdedTankDrive(hardwareMap);
        drive.setPoseEstimate(startingPose);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        packet = new TelemetryPacket();

        //external stuffs
        trackedHeading = drive.getRawExternalHeading();
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        Trajectory traj = drive.trajectoryBuilder(startingPose)
                .forward(DISTANCE)
                .build();
        drive.followTrajectory(traj);
        drive.turn(Math.toRadians(angleTurn));

        startingPose = traj.end().plus(new Pose2d(0, 0, Math.toRadians(angleTurn)));
    }

    @Override
    public void opFixedUpdate(double deltaTime) {
        drive.queueDraw();
    }

    @Override
    public void opStop() {
        drive.dispose();
    }
}
