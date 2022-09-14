package org.firstinspires.ftc.teamcode.robot.rrImpl.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name="RRIMPLMecanumTest", group="Test")
public class MecanumDriveTest extends OpMode {

    public static SampleMecanumDrive drive;

    //poses
    public static Pose2d startingPose = new Pose2d( 0, 0 );

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startingPose);
    }

    @Override
    public void loop() {
        Trajectory tra = drive.trajectoryBuilder(startingPose).forward(24).build();
        drive.followTrajectory(tra);

    }
}
