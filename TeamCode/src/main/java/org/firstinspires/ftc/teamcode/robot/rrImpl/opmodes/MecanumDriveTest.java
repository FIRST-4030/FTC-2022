package org.firstinspires.ftc.teamcode.robot.rrImpl.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Config
@Autonomous(name="RRIMPLMecanumTest", group="Test")
public class MecanumDriveTest extends OpMode {

    //drive
    public static SampleMecanumDrive drive;

    //poses
    public static Pose2d startingPose = new Pose2d( 0, 0 );

    @Override
    public void init() {
        //initialize drive and virtual position
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startingPose);
    }

    @Override
    public void loop() {
        //build trajectory that goes forward by 24 inches
        Trajectory tra = drive.trajectoryBuilder(startingPose)
                .forward(24)
                .build();

        //submit trajectory
        drive.followTrajectory(tra);
    }
}
