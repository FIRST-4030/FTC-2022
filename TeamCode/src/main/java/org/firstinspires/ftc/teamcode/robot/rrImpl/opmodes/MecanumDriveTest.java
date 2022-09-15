package org.firstinspires.ftc.teamcode.robot.rrImpl.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Config
@Autonomous(name="RRIMPLMecanumTest", group="Test")
public class MecanumDriveTest extends OpMode {

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    //drive
    public static SampleMecanumDrive drive;

    //poses
    public static Pose2d startingPose = new Pose2d( 0, 0 );

    @Override
    public void init() {
        //initialize drive and virtual position
        //drive = new SampleMecanumDrive(hardwareMap);
        //drive.setPoseEstimate(startingPose);

        motorFrontLeft = hardwareMap.get(DcMotor.class,"FL");
        motorBackLeft = hardwareMap.get(DcMotor.class,"BL");
        motorFrontRight = hardwareMap.get(DcMotor.class,"FR");
        motorBackRight = hardwareMap.get(DcMotor.class,"BR");
    }

    @Override
    public void loop() {
        //build trajectory that goes forward by 24 inches
        //Trajectory tra = drive.trajectoryBuilder(startingPose)
               // .forward(24)
               // .build();

        //tra.end();
        //submit trajectory
        //drive.followTrajectory(tra);
        /*
        double rx = (gamepad1.right_stick_x)/2;
        double y = (-gamepad1.left_stick_y)/2;
        double x = (gamepad1.left_stick_x)/2;

         */

        double x = gamepad1.left_stick_y * 0.6;
        double y = -gamepad1.left_stick_x * 0.5;
        double rx = -gamepad1.right_stick_x * 0.25;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = ( y - x - rx) / denominator;
        double frontRightPower = ( y - x + rx) / denominator;
        double backRightPower = ( y + x - rx) / denominator;

        motorFrontLeft.setPower(-frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(-frontRightPower);
        motorBackRight.setPower(backRightPower);
    }
}
