package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.utils.general.maths.integration.predefined.VerletIntegrator;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@TeleOp(name = "MecanumTestDrive", group = "Test")
public class DriveTest extends LoopUtil {

    CustomMecanumDrive drive;
    Vector3d joystick;

    @Override
    public void opInit() {
        drive = new CustomMecanumDrive(hardwareMap, new VerletIntegrator(), 1, 1.1, 1);
        drive.mapMotors("FL", true, "BL", false, "FR", true, "BR", false);

        joystick = new Vector3d();
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        joystick.x = gamepad1.left_stick_x;
        joystick.y = -gamepad1.left_stick_y;
        joystick.z = gamepad1.right_stick_x;

        drive.update(joystick, true, deltaTime);
        telemetry.addData("Angle: ", drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        telemetry.addData("Position(Integrated): ", drive.getIntegrator().getCurrentPosition());
        telemetry.addData("Velocity(Integrated): ", drive.getIntegrator().getCurrentVelocity());
        telemetry.addData("Acceleration: ", drive.getImu().getLinearAcceleration());
        AngularVelocity avel = drive.getImu().getAngularVelocity();
        telemetry.addData("Turn Velocity: ", new Vector3d(avel.xRotationRate, avel.yRotationRate, avel.zRotationRate));
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
