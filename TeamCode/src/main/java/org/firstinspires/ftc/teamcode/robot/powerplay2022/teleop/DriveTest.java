package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.AnglePID;
import org.firstinspires.ftc.teamcode.utils.general.maths.integration.predefined.VerletIntegrator;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@TeleOp(name = "MecanumTestDrive", group = "Test")
public class DriveTest extends LoopUtil {

    CustomMecanumDrive drive;
    Vector3d joystick;
    AnglePID Apid;

    @Override
    public void opInit() {
        drive = new CustomMecanumDrive(hardwareMap, new VerletIntegrator(), 1, 1.1, 1);
        drive.mapMotors("FL", true, "BL", false, "FR", true, "BR", false);

        joystick = new Vector3d();

        Apid = new AnglePID(1/Math.PI, 0.000001, 1/4000);
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {

    }

    @Override
    public void opFixedUpdate(double deltaTime) {
        if (loop_timer.milliseconds() < 3000) {
            Apid.update(deltaTime, Math.PI, drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        }else{
            Apid.update(deltaTime, 0, drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        }
        joystick.x = gamepad1.left_stick_x;
        joystick.y = -gamepad1.left_stick_y;
        joystick.z = gamepad1.right_stick_x;
        joystick.z -= Apid.correctionPower;

        drive.update(joystick, true, deltaTime);
        telemetry.addData("Angle: ", drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        telemetry.addData("Position(Integrated): ", drive.getIntegrator().getCurrentPosition());
        telemetry.addData("Velocity(Integrated): ", drive.getIntegrator().getCurrentVelocity());
        telemetry.addData("Acceleration: ", drive.getImu().getLinearAcceleration());
        AngularVelocity avel = drive.getImu().getAngularVelocity();
        telemetry.addData("Turn Velocity: ", new Vector3d(avel.xRotationRate, avel.yRotationRate, avel.zRotationRate));
    }

    @Override
    public void opStop() {

    }
}
