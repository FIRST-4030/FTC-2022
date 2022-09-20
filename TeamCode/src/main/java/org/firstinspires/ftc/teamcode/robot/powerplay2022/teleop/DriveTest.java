package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.MecanumMovementFactory;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@TeleOp(name = "MecanumTestDrive", group = "Test")
public class DriveTest extends LoopUtil {

    MecanumMovementFactory drive;
    Vector3d joystick;

    @Override
    public void opInit() {
        drive = new MecanumMovementFactory(hardwareMap, 1, 1.1, 1);
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

    }

    @Override
    public void opFixedUpdate(double deltaTime) {
        joystick.x = gamepad1.left_stick_x;
        joystick.y = -gamepad1.left_stick_y;
        joystick.z = gamepad1.right_stick_x;

        drive.update(joystick, true);
    }

    @Override
    public void opStop() {

    }
}
