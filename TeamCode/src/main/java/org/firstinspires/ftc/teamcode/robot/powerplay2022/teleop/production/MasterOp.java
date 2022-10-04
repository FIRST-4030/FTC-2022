package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop.production;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.auto.production.MecanumAuto;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

@Autonomous(name = "MasterOp")
public class MasterOp extends LoopUtil {

    public static LoopUtil auto, tele;
    public double totalTime;

    @Override
    public void opInit() {
        tele = new DriveTest();
        auto = new MecanumAuto();
        tele.hardwareMap = this.hardwareMap;
        auto.hardwareMap = this.hardwareMap;
        tele.opInit();
        auto.opInit();
        totalTime = 0;
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        tele.gamepad1 = gamepad1;
        auto.gamepad1 = gamepad1;
        tele.gamepad2 = gamepad2;
        auto.gamepad2 = gamepad2;

        totalTime+=deltaTime;
        if(totalTime < 30 * EULConstants.SEC2MS){
            auto.opUpdate(deltaTime);
        }else{
            tele.opFixedUpdate(deltaTime);
        }
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
