package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop.production;

import org.firstinspires.ftc.teamcode.robot.powerplay2022.auto.production.MecanumAuto;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;

public class MasterOp extends LoopUtil {

    public static LoopUtil auto, tele;
    public double totalTime;

    @Override
    public void opInit() {
        tele = new DriveTest();
        auto = new MecanumAuto();
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
        totalTime+=deltaTime;
        if(totalTime < 0){
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
