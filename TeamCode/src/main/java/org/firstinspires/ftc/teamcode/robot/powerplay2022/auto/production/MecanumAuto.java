package org.firstinspires.ftc.teamcode.robot.powerplay2022.auto.production;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.AlgorithmicCorrection;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.ColorView;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.InputAutoMapper;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.velocityramping.VelocityRampStepper;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.velocityramping.VelocityRamping;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.statemachine.OpState;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.statemachine.OpStateList;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;
import org.firstinspires.ftc.teamcode.utils.sensors.color_range.RevColorRange;

@TeleOp(name = "MecanumAuto")
public class MecanumAuto extends LoopUtil {

    public static CustomMecanumDrive drive;
    public static AlgorithmicCorrection correction;
    public static Vector3d motion;
    public static OpStateList stateList;
    double storedDeltaTime;
    double elapsedTime;
    public static RevColorRange RCR2;
    public static ColorView.CMYcolors SeenColor;
    public static ColorView CV2;
    //
    public static double ColorT1;
    public static boolean checked;
    public VelocityRamping forwardRamp, strafeRamp;
    public VelocityRampStepper stepper;

    public InputHandler inputHandler;
    public int startTick = 0, endTick = 0;
    public double startTime = 0, endTime = 0;

    @Override
    public void opInit() {
        //Velocity Ramps
        forwardRamp = new VelocityRamping(1.2192);
        strafeRamp = new VelocityRamping(1.3);
        //Misc
        elapsedTime = 0;
        storedDeltaTime = 0;
        RCR2 = new RevColorRange(hardwareMap, telemetry, "rcr");
        CV2 = new ColorView(RCR2.color(), RCR2.distance());
        checked = false;
        inputHandler = InputAutoMapper.normal.autoMap(this);

        //Drive controls movement
        drive = new CustomMecanumDrive(hardwareMap, 1, 1.1, 1);
        drive.mapMotors("FL", false, "BL", true, "FR", false, "BR", true);

        //Correction outputs calculated turn speed
        correction = new AlgorithmicCorrection(new AlgorithmicCorrection.Polynomial(20));

        //Motion controls current wheel movement
        motion = new Vector3d();

        stepper = new VelocityRampStepper(forwardRamp, strafeRamp);
        stepper.addRampForward(1, 1.24, 1.75);

        //...
        Runnable Idle = () -> {
            motion.x = 0;
            motion.y = 0;
        };

        Runnable driveUpdate = () -> {

        };


        //StateList is the list of all states the robot will occupy in autonomous
        stateList = new OpStateList();

        stateList.addStates(
                new OpState(
                        () -> {
                            correction.update(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, Math.PI, true);
                            motion.z = correction.getOutput();
                            motion.x = 0;
                            drive.update(motion, true, storedDeltaTime);
                        }
                ),
                new OpState(
                        Idle,
                        () -> {
                            correction.update(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, -Math.PI/2, true);
                            motion.z = correction.getOutput();
                            drive.update(motion, true, storedDeltaTime);
                        }
                ),
                new OpState(
                        () -> {
                            correction.update(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, -Math.PI/2, true);
                            motion.z = correction.getOutput();
                            motion.x = -0.8;
                            motion.y = 0;
                            drive.update(motion, true, storedDeltaTime);
                        }
                ),
                new OpState(
                        () -> {
                            correction.update(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, -Math.PI/2, true);
                            motion.z = correction.getOutput();
                            motion.x = 0.8;
                            motion.y = 0;
                            drive.update(motion, true, storedDeltaTime);
                        }
                )
        );

    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        storedDeltaTime = deltaTime;
        elapsedTime += deltaTime;
        CV2.update(RCR2.color(), RCR2.distance());
/*
        if (elapsedTime < 1.75 * EULConstants.SEC2MS) {
            motion.y = -stepper.update(deltaTime * EULConstants.MS2SEC)[0];

            stateList.setIndex(0);

        }

 */
        /*
        if (elapsedTime < (1.75/2)*EULConstants.SEC2MS) {
            //VRamp.solve(1.1, 1.75);
            //motion.y = -(Math.min((elapsedTime*EULConstants.MS2SEC) * VRamp.acceleration, 1.3) / 1.3);
            stateList.setIndex(0);
        }else if (elapsedTime < 1.75*EULConstants.SEC2MS) {
            //VRamp.solve(1.1, 1.75);
            //motion.y = -((Math.min(((elapsedTime*EULConstants.MS2SEC)-1.75) * -VRamp.acceleration, 1.3))+0 / 1.3);
            stateList.setIndex(0);

         */
        if (elapsedTime < 1.5 * EULConstants.SEC2MS){
            motion.x = 0;
            motion.y = -1;
            motion.z = 0;

        } else {
            motion.x = 0;
            motion.y = 0;
            motion.z = 0;
        }/*else if (elapsedTime < 2.25*EULConstants.SEC2MS){
            stateList.setIndex(1);
        }else if (elapsedTime < 2.85*EULConstants.SEC2MS){
            stateList.setIndex(2);
        }else if (elapsedTime < 27.85*EULConstants.SEC2MS){
            stateList.setIndex(1);
        }else if (elapsedTime < 28.95*EULConstants.SEC2MS && SeenColor== ColorView.CMYcolors.YELLOW){
            stateList.setIndex(3);
        }else if (elapsedTime < 28.5*EULConstants.SEC2MS && SeenColor== ColorView.CMYcolors.MAGENTA) {
            stateList.setIndex(3);
        }else {
            stateList.setIndex(1);
        }*/



        if (RCR2.distance() < 60){
            if (!checked){ checked = true; ColorT1 = elapsedTime; }
            if (elapsedTime - ColorT1 < 100) {
                SeenColor = CV2.getColorBetter(100);
            }
        }

        if (inputHandler.up("D1:LB")){
            startTick = drive.getMotor(0).getCurrentPosition();
            startTime = elapsedTime;
        }

        if (inputHandler.up("D1:RB")){
            endTick = drive.getMotor(0).getCurrentPosition();
            endTime = elapsedTime;
        }

        stateList.getCurrentState().runAll(deltaTime);
        telemetry.addData("Time: ", elapsedTime * EULConstants.MS2SEC);
        telemetry.addData("State Index: ", stateList.getIndex());
        telemetry.addData("Saved Color: ", SeenColor);
        telemetry.addData("Dist: ", RCR2.distance());
        telemetry.addData("ColorBetter: ", CV2.getColorBetter(80));
        telemetry.addData("Color: ", CV2.getColor());
        telemetry.addData("Color: ", CV2.convertRGBToHSV(CV2.colorInput)[0]);
        telemetry.addData("Motion.y", motion.y);

        telemetry.addData("VelocityRecording: ", "");
        telemetry.addData("Start Tick: ", startTick);
        telemetry.addData("End Tick: ", endTick);
        telemetry.addData("Absolute Delta: ", Math.abs(endTick - startTick));
        telemetry.addData("Start Time: ", startTime);
        telemetry.addData("End Time: ", endTime);
        telemetry.addData("Absolute Delta: ", Math.abs(endTime - startTime));
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
