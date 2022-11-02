package org.firstinspires.ftc.teamcode.robot.powerplay2022.auto.production;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.movement.AlgorithmicCorrection;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.misc.ColorView;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.movement.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.misc.PowerPlayGlobals;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.servos.kinematics.AngleConversion;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.servos.kinematics.ThreeJointArm;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.servos.kinematics.VirtualServo;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.slide.SlideController;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.statemachine.OpState;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.statemachine.OpStateList;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.movement.velocityramping.VelocityRampStepper;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.movement.velocityramping.VelocityRamping;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoConfig;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;
import org.firstinspires.ftc.teamcode.utils.sensors.color_range.RevColorRange;

@Autonomous(name = "MecanumAutoRight")
public class MecanumAuto extends LoopUtil {
    //Controls Declaration for Arm and Slide
    public ThreeJointArm newPropArm;

    public static ServoFTC servoA, servoB, servoC, servoD, servoR;
    public static ServoConfig configA, configB, configC, configD, configR;
    public static AngleConversion servoConversionA, servoConversionB, servoConversionC;
    public DcMotor left, right;
    SlideController.LEVEL slideLevelAuto;

    public static double commandedPositionMultiplier;
    public Vector2d betterCommandedPosition;

    public static SlideController slide;

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
    double elapsedTimeCycle;
    double elapsedTimeCycleAcum;
    double topConeY;

    public InputHandler inputHandler;
    public int startTick = 0, endTick = 0;
    public double startTime = 0, endTime = 0;

    @Override
    public void opInit() {
        //Slide
        slide = new SlideController(hardwareMap, "LSLM", true, "LSRM", false);
        //Velocity Ramps
        forwardRamp = new VelocityRamping(PowerPlayGlobals.MAX_VELOCITY);
        strafeRamp = new VelocityRamping(PowerPlayGlobals.MAX_VELOCITY);
        //Misc
        elapsedTime = 0;
        storedDeltaTime = 0;
        RCR2 = new RevColorRange(hardwareMap, telemetry, "rcr");
        CV2 = new ColorView(RCR2.color(), RCR2.distance());
        checked = false;
        inputHandler = InputAutoMapper.normal.autoMap(this);

        //Slide and Arm Auto Init
        configA = new ServoConfig("A",false, 0, 0.83);
        configB = new ServoConfig("B",true, 0, 1);
        configC = new ServoConfig("C",true, 0, 1);
        configD = new ServoConfig("D",true, 0, 1);
        configR = new ServoConfig("R",true, 0, 1);

        servoA = new ServoFTC(hardwareMap, telemetry, configA);
        servoB = new ServoFTC(hardwareMap, telemetry, configB);
        servoC = new ServoFTC(hardwareMap, telemetry, configC);
        servoD = new ServoFTC(hardwareMap, telemetry, configD);
        servoR = new ServoFTC(hardwareMap, telemetry, configR);

        servoConversionA = new AngleConversion(new AngleConversion.Centered(), AngleConversion.MODE.RADIANS);
        servoConversionB = new AngleConversion(new AngleConversion.Centered(), AngleConversion.MODE.RADIANS);
        servoConversionC = new AngleConversion(new AngleConversion.Centered(), AngleConversion.MODE.RADIANS);

        betterCommandedPosition = new Vector2d(0,25);
        commandedPositionMultiplier = 1;

        newPropArm = new ThreeJointArm(
                new VirtualServo(15, new Vector2d(0, -1), new Vector2d(0, 0)),
                new ServoFTC[]{servoA, servoB, servoC},
                new AngleConversion[]{servoConversionA, servoConversionB, servoConversionC},
                15,
                17);
        newPropArm.bindTelemetry(telemetry);

        elapsedTimeCycle = 0;
        elapsedTimeCycleAcum = 0;
        topConeY = -0.2;

        //Slide init
        inputHandler = InputAutoMapper.normal.autoMap(this);
        slide = new SlideController(hardwareMap, "LSLM", true, "LSRM", false);

        left = slide.getLeft();
        right = slide.getRight();

        slideLevelAuto = SlideController.LEVEL.REST;

        //Drive controls movement
        drive = new CustomMecanumDrive(hardwareMap, 1, 1.1, 1);
        drive.mapMotors("FL", false, "BL", true, "FR", false, "BR", true);

        //Correction outputs calculated turn speed
        correction = new AlgorithmicCorrection(new AlgorithmicCorrection.Polynomial(20));

        //Motion controls current wheel movement
        motion = new Vector3d();

        stepper = new VelocityRampStepper(forwardRamp, strafeRamp);
        stepper.addRampForward(1, 1.15, 1.75);

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
                            motion.x = -0.6;
                            motion.y = 0;
                            drive.update(motion, true, storedDeltaTime);
                        }
                ),
                new OpState(
                        () -> {
                            correction.update(drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, -Math.PI/2, true);
                            motion.z = correction.getOutput();
                            motion.x = 0.6;
                            motion.y = 0;
                            drive.update(motion, true, storedDeltaTime);
                        }
                )
        );

    }

    public void cycle(double deltaTime) { //Cycle 4 cones, 24.5 seconds
        elapsedTimeCycle += deltaTime;
        elapsedTimeCycleAcum += deltaTime;
        while(elapsedTimeCycleAcum < ((24.5 - (5.5 + 1)) * EULConstants.SEC2MS)) { //(Total Time - (Cycle Time + Buffer))
            if (elapsedTimeCycle < 3.5 * EULConstants.SEC2MS) {
                slideLevelAuto = SlideController.LEVEL.HIGH;
                servoR.setPosition(1);
                betterCommandedPosition.x = 20;
                betterCommandedPosition.y = -10;
                servoD.setPosition(0.6);
            } else if (elapsedTimeCycle < 3.65 * EULConstants.SEC2MS) {
                servoD.setPosition(0.07);
            } else if (elapsedTimeCycle < 5 * EULConstants.SEC2MS) {
                servoD.setPosition(0.6);
                slideLevelAuto = SlideController.LEVEL.REST;
                servoR.setPosition(0.5);
                betterCommandedPosition.x = 12;
                betterCommandedPosition.y = topConeY;
            } else if (elapsedTimeCycle < 5.5 * EULConstants.SEC2MS) {
                servoD.setPosition(0.07);
                slideLevelAuto = SlideController.LEVEL.REST;
                servoR.setPosition(0.5);
                betterCommandedPosition.x = 12;
                betterCommandedPosition.y = -2;
            } else {
                elapsedTimeCycle = 0;
                topConeY -= 3; //Centimeters between cones in stack
                break;
            }
        }
        servoD.setPosition(0.6);
        slideLevelAuto = SlideController.LEVEL.REST;
        servoR.setPosition(0.5);
        betterCommandedPosition.x = 15;
        betterCommandedPosition.y = 15;
    }

    public void givingUpCycle(double deltaTime) {
        elapsedTimeCycle += deltaTime;
        elapsedTimeCycleAcum += deltaTime;
        while(elapsedTimeCycleAcum < ((24.5 - (2.25 + 1)) * EULConstants.SEC2MS)) { //(Total Time - (Cycle Time + Buffer))
            if (elapsedTimeCycle < 0.75 * EULConstants.SEC2MS) {
                motion.x = -0.3;
                betterCommandedPosition.x = 0;
                betterCommandedPosition.y = 0;
                servoR.setPosition(0.5);
                servoD.setPosition(0.07);
            } else if (elapsedTimeCycle < 1 * EULConstants.SEC2MS) {
                servoD.setPosition(0.7);
            } else if (elapsedTimeCycle < 1.75 * EULConstants.SEC2MS) {
                motion.x = 0.3;
                betterCommandedPosition.x = 0;
                betterCommandedPosition.y = topConeY;
                servoR.setPosition(0.5);
            } else if (elapsedTimeCycle < 2.25 * EULConstants.SEC2MS) {
                servoD.setPosition(0.07);
                servoR.setPosition(0.5);
                betterCommandedPosition.x = 0;
                betterCommandedPosition.y = 5;
            } else {
                elapsedTimeCycle = 0;
                topConeY -= 3; //Centimeters between cones in stack
                break;
            }
        }
        servoD.setPosition(0.6);
        slideLevelAuto = SlideController.LEVEL.REST;
        servoR.setPosition(0.5);
        betterCommandedPosition.x = 15;
        betterCommandedPosition.y = 15;
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        inputHandler.loop();
        storedDeltaTime = deltaTime;
        elapsedTime += deltaTime;
        CV2.update(RCR2.color(), RCR2.distance());
        newPropArm.circleFind(betterCommandedPosition);
        slide.update(deltaTime, slideLevelAuto, 1);
        //All comments in StateLoop are wrong, correct later
        if (elapsedTime < 1.75 * EULConstants.SEC2MS) { //Drive Forward
            motion.y = -stepper.update(deltaTime * EULConstants.MS2SEC)[0];
            stateList.setIndex(0);
            servoD.setPosition(0.6);
        }else if (elapsedTime < 2.25*EULConstants.SEC2MS){ //Halted Turn
            stateList.setIndex(1);
        }else if (elapsedTime < 2.85*EULConstants.SEC2MS){ //
            stateList.setIndex(2);
        }else if (elapsedTime < 3.35*EULConstants.SEC2MS) { //Idle
            stateList.setIndex(1);
        }else if (elapsedTime < 27.85*EULConstants.SEC2MS) { //Cycle
            givingUpCycle(deltaTime);
        }else if (elapsedTime < 28.95*EULConstants.SEC2MS && SeenColor== ColorView.CMYcolors.YELLOW){ // Move to Yellow
            stateList.setIndex(3);
        }else if (elapsedTime < 28.5*EULConstants.SEC2MS && SeenColor== ColorView.CMYcolors.MAGENTA) { // Move to Magenta
            stateList.setIndex(3);
        }else { // Stay in Cyan
            stateList.setIndex(1);
        }



        if (RCR2.distance() < 15){
            if (!checked){ checked = true; ColorT1 = elapsedTime; }
            if (elapsedTime - ColorT1 < 250) {
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
