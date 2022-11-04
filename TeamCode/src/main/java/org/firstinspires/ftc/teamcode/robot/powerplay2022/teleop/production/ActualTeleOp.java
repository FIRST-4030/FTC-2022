package org.firstinspires.ftc.teamcode.robot.powerplay2022.teleop.production;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.misc.EULConstants;
import org.firstinspires.ftc.teamcode.robot.frieghtfrenzy2021.Globals;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.depreciated.movement.AnglePID;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.misc.ColorView;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.misc.InputAutoMapper;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.movement.AlgorithmicCorrection;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.movement.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.servos.kinematics.AngleConversion;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.servos.kinematics.ThreeJointArm;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.servos.kinematics.VirtualServo;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.slide.SlideController;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.statemachine.OpState;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.statemachine.OpStateList;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoConfig;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.utils.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.utils.momm.LoopUtil;
import org.firstinspires.ftc.teamcode.utils.sensors.color_range.RevColorRange;
import org.firstinspires.ftc.teamcode.utils.sensors.distance.RevDistance;

//Control mapping

/**
 * DRIVER 1:
 * Left joystick is the drive (forward, backward, strafe) and right joystick is turn angle
 * DRIVER 2:
 * Left joystick controls the servo arm (y for inward and outward, x for turning)
 * Right joystick y controls the vertical portion of the servo arm
 * Linear slide control: A is linear slide at its natural resting position; B is to raise the level to low junctions; Y is to raise it to medium junctions; X is to raise it to the high junction
 * Claw of servo arm closes on Right Bumper
 */
@TeleOp(name = "ActualTeleOp", group = "actual")
public class ActualTeleOp extends LoopUtil {

    public static OpStateList stateList;

    public ThreeJointArm newPropArm;

    public static ServoFTC servoA, servoB, servoC, servoD, servoR;
    public static ServoConfig configA, configB, configC, configD, configR;
    public static AngleConversion servoConversionA, servoConversionB, servoConversionC;

    public static InputHandler gamepadHandler;
    public static boolean enableJoystick;
    public static double commandedPosition;
    public static double commandedPositionMultiplier;
    public Vector2d betterCommandedPosition;

    public static CustomMecanumDrive drive  = null;
    public static Vector3d joystick = new Vector3d();
    public static Vector2d right_stick = new Vector2d();
    public static AnglePID Apid = null;
    public static double[] angles = null;
    public static int angleIndex = 0;
    public static boolean lastStateLB = false, lastStateRB = false,currentStateLB = false, currentStateRB = false;
    public static double pGain = 0, iGain = 0, dGain = 0;
    ///public static RevDistance D1, D2;


    public InputHandler inputHandler;
    public SlideController controller;

    public DcMotor left, right;

    public double linearSlideSpeed = 0.5;
    public SlideController.LEVEL slideLevel =
            SlideController.LEVEL.REST;
    public double lsInput = 0;
    public boolean DOpen = false;
    double DPos = DOpen ? 0.07 : 0.7;
    public double R = 0.5;

    public double RunnableTimer = 0;
    public boolean PickUpRunning = false;
    public boolean StepperLowerRunning = false;



    //Algorithm-based correction (not PID)
    public static AlgorithmicCorrection correction;

    RevColorRange RCR2;
    ColorView CV2;

    Runnable setArmToPlace = () -> {
        betterCommandedPosition.x = 26;
        betterCommandedPosition.y = 5;
        R = 0.5;
        DOpen = false;
    };
    Runnable setArmToStow = () -> {
        betterCommandedPosition.x = 10;
        betterCommandedPosition.y = 10;
        R = 0.5;
        DOpen = false;
    };
    Runnable setArmToIntake = () -> {
        betterCommandedPosition.x = 26;
        betterCommandedPosition.y = 5;
        R = 0.5;
        DOpen = true;
    };
    Runnable highPlace = () -> {
        slideLevel = SlideController.LEVEL.HIGH;
        setArmToPlace.run();
    };
    Runnable midPlace = () -> {
        slideLevel = SlideController.LEVEL.MIDDLE;
        setArmToPlace.run();
    };
    Runnable lowPlace = () -> {
        slideLevel = SlideController.LEVEL.LOW;
        setArmToPlace.run();
    };
    Runnable groundPlace = () -> {
        slideLevel = SlideController.LEVEL.REST;
        setArmToPlace.run();
    };
    Runnable pickUp = () -> {
        if(RunnableTimer < 1* EULConstants.SEC2MS){
            betterCommandedPosition.y = -5;
            DOpen = true;
        }else if(RunnableTimer < 1.5* EULConstants.SEC2MS){
            DOpen = false;
        }else if(RunnableTimer < 1.75* EULConstants.SEC2MS){
            betterCommandedPosition.y = 5;
        }else{
            PickUpRunning = false;
        }
    };
    Runnable stepperLower = () -> {
        if(RunnableTimer > 1* EULConstants.SEC2MS){
            RunnableTimer = 0;
            if(slideLevel == SlideController.LEVEL.HIGH){
                slideLevel = SlideController.LEVEL.MIDDLE;
            }
            if(slideLevel == SlideController.LEVEL.MIDDLE){
                slideLevel = SlideController.LEVEL.LOW;
            }
            if(slideLevel == SlideController.LEVEL.LOW){
                slideLevel = SlideController.LEVEL.REST;
                StepperLowerRunning = false;
            }
        }
    };



    @Override
    public void opInit() {

        //Pre-Defined Arm/Slide Movements and Positions


        //Arm init
        configA = new ServoConfig("A",false, 0.0001, 0.83);
        configB = new ServoConfig("B",true, 0.0001, 0.9999);
        configC = new ServoConfig("C",true, 0.0001, 0.9999);
        configD = new ServoConfig("D",true, 0.0001, 0.9999);
        configR = new ServoConfig("R",true, 0.0001, 0.9999);

        servoA = new ServoFTC(hardwareMap, telemetry, configA);
        servoB = new ServoFTC(hardwareMap, telemetry, configB);
        servoC = new ServoFTC(hardwareMap, telemetry, configC);
        servoD = new ServoFTC(hardwareMap, telemetry, configD);
        servoR = new ServoFTC(hardwareMap, telemetry, configR);

        servoConversionA = new AngleConversion(new AngleConversion.Centered(), AngleConversion.MODE.RADIANS);
        servoConversionB = new AngleConversion(new AngleConversion.Centered(), AngleConversion.MODE.RADIANS);
        servoConversionC = new AngleConversion(new AngleConversion.Centered(), AngleConversion.MODE.RADIANS);

        gamepadHandler = InputAutoMapper.normal.autoMap(this);
        enableJoystick = false;
        commandedPosition = 0.0;
        betterCommandedPosition = new Vector2d(20,4);
        commandedPositionMultiplier = 1;

        newPropArm = new ThreeJointArm(
                new VirtualServo(15, new Vector2d(0, -1), new Vector2d(0, 0)),
                new ServoFTC[]{servoA, servoB, servoC},
                new AngleConversion[]{servoConversionA, servoConversionB, servoConversionC},
                15,
                17);
        newPropArm.bindTelemetry(telemetry);

        //Slide init
        inputHandler = InputAutoMapper.normal.autoMap(this);
        controller = new SlideController(hardwareMap, "LSLM", true, "LSRM", false);

        left = controller.getLeft();
        right = controller.getRight();

        //Drive init
        Globals.opmode(this);
        Globals.input(this);

        drive = new CustomMecanumDrive(hardwareMap, 1, 1.1, 1);
        drive.mapMotors("FL", false, "BL", true, "FR", false, "BR", true);

        joystick = new Vector3d();
        right_stick = new Vector2d(0, 1);

        lastStateRB = false;
        lastStateLB = false;
        currentStateLB = false;
        currentStateRB = false;

        RCR2 = new RevColorRange(hardwareMap, telemetry, "rcr");
        CV2 = new ColorView(RCR2.color(), RCR2.distance());

        correction = new AlgorithmicCorrection(new AlgorithmicCorrection.Polynomial(20));

        //D1 = new RevDistance(hardwareMap, telemetry, "range1");
        //D2 = new RevDistance(hardwareMap, telemetry, "range2");
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        if(!PickUpRunning) {
            handleInput(deltaTime);
        }
        armUpdate(deltaTime);
        slideUpdate(deltaTime);
        outputTelemetry();
        DPos = DOpen ? 0.07 : 0.7;
        if(Double.isNaN(DPos)){DPos=0.6;}
        if(Double.isNaN(R)){R=0.5;}
        servoD.setPosition(DPos);
        servoR.setPosition(1-R);
        RunnableTimer += deltaTime;
    }

    @Override
    public void opFixedUpdate(double deltaTime) {
        driveFixedUpdate(deltaTime);
    }

    public void armUpdate(double deltaTime) {
        //newPropArm.propagate(betterCommandedPosition, new Vector2d( 1, 0),true);
        newPropArm.circleFind(betterCommandedPosition);
        if(PickUpRunning){
            pickUp.run();
        }
        if(StepperLowerRunning){
            stepperLower.run();
        }
    }

    public void slideUpdate(double deltaTime){
        controller.update(deltaTime, slideLevel, linearSlideSpeed);
    }

    public void handleInput(double deltaTime){
        inputHandler.loop();
        //D Control
        if (inputHandler.up("D2:RB")){
            DOpen = !DOpen;
        }
        //Slide controls
        if (gamepad2.a){
            slideLevel = SlideController.LEVEL.REST;
        } else if (gamepad2.b){
            slideLevel = SlideController.LEVEL.LOW;
        } else if (gamepad2.y){
            slideLevel = SlideController.LEVEL.MIDDLE;
        } else if (gamepad2.x){
            slideLevel = SlideController.LEVEL.HIGH;
        }

        //arm controls
        gamepadHandler.loop();
        if (gamepadHandler.up("D2:LB")){
            RunnableTimer = 0;
            PickUpRunning = true;
        }
        if (gamepadHandler.up("D2:RT")){
            StepperLowerRunning = true;
        }
        if (gamepadHandler.up("D2:LT")){
            setArmToStow.run();
        }

        if (gamepadHandler.up("D2:DPAD_DOWN")){ //decrease outputSpeed by decimalPlace | now wrong comment
            setArmToIntake.run();
            StepperLowerRunning = true;
        }

        if (gamepadHandler.up("D2:DPAD_UP")){ //increase outputSpeed by decimalPlace | now wrong comment
            lowPlace.run();
        }
        if (gamepadHandler.up("D2:DPAD_LEFT")){ //increase outputSpeed by decimalPlace | now wrong comment
            midPlace.run();
        }
        if (gamepadHandler.up("D2:DPAD_RIGHT")){ //decrease outputSpeed by decimalPlace | now wrong comment
            groundPlace.run();
        }


        betterCommandedPosition = betterCommandedPosition.plus((new Vector2d(gamepad2.left_stick_y, -gamepad2.right_stick_y).times(0.5)));
        R = EULMathEx.doubleClamp(0.001, 0.999, R+gamepad2.left_stick_x*0.008);
    }

    public void driveFixedUpdate(double deltaTime){
        lastStateRB = currentStateRB;
        lastStateLB = currentStateLB;

        currentStateLB = gamepad1.left_bumper;
        if(currentStateLB && !lastStateLB){
            angleIndex++;
        }

        currentStateRB = gamepad1.right_bumper;
        if(currentStateRB && !lastStateRB){
            angleIndex--;
        }

        if(angleIndex <= -1){
            angleIndex = 3;
        } else if (angleIndex >= 4){
            angleIndex = 0;
        }

        joystick.x = gamepad1.left_stick_x * -1 * (controller.isInUse() ? 0.2 : 1);
        joystick.y = -gamepad1.left_stick_y * -1 * (controller.isInUse() ? 0.2 : 1);

        telemetry.addData("Joystick X: ", joystick.x);
        telemetry.addData("Joystick Y: ", joystick.y);

        right_stick.x = -gamepad1.right_stick_x;
        right_stick.y = gamepad1.right_stick_y;

        correction.update( drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, right_stick, false);

        CV2.update(RCR2.color(), RCR2.distance());

        joystick.z = correction.getOutput() * (controller.isInUse() ? 0.2 : 1);
        drive.update(joystick, true, deltaTime);
        AngularVelocity avel = drive.getImu().getAngularVelocity();
    }

    @Override
    public void opStop() {

    }

    public void outputTelemetry(){
        telemetry.addData("Commanded Multiplier: ", commandedPositionMultiplier);
        telemetry.addData("Commanded Position: ", betterCommandedPosition);
        telemetry.addData("Servo A Turn Position: ", servoA.getPosition());
        telemetry.addData("Servo B Turn Position: ", servoB.getPosition());
        telemetry.addData("Servo C Turn Position: ", servoC.getPosition());
        telemetry.addData("Servo D Turn Position: ", servoD.getPosition());
        telemetry.addData("Slide is in use?: ", controller.isInUse());
    }
}
