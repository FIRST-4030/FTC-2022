package org.firstinspires.ftc.teamcode.robot.rrImpl.util;


import static org.firstinspires.ftc.teamcode.roadrunner.drive.TankDriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.TankDriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.TankDriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.TankDriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.TankDriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.TankDriveConstants.kA;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.TankDriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.TankDriveConstants.kV;

import android.content.SharedPreferences;

import androidx.annotation.NonNull;

/*
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

 */
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple tank drive hardware implementation for REV hardware.
 */
//@Config
public class ModdedTankDrive extends TankDrive {
    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0, 0, 0);
    //public static PIDCoefficients HEADING_PID = new PIDCoefficients(10, 13, 13); //P: 36; I:0; D:0
    //8, 5, 6
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(1, 0, 0.3);

    public static double VX_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private boolean drawCall = false;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    public enum BufferLooping {
        ONCE,
        REPEAT
    }

    //protected FtcDashboard dashboard;
    private NanoClock clock;

    private org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive.Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    private List<Pose2d> poseHistory;
    private Pose2dRecorder currentPoseRecorder;
    private PathRecorder sampledPathRecorder;
    private List<Trajectory> trajectoryBuffer;

    private List<DcMotorEx> motors, leftMotors, rightMotors;
    private BNO055IMU imu;

    private VoltageSensor batteryVoltageSensor;
    private HardwareMap hardwareMap;

    public ModdedTankDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH);


        this.hardwareMap = hardwareMap;

        currentPoseRecorder = new Pose2dRecorder();
        sampledPathRecorder = new PathRecorder();
        trajectoryBuffer = new ArrayList<>();

        //dashboard = FtcDashboard.getInstance();
        //dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive.Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new TankConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new TankPIDVAFollower(AXIAL_PID, CROSS_TRACK_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new ArrayList<>();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        // add/remove motors depending on your robot (e.g., 6WD)
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "BL");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "BR");
        //DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        //DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        motors = Arrays.asList(leftRear, rightRear);
        leftMotors = Arrays.asList(leftRear);
        rightMotors = Arrays.asList(rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
    }

    /**
     * Trajectory Buffering for stuff
     */

    private BufferLooping trajLoop = BufferLooping.ONCE;
    public void setupTrajectoryBuffer(BufferLooping type){
        trajLoop = type;
    }

    public void trajectoryBufferAdd(Trajectory traj){
        trajectoryBuffer.add(traj);
    }

    public void trajectoryBufferRemove(int idx){
        trajectoryBuffer.remove(idx);
    }

    private int currentTrajBufferIdx = 0;
    public void trajectoryBufferExecuteNext(){
        //Follows the "first in last out" rule
        if (!isBusy() && (trajectoryBuffer.size() != 0)){
            followTrajectory(trajectoryBuffer.get(currentTrajBufferIdx));
            switch (trajLoop){
                default:
                case ONCE:
                    trajectoryBuffer.remove(0); //removes the first trajectory queued
                    break;
                case REPEAT:
                    currentTrajBufferIdx++; //increment index when executed specified
                    currentTrajBufferIdx %= trajectoryBuffer.size(); //normalizes the idx to the buffer's length
                    break;
            }

        }
    }

    /**
     * End of Trajectory Buffering
     */

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, constraints);
    }

    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );
        turnStart = clock.seconds();
        mode = org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive.Mode.TURN;
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        poseHistory.clear();
        follower.followTrajectory(trajectory);
        mode = org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive.Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        /*
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("Mode: ", mode);

        packet.put("X: ", currentPose.getX());
        packet.put("Y: ", currentPose.getY());
        packet.put("Heading/Direction: ", currentPose.getHeading());

        packet.put("X Error: ", lastError.getX());
        packet.put("Y Error: ", lastError.getY());
        packet.put("Heading/Direction Error: ", lastError.getHeading());


         */
        switch (mode) {
            case IDLE:
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                if (t >= turnProfile.duration()) {
                    mode = org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive.Mode.IDLE;

                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {

                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                /*
                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
                DashboardUtil.drawRobot(fieldOverlay, currentPose);

                 */
                //under FOLLOW_TRAJECTORY because we don't need to record every single update call
                currentPoseRecorder.record(currentPose); //records the current position of the bot
                sampledPathRecorder.record(trajectory.getPath()); //records the expected path of the bot

                if (!follower.isFollowing()) {
                    mode = org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive.Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        /*
        if (drawCall){
            DashboardUtil.drawPoseHistory(fieldOverlay.setStroke("#ff0000"), currentPoseRecorder.getAsList());
            DashboardUtil.drawSampledPaths(fieldOverlay.setStroke("#000000"), sampledPathRecorder.getAsList());
        }

        drawCall = false;

        dashboard.sendTelemetryPacket(packet);

         */
    }

    public void queueDraw(){
        this.drawCall = true;
    }


    public void dispose(){
        currentPoseRecorder.removeAll();
        sampledPathRecorder.removeAll();
        turnController.reset();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive.Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    0,
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        double leftSum = 0, rightSum = 0;
        for (DcMotorEx leftMotor : leftMotors) {
            leftSum += encoderTicksToInches(leftMotor.getCurrentPosition());
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightSum += encoderTicksToInches(rightMotor.getCurrentPosition());
        }
        return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
    }
    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        double leftSum = 0, rightSum = 0;
        for (DcMotorEx leftMotor : leftMotors) {
            leftSum += encoderTicksToInches(leftMotor.getVelocity());
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightSum += encoderTicksToInches(rightMotor.getVelocity());
        }
        return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
    }

    @Override
    public void setMotorPowers(double v, double v1) {
        for (DcMotorEx leftMotor : leftMotors) {
            leftMotor.setPower(v);
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightMotor.setPower(v1);
        }
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
