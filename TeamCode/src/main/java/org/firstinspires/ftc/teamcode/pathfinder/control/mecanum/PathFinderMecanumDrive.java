package org.firstinspires.ftc.teamcode.pathfinder.control.mecanum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.matrices.Matrix4d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector4d;
import org.firstinspires.ftc.teamcode.pathfinder.control.PathFinderDrive;
import org.firstinspires.ftc.teamcode.pathfinder.utilities.PFPath;
import org.firstinspires.ftc.teamcode.pathfinder.utilities.PFPose2d;

import java.util.Objects;
import java.util.Vector;

public class PathFinderMecanumDrive extends PathFinderDrive {

    //TODO: Measure and modify ticksPer[AdvancementCM (Relative forward/backwards), StrafeCM(Relative strafing), Turn(relative turn)] constants
    private double ticksPerAdvancementCM, ticksPerStrafeCM, ticksPerTurn;

    private PFPath followingPath;

    private DcMotorEx fl, fr, bl, br;

    private Matrix4d powerPartitionMatrix;
    private double strafeCo, advancementCo, turnCo, coefficientSum;
    private float encoderReverseFL, encoderReverseFR, encoderReverseBL, encoderReverseBR;

    public PathFinderMecanumDrive(HardwareMap hardwareMap, Telemetry telemetry, String flMotorName, boolean flReversed, String frMotorName, boolean frReversed, String blMotorName, boolean blReversed, String brMotorName, boolean brReversed){
        super(hardwareMap, telemetry);

        /**Beginning of setting up motors**/

        this.fl = (DcMotorEx) hardwareMap.get(DcMotor.class, flMotorName);
        this.fr = (DcMotorEx) hardwareMap.get(DcMotor.class, frMotorName);
        this.bl = (DcMotorEx) hardwareMap.get(DcMotor.class, blMotorName);
        this.br = (DcMotorEx) hardwareMap.get(DcMotor.class, brMotorName);

        this.fl.setDirection(flReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        this.fr.setDirection(frReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        this.bl.setDirection(blReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        this.br.setDirection(brReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        this.fl.setTargetPosition(0);
        this.fr.setTargetPosition(0);
        this.bl.setTargetPosition(0);
        this.br.setTargetPosition(0);

        this.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /**End of setting up motors**/

        /**Beginning of setting up the power partition Matrix**/

        this.modifyPowerPartitionCoefficients(1, 1, 1);

        /**End of setting up power partition Matrix**/

        setupInternals();
    }

    public PathFinderMecanumDrive modifyTickConstants(double ticksPerAdvancement, double ticksPerStrafe, double ticksPerTurn){
        this.ticksPerAdvancementCM = ticksPerAdvancement;
        this.ticksPerStrafeCM = ticksPerStrafe;
        this.ticksPerTurn = ticksPerTurn;
        return this;
    }

    public PathFinderMecanumDrive modifyEncoderSummation(boolean encoderReverseFL, boolean encoderReverseFR, boolean encoderReverseBL, boolean encoderReverseBR){
        this.encoderReverseFL = encoderReverseFL ? -1 : 1;
        this.encoderReverseFR = encoderReverseFR ? -1 : 1;
        this.encoderReverseBL = encoderReverseBL ? -1 : 1;
        this.encoderReverseBR = encoderReverseBR ? -1 : 1;
        return this;
    }

    public PathFinderMecanumDrive modifyPowerPartitionCoefficients(double nStrafeCo, double nAdvancementCo, double nTurnCo){
        this.strafeCo = nStrafeCo;
        this.advancementCo = nAdvancementCo;
        this.turnCo = nTurnCo;

        coefficientSum = Math.abs(nStrafeCo) + Math.abs(nAdvancementCo) + Math.abs(nTurnCo);

        this.powerPartitionMatrix = new Matrix4d(new double[][]{
                { strafeCo, advancementCo,  turnCo, 0}, //FL
                {-strafeCo, advancementCo,  turnCo, 0}, //BL
                {-strafeCo, advancementCo, -turnCo, 0}, //FR
                { strafeCo, advancementCo, -turnCo, 0}  //BR
        }).times(1/coefficientSum);

        return this;
    }

    @Override
    public PFPath makeNewPath(PFPose2d initialPose){
        return new PFPath(this, initialPose);
    }

    @Override
    public void setFollowingPath(PFPath nPath){
        this.followingPath = nPath;
    }

    @Override
    public void buildPath(PFPath path){
        Vector<float[]> output = new Vector<>();
        Vector<PFPose2d> poses = path.getPoseLookup();
        PFPose2d currentPose, targetPose;
        float[] valueCache = new float[4], previousCache = new float[]{0, 0, 0, 0};
        float deltaValue;

        for (int i = 0; i < poses.size(); i++) {
            currentPose = poses.get(i);
            targetPose = (i + 1) >= poses.size() ? poses.get(i) : poses.get(i + 1);

            deltaValue = 0; //zero deltas out to prepare
            deltaValue += targetPose.pos.minus(currentPose.pos).length() / calcNewMovementCoefficient(currentPose, targetPose);
            deltaValue += calcAngleDelta(currentPose, targetPose) * this.ticksPerTurn;

            //Add deltas
            //TODO: Tweak the multiplier of the deltas with actual data
            valueCache[0] = previousCache[0] + deltaValue * this.encoderReverseFL; //fl
            valueCache[1] = previousCache[1] + deltaValue * this.encoderReverseFR; //fr
            valueCache[2] = previousCache[2] + deltaValue * this.encoderReverseBL; //bl
            valueCache[3] = previousCache[3] + deltaValue * this.encoderReverseBR; //br

            //swap
            previousCache[0] = valueCache[0];
            previousCache[1] = valueCache[1];
            previousCache[2] = valueCache[2];
            previousCache[3] = valueCache[3];

            //add to encoder value stack
            output.add(valueCache);
        }

        path.setEncoderValues(output);
    }

    private double calcNewMovementCoefficient(PFPose2d current, PFPose2d target){
        if ((float) target.pos.minus(current.pos).length() == 0) return 0; //force truncation to happen
        Vector2d delta = current.toRelativeAxis(target.pos); // get target position relative to current position and rotation

        //Ellipsis-Line intersection math to find the new coefficient with the ticksPerAdvancementCM and ticksPerStrafeCM being the axes of the ellipses
        //Desmos Graph: https://www.desmos.com/calculator/kajouckd92
        double theta = Math.acos(Vector2d.AXIS_Y.times(delta)) * Math.signum(Vector2d.AXIS_X.times(delta) != 0 ? Math.signum(Vector2d.AXIS_X.times(delta)) : 1); //enforce CCW angles
        double axisA = ticksPerAdvancementCM * Math.sin(theta);
        double axisB = ticksPerStrafeCM * Math.cos(theta);
        double output = (ticksPerAdvancementCM * ticksPerStrafeCM) / (Math.sqrt(axisA * axisA + axisB * axisB));

        return Math.abs(output); //we need the absolute value because these measurements need to be the denominator for the length
    }

    private double calcAngleDelta(PFPose2d current, PFPose2d target){
        Vector2d currentDir = current.getDir(), targetDir = target.getDir(),
                 currentNormal = current.getNormal();
        return Math.acos(currentDir.times(targetDir)) * Math.signum(targetDir.times(currentNormal)) != 0 ? Math.signum(targetDir.times(currentNormal)) : 1;
    }

    @Override
    protected void setupInternals() {
        setupImu();
    }

    @Override
    public void followPath() {
        followingPath.updatePathProgress(fl.getCurrentPosition(),
                                         fr.getCurrentPosition(),
                                         bl.getCurrentPosition(),
                                         br.getCurrentPosition());
        float[] targetEncoderValues = followingPath.getCurrentEncoderValues();
        fl.setTargetPosition((int) targetEncoderValues[0]);
        fr.setTargetPosition((int) targetEncoderValues[1]);
        bl.setTargetPosition((int) targetEncoderValues[2]);
        br.setTargetPosition((int) targetEncoderValues[3]);

        //TODO: Check if the power values in followPath() work for intended application
        fl.setPower(1);
        fr.setPower(1);
        bl.setPower(1);
        br.setPower(1);
    }

    /**
     * Args are Vector3d where X: gamepad analog stick X, Y: gamepad analog stick Y, Z: whatever controls turn velocity
     * @param args
     */
    @Override
    public void joystickControl(Object... args) {
        Vector3d control = (Vector3d) args[0];
        boolean fieldCentric = (boolean) args[1];

        //create Vector4d 'in' from the passed in Vector3d(forward, strafe, turn)'s x, y, z, and an arbitrary w value
        //divide the input by the ratio found by max(|forward| + |strafe| + |turn|, 1)
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        Matrix3d rot = fieldCentric ? Matrix3d.makeAffineRotation(-angles.firstAngle) : new Matrix3d();
        Vector3d rotated = rot.times(control.unaryMinus());
        Vector4d internalControl = new Vector4d(rotated.x, rotated.y, rotated.z, 1);
        Vector4d out = powerPartitionMatrix.times(internalControl).div(Math.max(coefficientSum, 1));

        //set the motor powers as referenced in the hashmap
        fl.setPower(out.x);
        bl.setPower(out.y);
        fr.setPower(out.z);
        br.setPower(out.w);
    }


}
