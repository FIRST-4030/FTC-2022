package org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.servos.kinematics;

import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.EULMathEx;
import org.firstinspires.ftc.teamcode.extrautilslib.core.maths.vectors.Vector2d;
import org.firstinspires.ftc.teamcode.robot.powerplay2022.utilities.production.servos.ServoAngleConversion;
import org.firstinspires.ftc.teamcode.utils.actuators.ServoFTC;

public class ThreeJointArm {

    private VirtualServo virtualServoA, virtualServoB, virtualServoC;
    private ServoFTC servoA, servoB, servoC;
    private ServoAngleConversion conversionA, conversionB, conversionC;
    private final double armLengthA, armLengthB, totalArmLength;

    public ThreeJointArm(VirtualServo virtualCore, ServoFTC[] servos, ServoAngleConversion[] conversions, double armLengthA, double armLengthB){
        //check if params are correct
        if (servos.length != 3) throw new IllegalArgumentException("Servo Array is not length 3! Length passed in: " + servos.length);
        if (conversions.length != 3) throw new IllegalArgumentException("Conversion Array is not length 3! Length passed in: " + conversions.length);

        //assign segment lengths
        this.armLengthA = armLengthA;
        this.armLengthB = armLengthB;
        this.totalArmLength = armLengthA + armLengthB;

        //assign conversions
        this.conversionA = conversions[0];
        this.conversionB = conversions[1];
        this.conversionC = conversions[2];

        //assign hardware servos
        this.servoA = servos[0];
        this.servoB = servos[1];
        this.servoC = servos[2];

        //assign virtual servos
        this.virtualServoA = virtualCore;
        this.virtualServoB = new VirtualServo(this.virtualServoA, new Vector2d(), armLengthB);
        this.virtualServoC = new VirtualServo(this.virtualServoB, new Vector2d(), 0);
    }

    public void propagate(Vector2d target, Vector2d endHeading, boolean bottomSolution){
        //store the length
        double targetLength = target.length();
        //stores the restricted target and direction to it
        Vector2d restrictedTarget = targetLength <= totalArmLength ? target : target.normalized().times(totalArmLength);
        Vector2d targetDir = restrictedTarget.normalized();

        //stores the end heading
        Vector2d normalizedHeading = endHeading.length() <= 0.0000001 ? new Vector2d(0, -1) : endHeading.normalized();

        double biasedSign = Math.signum(virtualServoA.armDirectionNormal.times(targetDir)) == 0 ? 1 : Math.signum(virtualServoA.armDirectionNormal.times(targetDir));
        double angleAVirtual = Math.acos(virtualServoA.armDirection.times(targetDir)) * biasedSign; //angle from the arm's virtual base

        double angleA = Math.acos(virtualServoA.forward.times(targetDir));
        if (bottomSolution){
            angleA -= EULMathEx.lawOfCosines(this.armLengthA, targetLength, armLengthB);
        } else {
            angleA += EULMathEx.lawOfCosines(this.armLengthA, targetLength, armLengthB);
        }

        //rotate and actuate virtual and hardware servo (respectively)
        conversionA.angle2Scalar(angleA + (Math.PI / 2));
        virtualServoA.rotateArm(angleAVirtual);
        servoA.setPosition(conversionA.getOutput());

        Vector2d relativeBTarget = restrictedTarget.minus(virtualServoB.position).normalized();

        double biasedSignBVirtual = Math.signum(virtualServoB.armDirectionNormal.times(relativeBTarget)) == 0 ? 1 : Math.signum(virtualServoB.armDirectionNormal.times(relativeBTarget));
        double biasedSignBActual = Math.signum(virtualServoB.right.times(relativeBTarget)) == 0 ? 1 : Math.signum(virtualServoB.right.times(relativeBTarget));
        double angleBVirtual = Math.acos(virtualServoB.armDirection.times(relativeBTarget)) * biasedSignBVirtual;
        double angleBActual = Math.acos(virtualServoB.forward.times(relativeBTarget)) * biasedSignBActual;

        //rotate and actuate virtual and hardware servo (respectively)
        conversionB.angle2Scalar(3 * Math.PI / 4 - angleBActual);
        virtualServoB.rotateArm(angleBVirtual);
        servoB.setPosition(conversionB.getOutput());

        double biasedSignCVirtual = Math.signum(virtualServoB.armDirectionNormal.times(normalizedHeading)) == 0 ? 1 : Math.signum(virtualServoB.armDirectionNormal.times(normalizedHeading));
        double biasedSignCActual = Math.signum(virtualServoB.right.times(normalizedHeading)) == 0 ? 1 : Math.signum(virtualServoB.right.times(normalizedHeading));
        double angleCVirtual = Math.acos(virtualServoB.armDirection.times(normalizedHeading)) * biasedSignBVirtual;
        double angleCActual = Math.acos(virtualServoB.forward.times(normalizedHeading)) * biasedSignBActual;

        //rotate and actuate virtual and hardware servo (respectively)
        conversionC.angle2Scalar(3 * Math.PI / 4 - angleBActual);
        virtualServoC.rotateArm(angleCVirtual);
        servoC.setPosition(conversionC.getOutput());
    }
}
