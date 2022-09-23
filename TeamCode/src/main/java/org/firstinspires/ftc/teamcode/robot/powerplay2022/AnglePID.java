package org.firstinspires.ftc.teamcode.robot.powerplay2022;

public class AnglePID {
    public double currentTime, previousTime, currentError, previousError, correctionPower;
    private double Kp, Ki, Kd, i;

    public AnglePID(double KP, double KI, double KD){ //Kp should be 1/pi, Kd should be 0.00025, Ki ???
        currentTime = 0; currentError = 0; previousTime = 0; previousError = 0; i = 0;
        this.Kp = KP;
        this.Ki = KI;
        this.Kd = KD;
    }

    public void update(double deltaTime, double targetAngle, double currentAngle){
        previousTime = currentTime;
        currentTime += deltaTime;

        previousError = currentError;
        currentError = targetAngle - currentAngle;

        double p = Kp * currentError;

        i += Ki * (currentError * deltaTime);

        if ( i>1 ){ i=1; }
        if ( i<-1 ){ i=-1; }

        double d = Kd * (currentError - previousError) / deltaTime;

        correctionPower = p+i+d;

        if ( correctionPower>1 ){ correctionPower=1; }
        if ( correctionPower<-1 ){ correctionPower=-1; }
    }
}
