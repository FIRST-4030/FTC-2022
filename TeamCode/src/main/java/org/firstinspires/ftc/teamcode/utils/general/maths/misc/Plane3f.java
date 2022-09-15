package org.firstinspires.ftc.teamcode.utils.general.maths.misc;

import org.firstinspires.ftc.teamcode.utils.general.maths.misc.depreciated.vectors.DepreciatedVector3F;

public class Plane3f {

    private DepreciatedVector3F normal, position;

    //these are the preset planes for the cardinal axes X, Y, Z

    //Make a plane resting along the XZ axes
    public static Plane3f XZ_PLANE = new Plane3f(new DepreciatedVector3F(0, 1, 0), new DepreciatedVector3F(0, 0, 0));

    //Make a plane resting along the YZ axes
    public static Plane3f YZ_PLANE = new Plane3f(new DepreciatedVector3F(1, 0, 0), new DepreciatedVector3F(0, 0, 0));

    //Make a plane resting along the XY axes
    public static Plane3f XY_PLANE = new Plane3f(new DepreciatedVector3F(0, 0, 1), new DepreciatedVector3F(0, 0, 0));

    public Plane3f(){
        this.normal = new DepreciatedVector3F(0, 1, 0);
        this.position = new DepreciatedVector3F(0, 0, 0);
    }

    public Plane3f(DepreciatedVector3F normal, DepreciatedVector3F position){
        this.normal = normal;
        this.position = position;
    }

    /**
     * This will do the math of vector-plane intersection in the 3rd dimension
     * @return position of the intersection
     */
    public DepreciatedVector3F getVector3fInt(DepreciatedVector3F lineStart, DepreciatedVector3F lineEnd){
        float plane_d = -DepreciatedVector3F.dot(this.normal, this.position);
        float ad = DepreciatedVector3F.dot(lineStart, this.normal);
        float bd = DepreciatedVector3F.dot(lineEnd, this.normal);
        float t = (-plane_d - ad) / (bd - ad);
        DepreciatedVector3F intersection = MathEx.lerp(lineStart, lineEnd, t);
        return intersection;
    }

    public void setNormal(DepreciatedVector3F newNormal){
        this.normal = newNormal.normalized(); //pass a normalized version of the input vector, does not mutate it
    }

    public void setPosition(DepreciatedVector3F newPosition){
        this.position = newPosition; //unlike setting the normal, it does not normalized because it is a transformation vector
    }

    public DepreciatedVector3F getNormal(){
        return this.normal;
    }

    public DepreciatedVector3F getPosition(){
        return this.position;
    }
}
