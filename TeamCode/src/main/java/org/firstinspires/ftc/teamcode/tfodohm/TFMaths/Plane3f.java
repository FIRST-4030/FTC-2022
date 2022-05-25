package org.firstinspires.ftc.teamcode.tfodohm.TFMaths;

public class Plane3f {

    private Vector3f normal, position;

    //these are the preset planes for the cardinal axes X, Y, Z

    //Make a plane resting along the XZ axes
    public static Plane3f XZ_PLANE = new Plane3f(new Vector3f(0, 1, 0), new Vector3f(0, 0, 0));

    //Make a plane resting along the YZ axes
    public static Plane3f YZ_PLANE = new Plane3f(new Vector3f(1, 0, 0), new Vector3f(0, 0, 0));

    //Make a plane resting along the XY axes
    public static Plane3f XY_PLANE = new Plane3f(new Vector3f(0, 0, 1), new Vector3f(0, 0, 0));

    public Plane3f(){
        this.normal = new Vector3f(0, 1, 0);
        this.position = new Vector3f(0, 0, 0);
    }

    public Plane3f(Vector3f normal, Vector3f position){
        this.normal = normal;
        this.position = position;
    }

    /**
     * This will do the math of vector-plane intersection in the 3rd dimension
     * @return position of the intersection
     */
    public Vector3f getVector3fInt(Vector3f lineStart, Vector3f lineEnd){
        float plane_d = -Vector3f.dot(this.normal, this.position);
        float ad = Vector3f.dot(lineStart, this.normal);
        float bd = Vector3f.dot(lineEnd, this.normal);
        float t = (-plane_d - ad) / (bd - ad);
        Vector3f intersection = TFMathExtension.lerp(lineStart, lineEnd, t);
        return intersection;
    }

    public void setNormal(Vector3f newNormal){
        this.normal = newNormal.normalized(); //pass a normalized version of the input vector, does not mutate it
    }

    public void setPosition(Vector3f newPosition){
        this.position = newPosition; //unlike setting the normal, it does not normalized because it is a transformation vector
    }

    public Vector3f getNormal(){
        return this.normal;
    }

    public Vector3f getPosition(){
        return this.position;
    }
}
