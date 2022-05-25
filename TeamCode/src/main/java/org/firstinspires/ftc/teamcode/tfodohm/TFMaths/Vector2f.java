package org.firstinspires.ftc.teamcode.tfodohm.TFMaths;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.io.IOException;

public class Vector2f {
    public float x, y;

    public Vector2f(){
        this.x = 0;
        this.y = 0;
    }

    public Vector2f(float x, float y){
        this.x = x;
        this.y = y;
    }

    public Vector2f(float[] list){
        if (list.length == 2) {
            this.x = list[0];
            this.y = list[1];
        } else {
            IOException e = new IOException("Array length is not 2!");
            e.printStackTrace();
        }
    }

    public float getX(){
        return this.x;
    }

    public float getY(){
        return this.y;
    }

    public void setX(float nx){ this.x = nx; }

    public void setY(float ny){ this.y = ny; }

    public float length(){
        return (float) Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public void normalize(){
        float l = this.length();
        this.x /= l;
        this.y /= l;
    }

    public Vector2f normalized(){
        float l = this.length();
        float nx = this.x / l;
        float ny = this.y / l;
        return new Vector2f(nx, ny);
    }

    public void add(Vector2f b){
        this.x += b.x;
        this.y += b.y;
    }

    public void sub(Vector2f b){
        this.x -= b.x;
        this.y -= b.y;
    }

    public void mul(float b){
        this.x *= b;
        this.y *= b;
    }

    public void div(float b){
        this.x /= b;
        this.y /= b;
    }

    public float dot(Vector2f b){
        return this.x * b.x + this.y * b.y;
    }

    public static Vector2f add(Vector2f a, Vector2f b){
        return new Vector2f(a.x + b.x ,a.y + b.y);
    }

    public static Vector2f sub(Vector2f a, Vector2f b){
        return new Vector2f(a.x - b.x ,a.y - b.y);
    }

    public static Vector2f mul(Vector2f a, float b){
        return new Vector2f(a.x * b, a.y * b);
    }

    public static Vector2f div(Vector2f a, float b){
        return new Vector2f(a.x / b, a.y / b);
    }

    public static float dot(Vector2f a, Vector2f b){
        return a.x * b.x + a.y * b.y;
    }

    public Vector2d getAsVector2d(){
        return new Vector2d(this.x, this.y);
    }

    @Override
    public String toString(){
        return ("X: " + this.x + " Y: " + this.y);
    }
}
