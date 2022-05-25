package org.firstinspires.ftc.teamcode.utils;

public class Pair<A, B>{

    private A first;
    private B second;

    public Pair(A first, B second){
        this.first = first;
        this.second = second;
    }

    public void setFirst(A nFirst){
        this.first = nFirst;
    }

    public void setSecond(B nSecond){
        this.second = nSecond;
    }

    public void setPair(A nFirst, B nSecond){
        this.first = nFirst;
        this.second = nSecond;
    }

    public A getFirst(){
        return this.first;
    }

    public B getSecond(){
        return this.second;
    }

    public void dispose(){
        this.first = null;
        this.second = null;
    }
}
