package frc.lib.pathing;

public class Vec2 {
    public final double x,y;
    private double mag = -1;
    public Vec2(double x, double y){
        this.x = x;
        this.y = y;
    }
    public static Vec2 add(Vec2 a, Vec2 b){
        return new Vec2(a.x + b.x, a.y + b.y);
    }
    public Vec2 add(Vec2 other){
        return add(this, other);
    }
    public static Vec2 scale(Vec2 v, double k){
        return new Vec2(v.x * k, v.y * k);
    }
    public Vec2 scale(double k){
        return scale(this, k);
    }
    public static Vec2 inverse(Vec2 v){
        return v.scale(-1);
    }
    public Vec2 inverse(){
        return inverse(this);
    }
    public static Vec2 diff(Vec2 a, Vec2 b){
        return add(a,b.inverse());
    }
    public Vec2 diff(Vec2 other){
        return diff(this, other);
    }
    public static double mag(Vec2 v){
        if(v.mag == -1){
            v.mag = Math.hypot(v.x, v.y);
        }
        return v.mag;
    }
    public double mag(){
        return mag(this);
    }

    @Override
    public String toString(){
        return "("+x+", "+y+")";
    }
}