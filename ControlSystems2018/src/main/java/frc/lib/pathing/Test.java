package frc.lib.pathing;

import frc.lib.util.Util;

public class Test {
    public static void main(String[] args){
        MotionState startState = MotionState.fromWheels(0, new Vec2(0,0), 0, -0.3, -3.4, 0, 0);
        PathProfile profile = new PathProfile(startState);
        profile.appendControlCurvature(1, -1, -3.4);
        System.out.println(profile.getState(0.4).pos);
    }
}