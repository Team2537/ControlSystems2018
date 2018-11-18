package frc.lib.pathing;

import frc.lib.util.Util;

public class Test {
    public static void main(String[] args){
        MotionState state = MotionState.fromWheels(0, new Vec2(0,0), 0, -1, 1, 0, 0);
        MotionState state2 = MotionState.forwardKinematicsCurvature(state, 1, 1, 4);
        //System.out.println(state2.angAcc);
        System.out.println(state2.pos);

        MotionState state3 = MotionState.fromWheels(0, new Vec2(0,0), 0, 1, -1, 0, 0);
        MotionState state4 = MotionState.forwardKinematicsCurvature(state3, 1, 1, -4);
        System.out.println(state4.pos);
    }
}