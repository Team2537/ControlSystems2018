package frc.lib.pathing;

import java.util.ArrayList;
import java.util.List;

public class PathSegment {
    private List<MotionState> states;
    private MotionState startState;
    private double dt, tTotal;

    public PathSegment(MotionState startState){
        this.states = new ArrayList<>();
        this.states.add(startState);
        this.startState = startState;
    }
    /*
    public MotionState stateByTime(double t){
        //for()
        int index = Math.max(Math.min((int)(t / dt), 1), 0);
        return MotionState.lerp(states.get(index), states.get(index+1), t);
    }
    */

}