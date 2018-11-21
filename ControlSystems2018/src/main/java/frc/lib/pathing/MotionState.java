package frc.lib.pathing;

import frc.lib.util.Util;

/** A MotionState represents the linear motion, angular motion, wheel states,
 * and curvature of the robot's drive train at a specific point in time.
 */
public class MotionState {
    public static final double length = 1;
    public final Vec2 pos;
    public final double t, angle, angVel, angAcc, vel, acc, velL, velR, accL, accR, curvature;
    private boolean kinematicsComputed;
    public MotionState(double t, 
                    Vec2 pos, double vel, double acc, 
                    double angle, double angVel, double angAcc,
                    double velL, double velR, double accL, double accR,
                    double curvature){
        kinematicsComputed = false;
        this.t = t;
        this.pos = pos;
        this.vel = vel;
        this.acc = acc;
        this.angle = angle;
        this.angVel = angVel;
        this.angAcc = angAcc;
        this.velL = velL;
        this.velR = velR;
        this.accL = accL;
        this.accR = accR;
        this.curvature = curvature;
    }

    /** Returns a new MotionState, inferring wheel states and curvature
     * from known angular motion.
     */
    public static MotionState fromAngular(double t,
                    Vec2 pos, double vel, double acc,
                    double angle, double angVel, double angAcc){
        return new MotionState(
            t,
            pos, vel, acc,
            angle, angVel, angAcc,
            vel - 0.5*angVel*length,    // left wheel velocity
            vel + 0.5*angVel*length,    // right wheel velocity
            acc - 0.5*angAcc*length,    // left wheel acceleration
            acc + 0.5*angAcc*length,    // right wheel acceleration
            angVel / vel                // curvature
        );
    }

    /** Returns a new MotionState, replacing the old one's accelerations by using
     * the linear and angular acceleration parameters.
     */
    public static MotionState controlAngular(MotionState old, double acc, double angAcc){
        return fromAngular(
            old.t,
            old.pos, old.vel, acc,
            old.angle, old.angVel, angAcc
        );
    }

    /** Returns a new MotionState, replacing the old one's accelerations by using
     * the linear and angular acceleration parameters.
     */
    public MotionState controlAngular(double acc, double angAcc){
        return controlAngular(this, acc, angAcc);
    }

    /** Returns a new MotionState, inferring angular motion and curvature
     * from known wheel states.
     */
    public static MotionState fromWheels(double t,
                        Vec2 pos, double angle,
                        double velL, double velR,
                        double accL, double accR){
        return new MotionState(
            t,
            pos, (velR+velL)/2, (accR+accL)/2,              // linear velocity and acceleration
            angle, (velR-velL)/length, (accR-accL)/length,  // angular velocity and acceleration
            velL, velR, accL, accR,
            (2*(velR-velL))/(length*(velR+velL))            // curvature
        );
    }

    /** Returns a new MotionState, replacing the old one's accelerations by using
     * the wheel accelerations parameters.
     */
    public static MotionState controlWheels(MotionState old, double accL, double accR){
        return fromWheels(
            old.t,
            old.pos, old.angle, 
            old.velL, old.velR, 
            accL, accR
        );
    }

    /** Returns a new MotionState, replacing the old one's accelerations by using
     * the wheel accelerations parameters.
     */
    public MotionState controlWheels(double accL, double accR){
        return controlWheels(this, accL, accR);
    }

    private static final double sqrtPi = Math.sqrt(Math.PI);
    private double a_aA, sqrtAngAcc, scalar, sinS, cosS, o, inner0;

    /** Calculates and returns the new MotionState of the robot after driving for some time, 
     * given its previous state and the change in time.
     * @param start The previous MotionState.
     * @param dt Amount of time elapsed between the old and new MotionStates.
     * @return The new MotionState.
     */
    public static MotionState forwardKinematics(MotionState start, double dt){
        if(Util.epsilonEquals(dt, 0)) return start;

        double vel = start.acc*dt + start.vel, angVel = start.angAcc*dt + start.angVel;
        double angle = 0.5*start.angAcc*dt*dt + start.angVel*dt + start.angle;

        /* To calculate the change in x and y based on the given wheel accelerations and
         * the previous state, we must integrate over the x- and y-velocity functions:
         * v_x(t) = v(t)*cos(angle(t))  and  v_y(t) = v(t)*sin(angle(t)) .
         */
        double dx, dy;
        if(Util.epsilonEquals(start.angAcc, 0)){
            if(Util.epsilonEquals(start.angVel, 0)){
                // Integrating: (a*t + v_0)*cos(angle_0)
                double ds = 0.5*start.acc*dt*dt + start.vel*dt;
                dx = ds*Math.cos(start.angle);
                dy = ds*Math.sin(start.angle);
            } else {
                // Integrating: (a*t + v_0)*cos(angVel_0*t + angle_0)
                double inv_av = 1/start.angVel;
                double one = inv_av*(start.acc*dt + start.vel), two = start.acc*inv_av*inv_av;
                double sin = Math.sin(angle) - Math.sin(start.angle);
                double cos = Math.cos(angle) - Math.cos(start.angle);
                dx = one*sin + two*cos;
                dy = two*sin - one*cos;
            }
        } else {
            // Integrating: (a*t + v_0)*cos(0.5*angAcc*t*t + angVel_0*t + angle_0)
            if(!start.kinematicsComputed){
                start.a_aA = start.acc/start.angAcc;
                start.sqrtAngAcc = Math.sqrt(Math.abs(start.angAcc));
                start.scalar = (sqrtPi/start.sqrtAngAcc)*(start.a_aA*start.angVel - start.vel);
                double trigInner = start.angle - start.angVel*start.angVel/(2*start.angAcc);
                start.sinS = Math.sin(trigInner);
                start.cosS = Math.cos(trigInner);
                start.o = Math.signum(start.angAcc);
                start.inner0 = start.o*start.angVel/(sqrtPi*start.sqrtAngAcc);

                start.kinematicsComputed = true;
            }

            double innerT = dt*(start.sqrtAngAcc/sqrtPi) + start.inner0;
            double fresnelC = Util.fresnelC(innerT) - Util.fresnelC(start.inner0);
            double fresnelS = start.o*(Util.fresnelS(innerT) - Util.fresnelS(start.inner0));
            
            dx =  start.a_aA*(Math.sin(angle)-Math.sin(start.angle)) 
                + start.scalar*(start.sinS*fresnelS - start.cosS*fresnelC);
            dy = -start.a_aA*(Math.cos(angle)-Math.cos(start.angle)) 
                - start.scalar*(start.sinS*fresnelC + start.cosS*fresnelS);
        }

        Vec2 pos = new Vec2(dx,dy).add(start.pos);

        return MotionState.fromAngular(
            start.t + dt,
            pos, vel, start.acc,
            angle, angVel, start.angAcc
        );
    }

    /** Calculates and returns the new MotionState of the robot after driving for some time, 
     * given its previous state and the change in time.
     * @param dt Amount of time elapsed between the old and new MotionStates.
     * @return The new MotionState.
     */
    public MotionState forwardKinematics(double dt){
        return MotionState.forwardKinematics(this, dt);
    }

    /*
    public static MotionState forwardKinematicsCurvature(MotionState prev, double dt, double acc, double curvature){
        double k = curvature*length/4;
        double accR = (k-0.5)*(prev.velR/dt) + (k+0.5)*(2*acc + prev.velL/dt);
        double accL = 2*acc - accR;
        return forwardKinematicsWheels(prev, dt, accL, accR);
    }
    */

}