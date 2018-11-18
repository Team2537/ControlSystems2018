package frc.lib.pathing;

import frc.lib.util.Util;

/** A MotionState represents the linear motion, angular motion, wheel states,
 * and curvature of the robot's drive train at a specific point in time.
 */
public class MotionState {
    public static final double length = 1;
    public final Vec2 pos;
    public final double t, angle, angVel, angAcc, vel, acc, velL, velR, accL, accR, curvature;
    public MotionState(double t, 
                    Vec2 pos, double vel, double acc, 
                    double angle, double angVel, double angAcc,
                    double velL, double velR, double accL, double accR,
                    double curvature){
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
            vel - 0.5*angVel*length,
            vel + 0.5*angVel*length,
            acc - 0.5*angAcc*length,
            acc + 0.5*angAcc*length,
            angVel / vel
        );
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
            pos, (velR+velL)/2, (accR+accL)/2,
            angle, (velR-velL)/length, (accR-accL)/length,
            velL, velR, accL, accR,
            (2*(velR-velL))/(length*(velR+velL))
        );
    }

    /** Calculates and returns the new MotionState of the robot after driving for some time, 
     * given its previous state, the change in time, and constant wheel accelerations.
     * @param prev The previous MotionState.
     * @param dt Amount of time elapsed between the old and new MotionStates.
     * @param accL Constant acceleration of the left wheel during this period.
     * @param accR Constant acceleration of the right wheel during this period.
     * @return The new MotionState.
     */
    public static MotionState forwardKinematicsWheels(MotionState prev, double dt, double accL, double accR){
        double acc = (accR+accL)/2, angAcc = (accR-accL)/length;
        double vel = acc*dt + prev.vel, angVel = angAcc*dt + prev.angVel;
        double angle = 0.5*angAcc*dt*dt + prev.angVel*dt + prev.angle;

        /* To calculate the change in x and y based on the given wheel accelerations and
         * the previous state, we must integrate over the x- and y-velocity functions:
         * v_x(t) = v(t)*cos(angle(t))  and  v_y(t) = v(t)*sin(angle(t)) .
         */
        double dx, dy;
        if(Util.epsilonEquals(angAcc, 0)){
            if(Util.epsilonEquals(prev.angVel, 0)){
                // Integrating: (a*t + v_0)*cos(angle_0)
                double ds = 0.5*acc*dt*dt + prev.vel*dt;
                dx = ds*Math.cos(prev.angle);
                dy = ds*Math.sin(prev.angle);
            } else {
                // Integrating: (a*t + v_0)*cos(angVel_0*t + angle_0)
                double inv_av = 1/prev.angVel;
                double one = inv_av*(acc*dt + prev.vel), two = acc*inv_av*inv_av;
                double sin = Math.sin(angle) - Math.sin(prev.angle);
                double cos = Math.cos(angle) - Math.cos(prev.angle);
                dx = one*sin + two*cos;
                dy = two*sin - one*cos;
            }
        } else {
            // Integrating: (a*t + v_0)*cos(0.5*angAcc*t*t + angVel_0*t + angle_0)
            double a_aA = acc/angAcc;
            double sqrtPi = Math.sqrt(Math.PI);
            double sqrtAngAcc = Math.sqrt(Math.abs(angAcc));
            double scalar = (sqrtPi/sqrtAngAcc)*(a_aA*prev.angVel - prev.vel);

            double trigInner = prev.angle - prev.angVel*prev.angVel/(2*angAcc);
            double sin = Math.sin(trigInner), cos = Math.cos(trigInner);

            double o = Math.signum(angAcc);
            double inner0 = o*prev.angVel/(sqrtPi*sqrtAngAcc);
            double innerT = dt*(sqrtAngAcc/sqrtPi) + inner0;
            double fresnelC = Util.fresnelC(innerT) - Util.fresnelC(inner0);
            double fresnelS = o*(Util.fresnelS(innerT) - Util.fresnelS(inner0));
            
            dx =  a_aA*(Math.sin(angle)-Math.sin(prev.angle)) + scalar*(sin*fresnelS - cos*fresnelC);
            dy = -a_aA*(Math.cos(angle)-Math.cos(prev.angle)) - scalar*(sin*fresnelC + cos*fresnelS);
        }

        Vec2 pos = new Vec2(dx,dy).add(prev.pos);

        return MotionState.fromAngular(
            prev.t + dt,
            pos, vel, acc,
            angle, angVel, angAcc
        );
    }

    /** Calculates and returns the new MotionState of the robot after driving for some time, 
     * given its previous state, change in time, constant linear acceleration, and final
     * curvature.
     * @param prev The previous MotionState.
     * @param dt Amount of time elapsed between the old and new MotionStates.
     * @param acc Constant linear acceleration of the drive train during this period.
     * @param curvature The desired curvature of the robot at the end of the period.
     * @return The new MotionState.
     */
    public static MotionState forwardKinematicsCurvature(MotionState prev, double dt, double acc, double curvature){
        double k = curvature*length/4;
        double accR = (k-0.5)*(prev.velR/dt) + (k+0.5)*(2*acc + prev.velL/dt);
        double accL = 2*acc - accR;
        return forwardKinematicsWheels(prev, dt, accL, accR);
    }

    public static MotionState interp(MotionState a, MotionState b, double t){
        double dt = Util.clamp(t, a.t, b.t) - a.t;
        return forwardKinematicsWheels(a, dt, b.accL, b.accR);
    }

    public static Vec2 linearInterpPos(MotionState a, MotionState b, double t){
        t = (t - a.t)/(b.t - a.t);
        return Util.linearInterp(a.pos, b.pos, t);
    }
}