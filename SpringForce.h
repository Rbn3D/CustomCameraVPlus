#pragma once
class SpringForce {
public:
    double 
        gravity,
        mass, 
        position,
        velocity,
        anchor, // target
        k,
        damping;

    SpringForce() {
        gravity = 1;
        mass = 1;
        position = 0;
        velocity = 0;
        anchor = 0;
        k = 18;
        damping = 16;
    }

    double Update(double target, double dt) {

        anchor = target;

        double springForce = -k * (position - anchor);
        double dampingForce = damping * velocity;
        double force = springForce + mass * gravity - dampingForce;
        double acceleration = force / mass;
        velocity = velocity + acceleration * dt;
        position = position + velocity * dt;

        return position;
    }
};
