package org.team100.lib.reference.r1;

import java.util.function.Supplier;

import org.team100.lib.coherence.Takt;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;

/**
 * Produces "next" and "current" references from a supplier, using the clock to
 * keep track of whether it's time to fetch a new one.
 */
public class ReferenceR1 {

    private final Supplier<ControlR1> m_setpoints;
    boolean done;
    double currentInstant;
    ControlR1 currentSetpoint;
    ControlR1 nextSetpoint;

    /** When we notice the clock has advanced */
    void update() {
        double t = Takt.get();
        if (t == currentInstant) {
            // we already did it
            return;
        }
        currentInstant = t;
        currentSetpoint = nextSetpoint;
        nextSetpoint = m_setpoints.get();
    }

    public ReferenceR1(Supplier<ControlR1> setpoints) {
        m_setpoints = setpoints;
    }

    public void initialize(ModelR1 measurement) {
        // do we really care about the measurement?
        done = false;
        currentInstant = Takt.get();
    }

    /** Desired state at the current instant, used for feedback. */
    public ControlR1 current() {
        double t = Takt.get();
        if (t == currentInstant)
            return currentSetpoint;
        update();
        return currentSetpoint;
    }

    /** Desired state at the next instant, used for feedforward. */
    public ControlR1 next() {
        double t = Takt.get();
        if (t == currentInstant)
            return currentSetpoint;
        update();
        return nextSetpoint;
    }

    public boolean done() {
        return done;
    }
}
