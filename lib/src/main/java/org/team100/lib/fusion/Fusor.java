package org.team100.lib.fusion;

import org.team100.lib.uncertainty.VariableR1;

/**
 * Pooling for uncertain variables.
 */
public interface Fusor {
    VariableR1 fuse(VariableR1 a, VariableR1 b);
}
