package org.team100.sim2026;

public enum GamePhase {
    /** 20 s */
    AUTO,
    /** 10 s */
    TRANSITION,
    /** 25 s */
    SHIFT_1,
    /** 25 s */
    SHIFT_2,
    /** 25 s */
    SHIFT_3,
    /** 25 s */
    SHIFT_4,
    /** 30 s */
    END_GAME;

    static GamePhase at(int time) {
        if (time < 20)
            return AUTO;
        if (time < 30)
            return TRANSITION;
        if (time < 55)
            return SHIFT_1;
        if (time < 80)
            return SHIFT_2;
        if (time < 105)
            return SHIFT_3;
        if (time < 130)
            return SHIFT_4;
        return END_GAME;
    }
}
