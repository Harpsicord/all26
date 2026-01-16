package org.team100.sim2026.actions;

public class Move implements Action {

    private final String destination;

    public Move(String destination) {
        this.destination = destination;
    }

    @Override
    public String toString() {
        return "move to " + destination ;
    }
    

}
