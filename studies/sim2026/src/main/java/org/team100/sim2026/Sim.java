package org.team100.sim2026;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/** The main simulation loop. */
public class Sim {
    private int matchTimer;

    // 504 total
    final Zone redZone = new Zone("red", 24); // staged in the depot
    final Zone neutralZone = new Zone("mid", 360);
    final Zone blueZone = new Zone("blue", 24);// staged in the depot

    final Outpost redOutpost = new Outpost(redZone, 24);
    final Outpost blueOutpost = new Outpost(blueZone, 24);

    final Score redScore = new Score(this::phase);
    final Score blueScore = new Score(this::phase);

    final Hub redHub = new Hub(
            redScore,
            neutralZone,
            this::time,
            redScore::auto,
            blueScore::auto);
    final Hub blueHub = new Hub(
            blueScore,
            neutralZone,
            this::time,
            blueScore::auto,
            redScore::auto);

    final Robot red1 = new Robot(
            redZone, neutralZone, blueZone, redHub, 8, this::time);
    final Robot red2 = new Robot(
            redZone, neutralZone, blueZone, redHub, 8, this::time);
    final Robot red3 = new Robot(
            redZone, neutralZone, blueZone, redHub, 8, this::time);
    final Robot blue1 = new Robot(
            blueZone, neutralZone, redZone, blueHub, 8, this::time);
    final Robot blue2 = new Robot(
            blueZone, neutralZone, redZone, blueHub, 8, this::time);
    final Robot blue3 = new Robot(
            blueZone, neutralZone, redZone, blueHub, 8, this::time);

    final List<BallContainer> containers;
    final List<Actor> actors;

    // set after auto
    Alliance firstActive;

    public Sim() {
        containers = List.of(redZone, neutralZone, blueZone,
                redHub, blueHub,
                redOutpost, blueOutpost,
                red1, red2, red3, blue1, blue2, blue3);
        actors = List.of(redHub, blueHub,
                redOutpost, blueOutpost,
                red1, red2, red3, blue1, blue2, blue3);
        System.out.printf("initial total %d\n", total());
    }

    private int total() {
        return containers.stream().map(BallContainer::count).reduce(0, Integer::sum);
    }

    public void run() {
        // note 10 seconds longer this year
        int MATCH_LENGTH_SEC = 160;
        header();
        for (matchTimer = 0; matchTimer < MATCH_LENGTH_SEC; ++matchTimer) {
            balls();
            updateActiveHubs();
            List<Runnable> actions = new ArrayList<>();
            for (Actor actor : actors) {
                actions.add(actor.step());
            }
            for (Runnable runnable : actions) {
                runnable.run();
            }
        }
        balls();
        score();
    }

    GamePhase phase() {
        return GamePhase.at(matchTimer);
    }

    int time() {
        return matchTimer;
    }

    void red(boolean active) {
        redHub.active = active;
        red1.active = active;
        red2.active = active;
        red3.active = active;
    }

    void blue(boolean active) {
        blueHub.active = active;
        blue1.active = active;
        blue2.active = active;
        blue3.active = active;
    }

    void updateActiveHubs() {
        // choose which hub is active in the first shift
        if (matchTimer == 20) {
            // this should only happen once per match
            if (blueScore.autoFuel > redScore.autoFuel) {
                firstActive = Alliance.RED;
            } else if (blueScore.autoFuel < redScore.autoFuel) {
                firstActive = Alliance.BLUE;
            } else {
                firstActive = new Random().nextBoolean() ? Alliance.RED : Alliance.BLUE;
            }
        }
        // then set the active hubs
        switch (phase()) {
            case AUTO -> {
                red(true);
                blue(true);
            }
            case TRANSITION -> {
                red(true);
                blue(true);
            }
            case SHIFT_1 -> {
                boolean redActive = firstActive == Alliance.RED;
                red(redActive);
                blue(!redActive);
            }
            case SHIFT_2 -> {
                boolean redActive = firstActive == Alliance.BLUE;
                red(redActive);
                blue(!redActive);
            }
            case SHIFT_3 -> {
                boolean redActive = firstActive == Alliance.RED;
                red(redActive);
                blue(!redActive);
            }
            case SHIFT_4 -> {
                boolean redActive = firstActive == Alliance.BLUE;
                red(redActive);
                blue(!redActive);
            }
            case END_GAME -> {
                red(true);
                blue(true);
            }
        }
    }

    /** print the ball location header */
    void header() {
        System.out.print(
                "matchTimer, total, redZone, neutralZone, blueZone, redHub, blueHub, redOutpost, blueOutpost, ");
        System.out.print(
                " r1l, red1,  r2l, red2,  r3l, red3,  b1l, blue1,  b2l, blue2,  b3l, blue3, ");
        System.out.print(" ract,  bact\n");
    }

    /** print the ball locations */
    void balls() {
        System.out.printf("%10d, %5d, %7d, %11d, %8d, %6d, %7d, %10d, %11d, ",
                matchTimer, total(),
                redZone.count(), neutralZone.count(), blueZone.count(),
                redHub.count(), blueHub.count(),
                redOutpost.count(), blueOutpost.count());

        System.out.printf("%4s, %4d, %4s, %4d, %4s, %4d, %4s, %5d, %4s, %5d, %4s, %5d, ",
                red1.location.name, red1.count(), red2.location.name, red2.count(), red3.location.name, red3.count(),
                blue1.location.name, blue1.count(), blue2.location.name, blue2.count(), blue3.location.name,
                blue3.count());
        System.out.printf("%5b, %5b\n",
                redHub.active, blueHub.active);
    }

    void score() {
        System.out.printf("RED\n  %s\nBLUE\n  %s\n", redScore, blueScore);
    }

}
