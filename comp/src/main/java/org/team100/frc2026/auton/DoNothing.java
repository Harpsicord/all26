package org.team100.frc2026.auton;

import org.team100.lib.config.AnnotatedCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** An auton that does nothing at all. */
public class DoNothing implements AnnotatedCommand {

    @Override
    public String name() {
        return "Do Nothing";
    }

    @Override
    public Command command() {
        return Commands.idle().withName("Nothing from right bump");
    }

    @Override
    public Pose2d start() {
        return StartingPositions.RIGHT_BUMP;
    }
}
