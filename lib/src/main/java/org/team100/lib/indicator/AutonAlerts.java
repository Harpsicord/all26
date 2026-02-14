package org.team100.lib.indicator;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.geometry.Metrics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

public class AutonAlerts implements Runnable {
    private final Supplier<AnnotatedCommand> m_autons;
    private final Supplier<Pose2d> m_robotPose;
    private final Consumer<Pose2d> m_poseSetter;
    private final Alert m_noStartingPosition;
    private final Alert m_mismatchedAlliance;

    public AutonAlerts(
            Supplier<AnnotatedCommand> autons,
            Alerts alerts,
            Supplier<Pose2d> robotPose,
            Consumer<Pose2d> poseSetter) {
        m_autons = autons;
        m_robotPose = robotPose;
        m_poseSetter = poseSetter;
        m_noStartingPosition = alerts.add("No starting position!", AlertType.kWarning);
        m_mismatchedAlliance = alerts.add("Wrong Alliance!", AlertType.kWarning);
    }

    @Override
    public void run() {
        checkStart();
        checkAlliance();
    }

    private void checkStart() {
        AnnotatedCommand cmd = m_autons.get();
        if (cmd == null)
            return;
        Pose2d start = cmd.start();
        if (start == null) {
            m_noStartingPosition.set(true);
        } else {
            m_noStartingPosition.set(false);
            if (RobotBase.isReal()) {
                // for a real robot, we complain if we're far from the correct place.
                Pose2d pose = m_robotPose.get();
                double r = Metrics.doubleGeodesicDistance(start, pose);
                m_noStartingPosition.setText(null);
            } else {
                // in simulation we force the position
                m_poseSetter.accept(start);
            }
        }
    }

    private void checkAlliance() {
        AnnotatedCommand cmd = m_autons.get();
        if (cmd == null)
            return;
        Alliance alliance = cmd.alliance();
        if (alliance == null) {
            // works for either
            return;
        }
        Alliance dsAlliance = DriverStation.getAlliance().orElse(null);
        if (dsAlliance == null) {
            // not set yet
            return;
        }
        if (alliance != dsAlliance) {
            m_mismatchedAlliance.set(true);
        } else {
            m_mismatchedAlliance.set(false);
        }
    }

}
