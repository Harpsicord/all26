package org.team100.frc2026;

import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Kraken6Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final BareMotor m_motor;

    public Intake(LoggerFactory parent,CanId canID) {
        LoggerFactory log = parent.type(this);

        switch (Identity.instance) {
            case TEST_BOARD_B0, COMP_BOT -> {
                PIDConstants PID = PIDConstants.makeVelocityPID(log, 1);
                m_motor = new Kraken6Motor(
                        log,    //LoggerFactory parent,
                        canID,  // CanId canId,
                        NeutralMode100.COAST, //NeutralMode neutral,
                        MotorPhase.REVERSE, //MotorPhase motorPhase,
                        2, // og 50 //double supplyLimit,
                        4, // og 2 //double statorLimit,
                        PID,  //PIDConstants pid,
                        Kraken6Motor.highFrictionFF(log)//Feedforward100 ff
                );
            }

            default -> {
                m_motor = new SimulatedBareMotor(log, 600);
            }
        }
    }

    @Override
    public void periodic() {
        m_motor.periodic();
    }

    public Command intake() {
        return run(this::fullSpeed);
    }

    public Command stop() {
        return run(this::stopMotor);
    }

    public void stopMotor() {
        m_motor.setDutyCycle(0);
    }

    private void fullSpeed() {
        m_motor.setDutyCycle(1);
    }

}
