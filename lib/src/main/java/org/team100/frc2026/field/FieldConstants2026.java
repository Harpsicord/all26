package org.team100.frc2026.field;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * These measurements are from Onshape.
 * 
 * https://cad.onshape.com/documents/8a691e28680da30504859fce/w/c6aa636fb23edb3f1e272fb1/e/c043973ea96914e2eaa1e8fc
 */
public class FieldConstants2026 {
    /** Field width in meters */
    private static final double FIELD_WIDTH = 8.069;
    private static final double FIELD_LENGTH = 16.541;
    /**
     * Center of the hub target, inside the cone.
     * 
     * Use this for aiming.
     * 
     * Height is the bottom of the cone.
     */
    public static final Translation3d HUB = new Translation3d(
            4.626, 4.035, 1.434);
    /**
     * The rectangular base of the hub.
     * 
     * Avoid hitting this when lobbing.
     */
    public static final Rectangle2d HUB_BASE = new Rectangle2d(
            new Translation2d(4.028, 3.438),
            new Translation2d(5.223, 4.631));
    /**
     * The rectangular base of the opponent's hub.
     * 
     * Avoid hitting this when lobbing.
     */
    public static final Rectangle2d OPPONENT_HUB_BASE = new Rectangle2d(
            new Translation2d(11.318, 3.438),
            new Translation2d(12.512, 4.631));
    /**
     * Area on the field where you're allowed to shoot.
     * 
     * Measured from the outside edge of the tape to the wall.
     * 
     * Does not include the bump.
     * 
     * G407: bumpers must overlap this zone, or major foul (15 pt).
     */
    public static final Rectangle2d ALLIANCE_ZONE = new Rectangle2d(
            new Translation2d(0.000, 0.000),
            new Translation2d(4.029, FIELD_WIDTH));
    /**
     * Middle zone of the field.
     * 
     * Measured between the faces of each hub.
     * 
     * Does not include the bump.
     */
    public static final Rectangle2d NEUTRAL_ZONE = new Rectangle2d(
            new Translation2d(5.223, 0.000),
            new Translation2d(11.318, FIELD_WIDTH));

    /**
     * Alliance zone on the opposite side of the field.
     * 
     * Measured from the outside edge of the tape to the wall.
     * 
     * Does not include the bump.
     */
    public static final Rectangle2d OPPONENT_ZONE = new Rectangle2d(
            new Translation2d(12.512, 0.000),
            new Translation2d(FIELD_LENGTH, FIELD_WIDTH));
}
