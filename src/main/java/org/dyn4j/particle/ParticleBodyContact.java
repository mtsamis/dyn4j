package org.dyn4j.particle;

import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Vector2;

public class ParticleBodyContact {
  /** Index of the particle making contact. */
  public int index;
  /** The body making contact. */
  public Body body;
  /** Weight of the contact. A value between 0.0f and 1.0f. */
  float weight;
  /** The normalized direction from the particle to the body. */
  public final Vector2 normal = new Vector2();
  /** The effective mass used in calculating force. */
  float mass;
}
