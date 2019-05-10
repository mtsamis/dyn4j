package org.dyn4j.particle.callbacks;

import org.dyn4j.geometry.Vector2;

public interface ParticleRaycastCallback {
  /**
   * Called for each particle found in the query. See
   * {@link RayCastCallback#reportFixture(org.jbox2d.dynamics.Fixture, Vec2, Vec2, float)} for
   * argument info.
   * 
   * @param index
   * @param point
   * @param normal
   * @param fraction
   * @return
   */
  float reportParticle(int index, Vector2 point, Vector2 normal, float fraction);

}
