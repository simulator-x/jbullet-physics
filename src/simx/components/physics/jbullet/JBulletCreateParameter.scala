/*
 * Copyright 2012 The SIRIS Project
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 *
 * The SIRIS Project is a cooperation between Beuth University, Berlin and the
 * HCI Group at the University of Würzburg. The project is funded by the German
 * Federal Ministry of Education and Research (grant no. 17N4409).
 */

package simx.components.physics.jbullet

import simx.core.components.physics.PhysicsAspect
import simx.core.ontology.Symbols
import simx.core.entity.description.SValSeq
import simplex3d.math.floatx.ConstVec3f

//Global Types
import simx.core.ontology.{types => gt}

/*
 * User: martin
 * Date: 6/8/11
 * Time: 11:34 AM
 */

/**
 *  Used to configure JBullet.
 *
 * @param simulationSpeed A multiplier to deltaT. If this value is greater than 1.0f the simulation runs faster and vice versa.
 * @param gravity         The global gravity of the simulation.
 */
case class JBulletCreateParameter(simulationSpeed: Either[Float, Null] = null, gravity: ConstVec3f = null,
                                override val targets : List[Symbol] = Nil) extends PhysicsAspect(Symbols.configuration) {

  override def getFeatures =
    Set( gt.SimulationSpeed, gt.Gravity )


  def getProvidings =
    getFeatures

  def getCreateParams = addCVars {
    val result = new SValSeq
    if(simulationSpeed.isLeft) result and gt.SimulationSpeed(simulationSpeed.left.get)
    if(gravity != null) result and gt.Gravity(gravity)
    result
  }
}