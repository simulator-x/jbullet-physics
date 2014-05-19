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

/*
 * Created by IntelliJ IDEA.
 * User: martin
 * Date: 5/11/11
 * Time: 11:39 AM
 */
package simx.components.physics.jbullet

import com.bulletphysics.dynamics.DiscreteDynamicsWorld
import com.bulletphysics.collision.broadphase.{BroadphaseInterface, Dispatcher}
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver
import com.bulletphysics.collision.dispatch.CollisionConfiguration
import javax.vecmath.Vector3f

class JBulletDiscreteDynamicsWorld(dispatcher: Dispatcher, pairCache: BroadphaseInterface, constraintSolver: ConstraintSolver, collisionConfiguration: CollisionConfiguration)
  extends DiscreteDynamicsWorld(dispatcher, pairCache, constraintSolver, collisionConfiguration){

  def getGravity : Vector3f = {
    val g = new Vector3f
    getGravity(g)
    g
  }
}


