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

package simx.components.physics.jbullet.types

import simx.core.ontology.{SVarDescription, Symbols}


object Acceleration extends SVarDescription( simx.components.physics.jbullet.types.Vector3 as Symbols.acceleration )

object Gravity extends SVarDescription( simx.components.physics.jbullet.types.Vector3 as Symbols.gravity )

object HalfExtends extends SVarDescription( simx.components.physics.jbullet.types.Vector3 as Symbols.halfExtends )

object Impulse extends SVarDescription( simx.components.physics.jbullet.types.Vector3 as Symbols.impulse )

object Normal extends SVarDescription( simx.components.physics.jbullet.types.Vector3 as Symbols.normal )

object Transformation extends SVarDescription[com.bulletphysics.linearmath.Transform, simplex3d.math.floatx.ConstMat4f]( simx.core.ontology.types.Matrix as Symbols.transformation createdBy new com.bulletphysics.linearmath.Transform(new javax.vecmath.Matrix4f(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1)))

object Vector3 extends SVarDescription[javax.vecmath.Vector3f, simplex3d.math.floatx.ConstVec3f]( simx.core.ontology.types.Vector3 createdBy new javax.vecmath.Vector3f)
object Velocity extends SVarDescription[javax.vecmath.Vector3f, simplex3d.math.floatx.ConstVec3f]( simx.core.ontology.types.Velocity createdBy new javax.vecmath.Vector3f)