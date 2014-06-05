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

import simplex3d.math.floatx.ConstVec3f
import javax.vecmath.{Matrix4f, Vector3f}
import simx.core.entity.typeconversion.Converter

//Global Types & Local Types
import simx.core.ontology.{types => gt}
import simx.components.physics.jbullet.{types => lt}

/*
* Created by IntelliJ IDEA.
* User: martin
* Date: 6/7/11
* Time: 10:32 AM
*/
object JBulletConverters {
  //Todo:
  //This encounters a jbullet bug:
  //When absolute sizes and positions get too small < 0.1f, jbullet does not compute correct values any more
  //This scale factor i used to magnify all values passed to jbullet
  //But this has to be solved in a nicer way
  //
  //!!! Be careful when changing scale to a value other than 1f. !!!
  //!!! This hotfix does currently not support to scale shapes from collada files !!!
  //It is likely that I (martin) forgot to check all locations where scale has to be multiplied
  val scale = 1f

  val vectorConverter = new Converter(lt.Vector3, lt.Gravity, lt.Acceleration, lt.Velocity, lt.HalfExtends, lt.Normal, lt.Impulse)(gt.Vector3) {
    def revert(from: ConstVec3f): Vector3f =
      new Vector3f(from.x*scale, from.y*scale, from.z*scale)

    def convert(from: Vector3f): ConstVec3f =
      ConstVec3f(from.x, from.y, from.z) * (1f/scale)
  }

  val transformConverter = new Converter(lt.Transformation)(gt.Transformation) {
    def revert(from: simplex3d.math.floatx.ConstMat4f): com.bulletphysics.linearmath.Transform = {
      new com.bulletphysics.linearmath.Transform(
        new Matrix4f(
          from.m00, from.m10, from.m20, from.m30 * scale,
          from.m01, from.m11, from.m21, from.m31 * scale,
          from.m02, from.m12, from.m22, from.m32 * scale,
          from.m03, from.m13, from.m23, from.m33)
      )
    }

    def convert(from: com.bulletphysics.linearmath.Transform): simplex3d.math.floatx.ConstMat4f = {
      val matrix = from.getMatrix(new Matrix4f)
      simplex3d.math.float.ConstMat4(
        matrix.m00, matrix.m10, matrix.m20, matrix.m30,
        matrix.m01, matrix.m11, matrix.m21, matrix.m31,
        matrix.m02, matrix.m12, matrix.m22, matrix.m32,
        matrix.m03 * (1f/scale), matrix.m13 * (1f/scale), matrix.m23 * (1f/scale), matrix.m33)
    }
  }

  def register() {}
}