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

import xml.Node
import simplex3d.math.floatx._
import javax.vecmath.Vector3f
import com.bulletphysics.dynamics.RigidBodyConstructionInfo
import com.bulletphysics.linearmath.{Transform, DefaultMotionState}
import simx.core.entity.description.NamedSValSet
import collection.mutable
import com.bulletphysics.collision.shapes.{CollisionShape, ConvexHullShape}
import simx.core.components.physics.PhysicsException

//Global Types
import simx.core.ontology.{types => gt}
//Helps shortening the javax and jbullet.linearmath code
import simx.components.physics.jbullet.{JBulletMath => m}


/**
 * User: martin
 * Date: Nov 29, 2010
 */

/**
 * Loads RigidBodys from collada files and manages loaded data.
 */
object JBulletColladaLoader {

  /**
   *  Identifies a RigidBody stored in a collada file.
   */
  private case class RigidBodyUrl(fileName: String, objectId: String)

  /**
   *  Stores all properties needed to create a RigidBody
   */
  private case class RigidBodyInfo(shape: CollisionShape, mass: Float, centerOfMassOffset: Transform){
    var restitution: Option[Float] = None
    var staticFriction: Option[Float] = None
    var linearVelocity: Option[Vector3f] = None
    var angularVelocity: Option[Vector3f] = None

    /**
     *  Creates a RigidBody from this RigidBodyInfo
     */
    def createRigidBody: JBulletRigidBody = {

      val inertia =
        if(mass != 0f) {
          val tempInertia = new Vector3f
          shape.calculateLocalInertia(mass, tempInertia)
          tempInertia
        }
        else m.ZeroVec

      val rb = new JBulletRigidBody(new RigidBodyConstructionInfo(
        mass,
        new DefaultMotionState(m.Idendity, centerOfMassOffset),
        shape,
        inertia))

      restitution.collect{case r => rb.setRestitution(r)}
      staticFriction.collect{case f => rb.setFriction(f)}
      linearVelocity.collect{case lv => rb.setLinearVelocity(lv)}
      angularVelocity.collect{case av => rb.setAngularVelocity(av)}

      rb
    }
  }

  /**
   *   Stores all loaded RigidBodies.
   */
  private val loaded = new mutable.HashMap[RigidBodyUrl, RigidBodyInfo]()

  /**
   *    Returns the RigidBody described in the SValSet.
   * Loades RigidBodies only once from disc.
   */
  def get(tcps: NamedSValSet): JBulletRigidBody = {

    val url = RigidBodyUrl(tcps.firstValueFor(gt.ColladaFile), tcps.getFirstValueForOrElse(gt.ColladaObjectId)(""))
    if(!loaded.contains(url)) loaded += (url -> load(url))
    loaded(url).createRigidBody
  }

  /**
   *  Loads a RigidBodyInfo from a collada file.
   */
  private def load(url: RigidBodyUrl): RigidBodyInfo = {

    val fileName = url.fileName
    val objectId = url.objectId

    //Load the collada file
    val dae = xml.XML.loadFile(fileName)

    //Returns to value of an node option or throws an exception
    def get(o: Option[Node]): Node = {
       o match {
         case Some(node) => node
         case None => throw PhysicsException("Error while reading from collada file '" + fileName + "'.")
       }
    }

    //Returns a specific library_geometries child
    def getGeometryNodeById(id: String) = {
      get(((dae \ "library_geometries") \ "geometry").find( (n: Node) => (n \ "@id").toString() == id))
    }

    def getMaterialNodeById(id: String) = {
      get(((dae \ "library_physics_materials") \ "physics_material").find( (n: Node) => (n \ "@id").toString() == id))
    }

    //Extract the RigidBody entries from library_physics_scenes
    val rigidBodyNodes = (dae \ "library_physics_scenes") \\ "instance_rigid_body"

    val instanceRbNode =
    //Search the actual rb
    if(objectId != ""){
      rigidBodyNodes.find( (n: Node) => (n \ "@target").toString() == ("#" + objectId) ) match {
        case Some(node) => node
        case None => throw PhysicsException("ObjectId '" + objectId + "' not found in collada file '" + fileName + "'.")
      }
    }
    //Take the first rb
    else {
      rigidBodyNodes.headOption match {
        case Some(node) => node
        case None => throw PhysicsException("No rigidbody found in collada file '" + fileName + "'.")
      }
    }


    val sid = (instanceRbNode \ "@body").toString()

    //Get the rb node from library_physics_models
    val rbNode = get(((dae \ "library_physics_models") \\ "rigid_body").find( (n: Node) => (n \ "@sid").toString() == sid))


    val shapeNode = get((rbNode \\ "shape").find( (n: Node) => n.child.exists(_.label == "instance_geometry") ))

    val instanceGeometryURL = ((shapeNode \ "instance_geometry") \ "@url").toString().replaceAll("#", "")

    val geometryNode = getGeometryNodeById(instanceGeometryURL)
    val convexMeshNode = get((geometryNode \ "convex_mesh").headOption)

    val sourceNode = convexMeshNode.attribute("convex_hull_of") match {
      case Some(id) => get((getGeometryNodeById(id.toString().replaceAll("#", "")) \ "mesh" \ "source").headOption)
      case None => get((convexMeshNode \ "source").headOption)
    }

    //Minimal test if data is what i think it is
    val accessorNode = get((sourceNode \ "technique_common" \ "accessor").headOption)
    if( (!accessorNode.attribute("stride").isDefined) || (accessorNode.attribute("stride").get.toString().toInt != 3))
      throw PhysicsException("Unsupported physical geomety in collada file '" + fileName + "'.")
    //Minimal test if data is what i think it is END

    val floatArrayNode = get((sourceNode \ "float_array").headOption)

    val rawData = for(f <- floatArrayNode.child.head.toString().split(" ")) yield f.toFloat/10.0f
    val floatArrayData = myZip(rawData.toList)

    //Transform shape
    val dataTransform = ConstMat4f((shapeNode \ "rotate").headOption match {
      case Some(node) => (for(f <- node.child.head.toString().split(" ")) yield f.toFloat).toList match {
        case x :: y :: z :: angle :: Nil => functions.rotationMat(functions.radians(angle), ConstVec3f( x, y, z))
        case _ => Mat3x4f.Identity
      }
      case None => Mat3x4f.Identity
    })

    val constDataTransform = (shapeNode \ "translate").headOption match {
      case Some(node) => (for(f <- node.child.head.toString().split(" ")) yield {f.toFloat/10.0f}).toList match {
        case x :: y :: z :: Nil =>  ConstMat4f(Mat4x3f.translate(Vec3f(x,y,z))) * ConstMat4f(dataTransform)
        case _ => ConstMat4f(dataTransform)
      }
      case None => ConstMat4f(dataTransform)
    }

    val mass = get((rbNode \\ "mass").headOption).child.head.toString().toFloat

    val transformedFloatArrayData =
    if(mass != 0) {
      for(point <- floatArrayData) yield {
        val s3dVec4 =  /*ConstMat4f(dataTransform)  * */ ConstVec4f(point)
        new Vector3f(s3dVec4.x, s3dVec4.y, s3dVec4.z)
      }
    }
    else {
      for(point <- floatArrayData) yield {
        val s3dVec4 =  constDataTransform  * ConstVec4f(point)
        new Vector3f(s3dVec4.x, s3dVec4.y, s3dVec4.z)
      }
    }
    //Transform shape END

    //Create RigidBody
    val dataContainer = new com.bulletphysics.util.ObjectArrayList[javax.vecmath.Vector3f]()
    transformedFloatArrayData.foreach((value) => dataContainer.add(value))
    val shape = new ConvexHullShape(dataContainer)

    var centerOfMassOffset = JBulletConverters.transformConverter revert Mat4x4f(functions.inverse(constDataTransform))

    if(mass == 0) centerOfMassOffset = m.Idendity


//    var inertia = (rbNode \\ "inertia").headOption match {
//      case Some(node) => (for(f <- node.child.head.toString().split(" ")) yield f.toFloat/10.0f).toList match {
//        case x :: y :: z :: Nil => val tempVec = new Vector3f(); shape.calculateLocalInertia(mass, tempVec); tempVec//new Vector3f(x, y, z)
//        case _ => val tempVec = new Vector3f(); shape.calculateLocalInertia(mass, tempVec); tempVec
//      }
//      case None => val tempVec = new Vector3f(); shape.calculateLocalInertia(mass, tempVec); tempVec
//    }

    val rbi = RigidBodyInfo(shape, mass, centerOfMassOffset)
//    val rb = if(mass != 0) new JBulletRigidBody(
//              new RigidBodyConstructionInfo(
//                  mass,
//                  new DefaultMotionState(new Transform(IdendityMat), centerOfMassOffset),
//                  shape,
//                  inertia )) else
//    new JBulletRigidBody(
//              new RigidBodyConstructionInfo(
//                  mass,
//                  new DefaultMotionState(new Transform(IdendityMat), centerOfMassOffset),
//                  shape,
//                  inertia ))

    //Set instance material properties if present
    (rbNode \\ "instance_physics_material").headOption match {
      case Some(node) => {
        val instancePhysicsMaterialUrl = (node \ "@url").toString().replaceAll("#", "")
        val physicsMaterialNode = getMaterialNodeById(instancePhysicsMaterialUrl)
        //Set restitution and friction if present
        (physicsMaterialNode \\ "restitution").headOption.collect{case n => rbi.restitution = Some(n.child.head.toString().toFloat)}
        (physicsMaterialNode \\ "static_friction").headOption.collect{case n => rbi.staticFriction = Some(n.child.head.toString().toFloat)}
      }
      case None => 
    }

    //Set velocity and angular velocity if present  
    (instanceRbNode \\ "velocity").headOption.collect{
      case node => (for(f <- node.child.head.toString().split(" ")) yield f.toFloat).toList match {
        case x :: y :: z :: Nil => rbi.linearVelocity = Some(new Vector3f(x, y, z))
        case _ =>
      }
    }

    (instanceRbNode \\ "angular_velocity").headOption.collect{
      case node => (for(f <- node.child.head.toString().split(" ")) yield f.toFloat).toList match {
        case x :: y :: z :: Nil => rbi.angularVelocity= Some(new Vector3f(x, y, z))
        case _ =>
      }
    }

    rbi
  }


  def myZip(l: List[Float]): List[Vec4f] = l match {
    case a :: b :: c :: tail => Vec4f(a, b, c, 1f) :: myZip(tail)
    case _ => Nil
  }

}