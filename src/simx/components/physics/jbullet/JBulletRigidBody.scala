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
 * Date: 5/5/11
 * Time: 11:39 AM
 */
package simx.components.physics.jbullet

import simx.core.ontology.Symbols
import com.bulletphysics.dynamics.{RigidBodyConstructionInfo, RigidBody}
import com.bulletphysics.linearmath.{DefaultMotionState, Transform}
import com.bulletphysics.collision.shapes._
import simx.core.entity.description.SValSet
import com.bulletphysics.collision.dispatch.CollisionObject
import javax.vecmath.Vector3f
import simx.core.components.physics.Level
import simplex3d.math.float._
import simplex3d.math.float.functions._
import simx.core.entity.description.NamedSValSet
import simx.core.components.physics.PhysicsException

//Helps shortening the javax and jbullet.linearmath code
import simx.components.physics.jbullet.{JBulletMath => m}
//Global Types
import simx.core.ontology.{types => gt}
//Local Types
import simx.components.physics.jbullet.ontology.{types => lt}

/**
 *    Object for RigidBody creation from TypedCreateParamSets
 */
object JBulletRigidBody {

  /**
   *    Creates a RigidBody from a NamedSValList
   * @return  The created RigidBody or None if critical error occurred
   */
  def apply(tcps: NamedSValSet): Option[JBulletRigidBody] =
    if(tcps.semantics == Symbols.colladaFile) fromCollada(tcps) else fromShape(tcps)

  /**
   *    Checks if a CollisionObject is a JBulletRigidBody and returns it if so.
   * @return  The casted JBulletRigidBody or null
   */
  def upcast(colObj: CollisionObject): JBulletRigidBody = RigidBody.upcast(colObj) match {
    case rb : JBulletRigidBody => rb
    case _ => null
  }

  /**
   *    Constructs a rigid body from a NamedSValList, assuming that its semantics
   *          implicates a creation from a collada file.
   * Returns None if the NamedSValList was malformed
   */
  private def fromCollada(tcps: NamedSValSet): Option[JBulletRigidBody] = {
    //Come get some!
    try Some(JBulletColladaLoader.get(tcps).setProperties(tcps))
    catch {
      case e : Exception =>
        println(PhysicsException("Error during rigid body construction form file.", Level.warn))
        println(e)
        None
    }
  }

  /**
   *    Constructs a rigid body from a NamedSValList, assuming that its semantics
   *          implicates a creation from a shape.
   * Returns None if the NamedSValList was malformed
   */
  private def fromShape(tcps: NamedSValSet): Option[JBulletRigidBody] = {

    //Buy some!
    try Some(new JBulletRigidBody(createShape(tcps)).setPropertiesWithDefaults(tcps))
    catch {
      case e : Exception =>
        println(PhysicsException("Error during rigid body construction form shape.", Level.warn))
        println(e)
        None
    }
  }

  /**
   *  Creates a CollisionShape from a NamedSValList
   */
  private def createShape(tcps: NamedSValSet): CollisionShape = {

    tcps.semantics match {
      case Symbols.sphere => new SphereShape(tcps.firstValueFor(gt.Radius)* JBulletConverters.scale)
      case Symbols.capsule => createCapsuleShape(tcps)
      case Symbols.cylinder => createCylinderShape(tcps)
      case Symbols.plane =>
        val normal: Vector3f = tcps.firstValueFor(lt.Normal)
        normal.normalize()
        new StaticPlaneShape(normal, (tcps.firstValueFor(gt.Thickness) / 2f) * JBulletConverters.scale)
      case Symbols.box => new BoxShape(tcps.firstValueFor(lt.HalfExtends))
      case shape => throw new Exception("Unknown collision shape '" + shape + "'")
    }
  }

  /**
   *  Creates a capsule shape from a NamedSValList
   */
  private def createCapsuleShape(tcps: NamedSValSet) = tcps.firstValueFor(gt.Alignment) match {
    case simx.core.components.physics.Axis.X =>
      new CapsuleShapeX(
        tcps.firstValueFor(gt.Radius) * JBulletConverters.scale,
        tcps.firstValueFor(gt.Height) * JBulletConverters.scale)
    case simx.core.components.physics.Axis.Y =>
      new CapsuleShape(
        tcps.firstValueFor(gt.Radius) * JBulletConverters.scale,
        tcps.firstValueFor(gt.Height) * JBulletConverters.scale)
    case simx.core.components.physics.Axis.Z =>
      new CapsuleShapeZ(
        tcps.firstValueFor(gt.Radius) * JBulletConverters.scale,
        tcps.firstValueFor(gt.Height) * JBulletConverters.scale)
    case _ => throw PhysicsException("Unknown enumeration value for Alignment. Please use simx.components.physics.Axis!")
  }

  /**
   *  Creates a cylinder shape from a NamedSValList
   */
  private def createCylinderShape(tcps: NamedSValSet) = {

    val halfExtends = m.mul(new Vector3f(1,1,1), tcps.firstValueFor(gt.Radius) * JBulletConverters.scale)
    tcps.firstValueFor(gt.Alignment) match {
      case simx.core.components.physics.Axis.X =>
        halfExtends.x = tcps.firstValueFor(gt.Height) * 0.5f * JBulletConverters.scale
        new CylinderShapeX(halfExtends)
      case simx.core.components.physics.Axis.Y =>
        halfExtends.y = tcps.firstValueFor(gt.Height) * 0.5f * JBulletConverters.scale
        new CylinderShape(halfExtends)
      case simx.core.components.physics.Axis.Z =>
        halfExtends.z = tcps.firstValueFor(gt.Height) * 0.5f * JBulletConverters.scale
        new CylinderShapeZ(halfExtends)
      case _ => throw PhysicsException("Unknown enumeration value for Alignment. Please use simx.components.physics.Axis!")
    }
  }

  /**
   *    Adds defaults described in simx.components.physics.PhysBaseProps
   *          to a NamedSValList.
   */
  private[jbullet] def addDefaults(tcps: SValSet) = {
    tcps.addIfNew(gt.Mass(1f))
    tcps.addIfNew(gt.Restitution(0f))
    tcps.addIfNew(gt.LinearDamping(0f))
    tcps.addIfNew(gt.AngularDamping(0f))
    tcps.addIfNew(gt.AngularFactor(1f))
    tcps
  }

  /**
   *  Returns property values to at least initialize SVars
   *        if an error occurs during a entity creation.
   */
  private[jbullet] def getFallbackProperties: SValSet =
    addDefaults(
      new SValSet(
        lt.Transformation(m.Idendity),
        lt.Gravity(m.ZeroVec),
        lt.Velocity(m.ZeroVec),
        lt.Acceleration(m.ZeroVec)
      ))

}

/**
 *
 *  Enhances jbullets RigidBody with some SIRIS related methods
 * @see com.bulletphysics.dynamics.RigidBody
 */
class JBulletRigidBody(constructionInfo: RigidBodyConstructionInfo) extends RigidBody(constructionInfo) {

  /**
   *    Creates a RigidBody from a given shape.
   * Uses mass = 0 and worldTransform = Identity
   */
  def this (shape: CollisionShape) = this( new RigidBodyConstructionInfo(
        0f, new DefaultMotionState(m.Idendity), shape, m.ZeroVec))

  private var linearAcceleration = m.ZeroVec
  private class Orientation(var yaw: Float, var pitch: Float, var roll: Float) {
    def toMatrix = {
      Mat4x3.rotateZ(roll).rotateX(pitch).rotateY(yaw)
      //Mat3x4f(rotationMat(toQuat))
    }

    def toQuat = {
      Quat4.rotateZ(roll).rotateX(pitch).rotateY(yaw)
    }
  }
  private var currentGFOrientation: Option[Quat4] = None
  private var goalGFOrientation: Option[Orientation] = None
  private var upVector: Option[ConstVec3] = None

  private def applyLinearAcceleration() {
    if(linearAcceleration != m.ZeroVec) {
      activate()
      val force = new Vector3f(linearAcceleration)
      force.scale(getMass)
      applyCentralForce(force)
    }
  }

  private def updateGeometricFlightOrientation(deltaT: Float) {
    updateOrientationGoal()
    updateGFOrientation(deltaT)
  }

  private def updateGFOrientation(deltaT: Float) {

    val radPerSec = 6f

    currentGFOrientation match {
      case None =>
        currentGFOrientation = Some(goalGFOrientation.get.toQuat)
      case Some(curr) =>
        val distance = length(Vec4(curr - goalGFOrientation.get.toQuat))
        val maxTurnAngle = radPerSec * deltaT

        //println("d(" + distance + ")")

        if(distance <= maxTurnAngle) currentGFOrientation = Some(goalGFOrientation.get.toQuat)
        //Interpolate
        else {
          val percentage = maxTurnAngle / distance
          //println("Interpolating p(" + percentage + "), d(" + distance + ")")
          val dProd = dot(Vec4(curr).xyz, Vec4(goalGFOrientation.get.toQuat).xyz)
          if(dProd < 0f)
            currentGFOrientation = Some(slerp(-curr, goalGFOrientation.get.toQuat, percentage))
          else
            currentGFOrientation = Some(slerp(curr, goalGFOrientation.get.toQuat, percentage))
//          val postDistance = length(Vec4f((currentGFOrientation.get - goalGFOrientation.get.toQuat)))
//
//          //println("dot(" + dProd + ")")
//          if(postDistance > distance) {
//
//            println("postD(" + postDistance + "), d(" + distance + "), dot(" + dProd + ")")
//          }
        }

    }
  }

  private def updateOrientationGoal() {

    val velInGFCoords = JBulletConverters.vectorConverter.convert(getLinearVelocity)

    if(length(velInGFCoords) < 0.001f) return

    val yawComp = if(velInGFCoords != Vec3.Zero) normalize(velInGFCoords).xz else Vec2.Zero


    if(length(yawComp) < 0.001f) return

    var yaw = math.acos(dot(normalize(normalize(yawComp)), Vec2.UnitY))
    if(yawComp.x < 0f) yaw = 2f * math.Pi - yaw

//    val pitch = math.acos(dot(normalize(velInGFCoords), Vec3.UnitY)) - (math.Pi/2f)

    upVector.collect{case upVec =>
//      val totalAccDir =
//        normalize(
//          -upVec +
//          JBulletConverters.vectorConverter.convert(getLinearAcceleration) +
//          JBulletConverters.vectorConverter.convert(getGravity)
//        )

//      println("TAD " + totalAccDir)
//      println("VEL " + velInGFCoords)

//      val roll =
//        if(length(normalize(velInGFCoords) - normalize(totalAccDir)) < 0.01 ) 0.0
//        else{
//          //Project totalAccDir onto plane defined by the planeNormal velInGFCoords
//          val projectedTotalAccDir =
//            totalAccDir - dot(totalAccDir, normalize(velInGFCoords)) * normalize(velInGFCoords)
//
////          println("ProJAcc " + normalize(projectedTotalAccDir))
//
//          val rotatedDown =
//            (Mat4x4(Mat4x3.rotateX(pitch.toFloat).rotateY(yaw.toFloat)) * Vec4(0,-1,0,0)).xyz
//
////          println("RotUp " + rotatedUp)
//
//          var d: Double = dot(normalize(projectedTotalAccDir), normalize(rotatedDown))
//          d = if(d > 1f) 1f else if(d < -1f) -1f else d
//
//          if(dot(cross(normalize(rotatedDown), normalize(velInGFCoords)), normalize(projectedTotalAccDir)) > 0f)
//            math.acos(d)
//          else
//            -math.acos(d)
//        }

//      println("roll " + degrees(roll.toFloat) )
      goalGFOrientation.get.roll = 0//roll.toFloat



    }


//    println("yaw " + degrees(yaw.toFloat))
//    println("pitch " + degrees(pitch.toFloat))

    goalGFOrientation.get.yaw = yaw.toFloat
    goalGFOrientation.get.pitch = 0//pitch.toFloat



  }

  /**
   * Does additional simulation. Has to be invoked before the simulate method of DynamicsWorld.
   *
   * @param deltaT The simulations deltaT in seconds.
   */
  def simulate(deltaT: Float) {
    applyLinearAcceleration()
    if(goalGFOrientation.isDefined) updateGeometricFlightOrientation(deltaT)
  }

  /**
   *   Sets properties of this RigidBody from a NamedSValList
   */
  private[jbullet] def setProperties(cps: SValSet): JBulletRigidBody = {
    //Transform
    cps.getFirstValueFor(lt.Transformation).collect{case t => setGraphicsWorldTransform(m.removeScale(t))}
    //Mass
    cps.getFirstValueFor(gt.Mass).collect{case m => setMass(m)}
    //Gravity
    cps.getFirstValueFor(lt.Gravity).collect{case g => setGravity(g)}
    //Restitution
    cps.getFirstValueFor(gt.Restitution).collect{case r => setRestitution(r)}
    //LinearDamping
    cps.getFirstValueFor(gt.LinearDamping).collect{case ld => setDamping(ld, getAngularDamping)}
    //AngularDamping
    cps.getFirstValueFor(gt.AngularDamping).collect{case ad => setDamping(getLinearDamping, ad)}
    //DisableCollisionRotation
    cps.getFirstValueFor(gt.AngularFactor).collect{case af => setAngularFactor(af)}
    //Geometric Flight
    cps.getFirstValueFor(gt.UpVector).collect{case upVec => initalizeGeometricFlight(upVec)}

    this
  }

  /**
   *      Sets properties of this RigidBody from a NamedSValList
   *
   *   Uses the defaults described in simx.components.physics.PhysBaseProps
   *            for values that are not provided.
   */
  private[jbullet] def setPropertiesWithDefaults(cps: SValSet): JBulletRigidBody = {

    val cpsWithDefaults = new SValSet(cps)
    JBulletRigidBody.addDefaults(cpsWithDefaults)
    setProperties(cpsWithDefaults)
  }

  /**
   *  Returns a SValSet containing the properties of this RigidBody
   */
  private[jbullet] def getProperties: SValSet = {
    val props = new SValSet

    props += lt.Transformation(getGraphicsWorldTransform.getOrElse(getWorldTransform))
    props += gt.Mass(getMass)
    props += lt.Gravity(getGravity)
    props += gt.Restitution(getRestitution)
    props += gt.LinearDamping(getLinearDamping)
    props += gt.AngularDamping(getAngularDamping)
    props += gt.AngularFactor(getAngularFactor)
    props += lt.Velocity(getLinearVelocity)
    props += lt.Acceleration(getLinearAcceleration)

  }

  /**
   *   Sets the graphics world transform if the underlying MotionState supports it.
   *          The graphics world transform can have an offset to the center of mass and thus to this
   *          RigidBodies world transform.
   *
   *  The only implementation of the interface MotionState, that JBullet provides, is DefaultMotionState.
   *          DefaultMotionState supports graphics world transforms that have an offset to the center of mass of the
   *          rigid body.
   */
  def setGraphicsWorldTransform(value: Transform) {
    if(!getMotionState.isInstanceOf[DefaultMotionState]) return
    if(getGraphicsWorldTransform.get != value){
      val out = new Transform
      val invComo = new Transform
      invComo.inverse(getMotionState.asInstanceOf[DefaultMotionState].centerOfMassOffset)
      out.set(value)
      out.mul(invComo)
      setWorldTransform(out)
      getMotionState.setWorldTransform(out)
      activate()
    }
  }

  /**
   *    Returns the graphics world transform if the underlying MotionState supports it.
   *          The graphics world transform can have an offset to the center of mass and thus to this
   *          RigidBodies world transform.
   *
   *  The only implementation of the interface MotionState, that JBullet provides, is DefaultMotionState.
   *          DefaultMotionState supports graphics worldtransformss that have an offset to the center of mass of the
   *          rigid body.
   */
  def getGraphicsWorldTransform : Option[Transform] =
    if(!getMotionState.isInstanceOf[DefaultMotionState]) None
    else {
      val transform = getMotionState.asInstanceOf[DefaultMotionState].graphicsWorldTrans
      currentGFOrientation match {
        case None => Some(transform)
        case Some(ori) =>
          val s3dTransform = JBulletConverters.transformConverter.convert(transform)
          val result = ConstMat4(Mat4x3(rotationMat(ori)).translate(s3dTransform(3).xyz))
          Some(JBulletConverters.transformConverter.revert(result))
      }
    }

  /**
   *  Sets the world transform. This is also the center of mass.
   */
  override def setWorldTransform(worldTransform: Transform) {
    if(getWorldTransform != worldTransform){
      super.setWorldTransform(worldTransform)
      activate()
    }
  }

  /**
   *  Returns the world transform. This is also the center of mass.
   */
  def getWorldTransform : Transform = {
    val out = new Transform
    getWorldTransform(out)
    out
  }

  /**
   * Sets the mass using the CollisionShape to calculate the inertia.
   */
  def setMass(m: Float) {
    if(m == getMass) return
    if(m != 0) {
      val inertia = new Vector3f
      getCollisionShape.calculateLocalInertia(m, inertia)
      setMassProps(m, inertia)
    }
    else setMassProps(0, new Vector3f(0,0,0))
  }

  /**
   *  Returns the mass.
   */
  def getMass: Float =
    if(getInvMass != 0) 1f/getInvMass else 0f

  /**
   *  Returns the gravity.
   */
  def getGravity : Vector3f = {
    val g = new Vector3f
    getGravity(g)
    g.scale(getInvMass)
    g
  }



  def getLinearVelocity : Vector3f = {
    val v = new Vector3f
    getLinearVelocity(v)
    v
  }

  def getLinearAcceleration =
    linearAcceleration

  def setLinearAcceleration(la: Vector3f) {
    linearAcceleration = la
  }

  /**
   * Sets the linear damping
   */
  def setLinearDamping(ld: Float) {
    setDamping(ld, getAngularDamping)
  }

  /**
   * Sets the angular damping
   */
  def setAngularDamping(ad: Float) {
    setDamping(getLinearDamping, ad)
  }

  private def initalizeGeometricFlight(upVec: ConstVec3) {

    //currentGFOrientation = Some(new Orientation(0,0,0))
    goalGFOrientation = Some(new Orientation(0,0,0))
    upVector = Some(upVec)

//    val localY = normalize(Vec3f(upVec))
//    var localX =
//      if(math.abs(localY.x - localY.y) > 0.1f) localY.yxz
//      else if(math.abs(localY.x - localY.z) > 0.1f) localY.zyx
//      else if(math.abs(localY.y - localY.z) > 0.1f) localY.xzy
//      else Vec3f(-localY.x, localY.yz)
//    val localZ = cross(localX, localY)
//    localX = cross(localY, localZ)
//
//    val conversionMat = inverse(Mat3x3f(localX, localY, localZ))
//
//    def converter(in: Vec3f): Vec3f = conversionMat * in
//    toGFLocal = converter

  }

  /**
   * Converts a vector from the world coordinate system to the rigid bodies local geometric flight system,
   * where Y is up and Z is the flight direction.
   */
  //private var toGFLocal: (Vec3f) => Vec3f = (in) => in



}