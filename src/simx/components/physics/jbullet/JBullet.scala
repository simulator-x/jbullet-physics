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

import scala.collection.mutable
import scala.collection.immutable

import simx.core.entity.description._
import simx.core.entity.typeconversion.ConvertibleTrait
import simx.core.helper.SVarUpdateFunctionMap

import simx.core.worldinterface.eventhandling._
import simx.core.ontology.Symbols

import javax.vecmath.Vector3f

import com.bulletphysics.collision.dispatch._
import com.bulletphysics.collision.broadphase._
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver
import com.bulletphysics.dynamics._
import simx.core.entity.component.EntityConfigLayer
import com.bulletphysics.linearmath.Transform
import simx.core.entity.Entity
import simx.core.components.physics._
import simx.core.components.physics.PhysicsConfiguration
import scala.Some
import simx.core.components.physics.PhysicsException
import simx.core.entity.description.NamedSValSet
import simplex3d.math.float.ConstVec3

//Helps shortening the javax and jbullet.linearmath code
import simx.components.physics.jbullet.{JBulletMath => m}
import simx.components.physics.jbullet.ontology.{types => lt}
//Global Types
import simx.core.ontology.{types => gt}
//Local Types

/**
 *
 * @param name            The name to register the component with.
 * @param simulationSpeed A multiplier to deltaT. If this value is greater than 1.0f the simulation runs faster and vice versa.
 * @param gravity         The global gravity of the simulation.
 * @param maxNrOfObjects  The maximum number of simulated objects
 * @param nrOfSubSteps    Number of sub steps maximally made by the engine in one step
 * @param fixedTimeStep   The size of one sub step in seconds (see http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_The_World)
 */
case class JBulletComponentAspect(
                                   name : Symbol,
                                   gravity: ConstVec3,
                                   simulationSpeed: Float = 1.0f,
                                   maxNrOfObjects: Int = 1000,
                                   nrOfSubSteps: Int = 1000,
                                   fixedTimeStep: Float = 1.0f / (60.0f * 8.0f)
                                   ) extends PhysicsComponentAspect[JBulletComponent](name, Seq[Any](maxNrOfObjects, nrOfSubSteps, fixedTimeStep))
{
  def getComponentFeatures: Set[ConvertibleTrait[_]] = Set(gt.Gravity, gt.SimulationSpeed)
  def getCreateParams: NamedSValSet = NamedSValSet(aspectType, gt.Gravity(gravity), gt.SimulationSpeed(simulationSpeed))
}

/**
 * A component implementation for the the JBullet physics engine.
 *
 * @param componentName   The name to register the component with.
 * @param maxNrOfObjects  The maximum number of simulated objects
 * @param nrOfSubSteps    Number of sub steps maximally made by the engine in one step
 * @param fixedTimeStep   The size of one sub step in seconds (see http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_The_World)
 */
class JBulletComponent( override val componentName : Symbol = JBullet.getComponentName, maxNrOfObjects: Int = 1000,
                        nrOfSubSteps: Int = 1000, fixedTimeStep: Float = 1.0f / (60.0f * 8.0f))
  extends PhysicsComponent(componentName) with SVarUpdateFunctionMap with EntityConfigLayer {

  def this(componentName: Symbol) = this( componentName, 1000, 1000, 1.0f / (60.0f * 8.0f) )
  def this() = this( JBullet.getComponentName, 1000, 1000, 1.0f / (60.0f * 8.0f) )


  protected def requestInitialConfigValues(toProvide: Set[ConvertibleTrait[_]], aspect: EntityAspect, e: Entity) = {
    configure(aspect.getCreateParams)
    aspect.getCreateParams.combineWithValues(toProvide)._1
  }

  protected def finalizeConfiguration(e: Entity){
    e.observe(lt.Gravity).first( discreteDynWorld.setGravity _ )
    e.observe(gt.SimulationSpeed).first( value => simulationSpeed = value )
  }

  //
  //  Init
  //
  /**
   * Internal JBullet objects
   */
  private val collisionConfiguration = new DefaultCollisionConfiguration()
  private val dispatcher = new CollisionDispatcher(collisionConfiguration)
  private val solver = new SequentialImpulseConstraintSolver()
  private val broadphase = new SimpleBroadphase(maxNrOfObjects)

  /**
   * The simulation object.
   */
  private val discreteDynWorld = new JBulletDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration)
  discreteDynWorld.setInternalTickCallback(new InternalCallBackClass, new Object())

  /**
   * Stores the internal represenation (JBulletRigidBody) of entities.
   */
  private var entityMap = mutable.Map[Entity, JBulletRigidBody]()
  /**
   * methods to be executed when the rigid body is registered
   */
  private val futureFuncMap = mutable.Map[Entity, List[JBulletRigidBody => Any]]()

  /**
   * Stores the external represenation (Entity) of JBulletRigidBodies.
   */
  private var rigidBodyMap = mutable.Map[JBulletRigidBody, Entity]()

  /**
   * Stores detached entities
   */
  private var detachedEntities = mutable.Set[Entity]()

  /**
   * Stores held JBulletRigidBodies and thier original mass.
   */
  private val heldRigidBodies = mutable.Map[JBulletRigidBody, Float]()

  /**
   * Stores the simulation speed multipier
   */
  private var simulationSpeed: Float = 1.0f

  /**
   * Use SVarUpdateFunctionMap to ignore own writes when observing svars
   */
  ignoredWriters = immutable.Set(self)

  //register converters
  JBulletConverters.register()

  //
  //  Register events
  //
  //provideEvent( PhysicsEvents.collision )
  requestEvent( PhysicsEvents.impulse )
  requestEvent( PhysicsEvents.torqueImpulse )
  //
  //  Register events End
  //

  //
  //  Handlers
  //



  /**
   *  Makes a JBulletRigidBody static.
   * @see   JBHold
   */
  addHandler[JBHold]{ msg =>
    holdEntity(msg.entity)
  }

  /**
   *  Releases a held JBulletRigidBody.
   * @see   JBHold, JBRelease
   */
  addHandler[JBRelease]{ msg =>
   releaseEntity(msg.entity)
  }
  //
  //   Handlers End
  //

  private def holdEntity(e : Entity, success : Boolean => Any = b => {}){
    execOnRigidBody(e) { rb =>
      if (!heldRigidBodies.contains(rb)) {
        heldRigidBodies += rb -> rb.getInvMass
        rb.setMassProps(0, new Vector3f(0, 0, 0))
        success(true)
      } else
        success(false)
    }
  }

  private def releaseEntity(e : Entity, success : Boolean => Any = b => {}){
    execOnRigidBody(e) { rb =>
      success apply heldRigidBodies.get(rb).collect {
        case inverseMass =>
          if (inverseMass == 0f) {
            rb.setMassProps(0, new Vector3f(0, 0, 0))
          }
          else {
            val mass = 1f / inverseMass
            val tempVec = new Vector3f()
            rb.getCollisionShape.calculateLocalInertia(mass, tempVec)
            rb.setMassProps(mass, tempVec)
            rb.activate(true)
          }
          heldRigidBodies.remove(rb)
          true
      }.getOrElse(false)
    }
  }

  override protected def disableAspect(asp: EntityAspect, e: Entity, success : Boolean => Any){
    if (entityMap.contains(e))
      holdEntity(e, success)
    else
      success(false)
  }

  override protected def enableAspect(asp: EntityAspect, e: Entity, success : Boolean => Any){
    if (entityMap.contains(e))
      releaseEntity(e, success)
    else
      success(false)
  }

  protected def performSimulationStep() {
    val deltaT = (System.currentTimeMillis() - this.beginOfLastSimulationStep) / 1000f
    rigidBodyMap.keys.foreach(_.simulate(deltaT))
    discreteDynWorld.stepSimulation(deltaT * simulationSpeed, nrOfSubSteps, fixedTimeStep)
    handleCollisions()
    updateAllSVars()
    this.simulationCompleted()
  }

  /**
   *    Starting up the component.
   * Executed after Actor.start and before the first message handling.
   */
  override def startUp() {
    //this ! PerformSimulationStep()
  }

  /**
   *  Configures JBullet
   */
  addHandler[PhysicsConfiguration] {
    msg =>
      val params = msg.toConfigurationParams
      configure( params )
  }

  protected def configure(params: SValSet) {
    params.getFirstValueFor(gt.SimulationSpeed).collect{ case s => simulationSpeed = s}
    params.getFirstValueFor(lt.Gravity).collect{ case g => discreteDynWorld.setGravity(g) }
  }

  /**
   *  Handles a SetTransformation message.
   *
   * @see PhysicsComponent, PhysicsMessages
   */
  def handleSetTransformation(e: Entity, t: SVal[gt.Transformation.dataType]) {
    execOnRigidBody(e){ _.setGraphicsWorldTransform(t.as(lt.Transformation)) }
  }

  /**
   * Handles a ApplyImpulse message.
   *
   * @see PhysicsComponent, PhysicsMessages
   */
  def handleApplyImpulse(e: Entity, i: SVal[gt.Impulse.dataType]) {
    execOnRigidBody(e){ rb =>
      if (!rb.isActive) rb.activate()
      rb.applyCentralImpulse(i.as(lt.Impulse))
    }
  }

  /**
   * Handles a ApplyTorqueImpulse message.
   *
   * @see PhysicsComponent, PhysicsMessages
   */
  def handleApplyTorqueImpulse(e: Entity, i: SVal[gt.Impulse.dataType]) {
    execOnRigidBody(e){ rb =>
      if (!rb.isActive) rb.activate()
      rb.applyTorqueImpulse(i.as(lt.Impulse))
    }
  }

  /**
   * Handles a SetLinearVelocity message.
   *
   * @see PhysicsComponent, PhysicsMessages
   */
  def handleSetLinearVelocity(e: Entity, v: SVal[gt.Velocity.dataType]) {
    execOnRigidBody(e){ _.setLinearVelocity(v.as(lt.Velocity)) }
  }

  /**
   * Handles a SetAngularVelocity message.
   *
   * @see PhysicsComponent, PhysicsMessages
   */
  def handleSetAngularVelocity(e: Entity, v: SVal[gt.Velocity.dataType]) {
    execOnRigidBody(e){ _.setAngularVelocity(v.as(lt.Velocity)) }
  }

  /**
   * Handles a SetGravity message.
   *
   * @see PhysicsComponent, PhysicsMessages
   */
  def handleSetGravity(e: Entity, g: SVal[gt.Gravity.dataType]) {
    execOnRigidBody(e){ rb =>
      rb.setGravity(g.as(lt.Gravity)) }
  }

  /**
   * Handles a DetachEntity message.
   *
   * @see PhysicsComponent, PhysicsMessages
   */
  def handleDetachEntity(e: Entity) {
    if (!detachedEntities.contains(e))
      execOnRigidBody(e){
        rb =>
          //Disable simulation
          discreteDynWorld.removeRigidBody(rb)
          //Disable updates
          e.getAllStateParticles.foreach( x => pauseUpdatesFor(x._3) )
          //Store entity
          detachedEntities.add(e)
      }
  }

  /**
   * Handles a AttachEntity message.
   *
   * @see PhysicsComponent, PhysicsMessages
   */
  def handleAttachEntity(e: Entity) {
    if (detachedEntities.contains(e))
      execOnRigidBody(e){
        rb =>
          //Set position
          e.getSVars(lt.Transformation).collect {
            case (_, t) => get( t)( (value:Transform) => {
              //Enable simulation
              discreteDynWorld.addRigidBody(rb)
              //Magic
              rb.setCollisionFlags(0)
              //Update position
              rb.setGraphicsWorldTransform(value)
              //Reset velocity
              rb.clearForces()
              rb.setLinearVelocity(m.ZeroVec)
              rb.setAngularVelocity(m.ZeroVec)
              //Enable updates
              e.getAllStateParticles.foreach( x => resumeUpdatesFor(x._3) )
              //Update internal data structure
              detachedEntities.remove(e)
            })
          }
      }
  }

  private val observedCollisions = mutable.Set[(CollisionObject, CollisionObject)]()

  /**
   *  Used to react on collisions.
   */
  private class InternalCallBackClass extends InternalTickCallback {

    def internalTick(world: DynamicsWorld, timeStep: Float) {
      val numManifolds = world.getDispatcher.getNumManifolds
      for (i <- 0 until numManifolds) {
        val contactManifold = world.getDispatcher.getManifoldByIndexInternal(i)
        val numContacts = contactManifold.getNumContacts
        for (j <- 0 until numContacts) {
          val pt = contactManifold.getContactPoint(j)
          if (pt.getDistance < 0.0f) {
            val rbA = contactManifold.getBody0.asInstanceOf[CollisionObject]
            val rbB = contactManifold.getBody1.asInstanceOf[CollisionObject]
            if(rbA.isActive || rbB.isActive)
              observedCollisions.add(rbA -> rbB)
          }
        }
      }
    }
  }

  /**
   *  The reaction on all collisions.
   */
  private def handleCollisions(){
    observedCollisions.foreach{ collisionTuple =>
      rigidBodyMap.get(JBulletRigidBody.upcast(collisionTuple._1)).collect{
        case entity1 => rigidBodyMap.get(JBulletRigidBody.upcast(collisionTuple._2)).collect{
          case entity2 => PhysicsEvents.collision.emit(Set(entity1, entity2))
        }
      }
    }
    observedCollisions.clear()
  }

  /**
   *  Removes an entity from this JBullet component.
   */
  def removeFromLocalRep(e: Entity) {

    e.getAllStateParticles.foreach{ x =>
      //Stop observing
      x._3.ignore()
      //Remove update function
      removeSVarUpdateFunctions( x._3 )
    }

    //Remove from local maps and simulation
    execOnRigidBody(e){
      case rb: JBulletRigidBody =>
        rigidBodyMap -= rb
        entityMap -= e
        detachedEntities -= e
        discreteDynWorld.removeRigidBody(rb)
    }
  }

  /**
   *  Returns the additional providings to a given aspect.
   */
  override def getAdditionalProvidings( aspect : EntityAspect ) : Set[ConvertibleTrait[_]] = {
    var providings = Set[ConvertibleTrait[_]]()
    val tcps = aspect.getCreateParams

    tcps.semantics match {
      case Symbols.configuration =>
        providings = providings + gt.SimulationSpeed
        providings = providings + gt.Gravity
      //RigidBody
      case _ =>
        //PhysBaseProps
        if(tcps.contains(gt.Transformation)) providings = providings + gt.Transformation
        providings = providings + gt.Mass
        providings = providings + gt.Gravity
        providings = providings + gt.Restitution
        providings = providings + gt.LinearDamping
        providings = providings + gt.AngularDamping
        providings = providings + gt.AngularFactor
        //Additional
        providings = providings + gt.Velocity
    }

    providings
  }

  def requestInitialValues( toProvide : Set[ConvertibleTrait[_]], aspect : EntityAspect, e : Entity,
                            given : SValSet ) {
    val tcps = new NamedSValSet(aspect.getCreateParams)

    val initialValues = tcps.semantics match {
      case Symbols.component     => tcps
      case Symbols.configuration =>
        tcps.addIfNew(gt.SimulationSpeed(simulationSpeed))
        tcps.addIfNew(lt.Gravity(discreteDynWorld.getGravity))
        tcps
      //RigidBody
      case _ =>
        val rbValues = JBulletRigidBody(tcps) match {
          case Some(rb) =>
            val tempProps = rb.getProperties - lt.Gravity
            tcps.getFirstValueFor(lt.Gravity) match {
              case Some(g) => tempProps.addIfNew(lt.Gravity(g))
              case None    => tempProps.addIfNew(lt.Gravity(discreteDynWorld.getGravity))
            }
            tempProps
          case None => JBulletRigidBody.getFallbackProperties }
        rbValues.addIfNew(lt.Velocity(m.ZeroVec))
        rbValues
    }

    //if(!initialValues.satisfies(toProvide)) throw PhysicsException("[jbullet][error] Debug")
    provideInitialValues(e, initialValues.combineWithValues(toProvide)._1)
  }

  /**
   *  Registers a JBulletRigidBody at the JBullet component.
   */
  private def registerRigidBody(rb: JBulletRigidBody, e: Entity) = {
    val g = rb.getGravity
    discreteDynWorld.addRigidBody(rb)
    rb.setGravity(g)
    entityMap += e -> rb
    rigidBodyMap += rb -> e
    futureFuncMap.remove(e).collect{ case list => list.reverse.foreach{ _(rb) } }
  }

  protected def execOnRigidBody(e : Entity)(func : JBulletRigidBody => Any){
    entityMap.get(e) match {
      case None     => futureFuncMap.put(e, func :: futureFuncMap.getOrElse(e, List[func.type]()))
      case Some(rb) => func(rb)
    }
  }

  /**
   *  Connects a JBulletRigidBody and its corresponding SVars using the SVarUpdateFunctions trait.
   */
  private def connectSVars(rb: JBulletRigidBody, e: Entity) {
    try {
      updateSVarUpdateFunctions(e.getSVars(lt.Transformation).head._2, rb.setGraphicsWorldTransform, rb.getGraphicsWorldTransform.get _)
      updateSVarUpdateFunctions(e.getSVars(gt.Mass).head._2, rb.setMass, rb.getMass _)
      updateSVarUpdateFunctions(e.getSVars(lt.Gravity).head._2, rb.setGravity, rb.getGravity _)
      updateSVarUpdateFunctions(e.getSVars(gt.Restitution).head._2, rb.setRestitution, rb.getRestitution)
      updateSVarUpdateFunctions(e.getSVars(gt.LinearDamping).head._2, rb.setLinearDamping, rb.getLinearDamping)
      updateSVarUpdateFunctions(e.getSVars(gt.AngularDamping).head._2, rb.setAngularDamping, rb.getAngularDamping)
      updateSVarUpdateFunctions(e.getSVars(gt.AngularFactor).head._2, rb.setAngularFactor, rb.getAngularFactor)

      addSVarUpdateFunctions(e.getSVars(lt.Velocity).head._2, Some((x : Vector3f) => rb.setLinearVelocity(x)), Some(rb.getLinearVelocity _))
      addSVarUpdateFunctions(e.getSVars(lt.Acceleration).head._2, Some((x : Vector3f) => rb.setLinearAcceleration(x)), Some(rb.getLinearAcceleration _))

    } catch {
      case e: NoSuchElementException => throw PhysicsException("Entity did not contain the assumed SVars.")
      case exception : Throwable =>
        exception.printStackTrace()
        throw PhysicsException("Error during entity creation.")
    }
  }

  /**
   *  Creates a JBulletRigidBody, connects it to its corresponding SVars and registers it.
   */
  private def createRigidBodyEntity(e: Entity, c : NamedSValSet) {
    try {

      def create(collectedValues: SValSet)  {
        JBulletRigidBody(new NamedSValSet(c.semantics, collectedValues).xMergeWith(c)) match {
          case Some(rb) =>
            registerRigidBody(rb, e)
            connectSVars(rb, e)
          case None => println(PhysicsException("Error during rigid body construction. Ingnoring entity!", Level.warn))
        }
      }

      collectSVars(e, lt.Transformation, gt.Mass, lt.Gravity, gt.Restitution,
        gt.LinearDamping, gt.AngularDamping, gt.AngularFactor)(create, keepRegistered = true)
    }
    catch {
      case e: NoSuchElementException => throw PhysicsException("Entity did not contain the assumed SVars.")
      case exception : Throwable =>
        exception.printStackTrace()
        throw PhysicsException("Error during entity creation.\n" )

    }
  }

  /**
   *  Creates and connects the configuration entity.
   */
  private def createConfigurationEntity(e: Entity, c : NamedSValSet) {
    c.getFirstValueFor(gt.SimulationSpeed).collect{case s => simulationSpeed = s}
    c.getFirstValueFor(lt.Gravity).collect{case g => discreteDynWorld.setGravity(g)}

    addSVarUpdateFunctions(e.getSVars(lt.Gravity).head._2, Some((g: Vector3f) => {discreteDynWorld.setGravity(g)}), None)
    addSVarUpdateFunctions(e.getSVars(gt.SimulationSpeed).head._2, Some((s: Float) => {simulationSpeed = s}), None)
  }

  override def entityConfigComplete(e : Entity, aspect : EntityAspect) {
    aspect.getCreateParams.semantics match {
      case Symbols.component =>
      case Symbols.configuration => createConfigurationEntity(e, aspect.getCreateParams)
      //RigidBody
      case _ => createRigidBodyEntity(e, aspect.getCreateParams)
    }
  }

  /**
   *  Handles incoming events
   */
  override def handleEvent(e: Event) {
    //ApplyImpulse
    if (e.name.value.toSymbol == PhysicsEvents.impulse.name.value.toSymbol){
      e.affectedEntities.headOption.collect{ case entity =>
        execOnRigidBody(entity){ rb =>
          e.get(lt.Impulse).collect{ case impulse =>
            if(!rb.isActive) rb.activate()
            rb.applyCentralImpulse(impulse)
          }
        }
      }
    }
    //ApplyTorqueImpulse
    if (e.name.value.toSymbol == PhysicsEvents.torqueImpulse.name.value.toSymbol){
      e.affectedEntities.headOption.collect{ case entity =>
        execOnRigidBody(entity){ rb =>
          e.get(lt.Impulse).collect{ case impulse =>
            if(!rb.isActive) rb.activate()
            rb.applyTorqueImpulse(impulse)
          }
        }
      }
    }
  }
}

/**
 *  JBullets companion object. Includes helper functionality.
 */
object JBullet {

  private var count = -1

  private[jbullet] def getComponentName : Symbol = synchronized {
    count = count + 1
    Symbol(Symbols.physics.value.toSymbol.name + "#" + count)
  }
}