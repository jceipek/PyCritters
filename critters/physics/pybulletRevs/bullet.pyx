# Copyright (c) Jean-Paul Calderone
# See LICENSE for details.

"""
PyBullet wraps the Bullet Physics library for Python.

Dynamics features are provided mainly by DynamicsWorld and RigidBody.  For
example:

    world = DiscreteDynamicsWorld()
    ball = RigidBody(shape=SphereShape(1.0), mass=1.0)
    world.addRigidBody(ball)
    world.stepSimulation(1.0, 60, 1.0 / 60.0)
    print ball.getWorldTransform().getOrigin()

"""

from libcpp cimport bool

import numpy
cimport numpy


cdef extern from "Python.h":
    cdef void Py_INCREF( object )
    cdef void Py_DECREF( object )

    cdef struct _object:
        pass

    ctypedef _object PyObject



cdef extern from "btBulletCollisionCommon.h":
    ctypedef float btScalar
    ctypedef int bool

    cdef enum PHY_ScalarType:
        PHY_FLOAT
        PHY_DOUBLE
        PHY_INTEGER
        PHY_SHORT
        PHY_FIXEDPOINT88
        PHY_UCHAR

    cdef int _ACTIVE_TAG "ACTIVE_TAG"
    cdef int _ISLAND_SLEEPING "ISLAND_SLEEPING"
    cdef int _WANTS_DEACTIVATION "WANTS_DEACTIVATION"
    cdef int _DISABLE_DEACTIVATION "DISABLE_DEACTIVATION"
    cdef int _DISABLE_SIMULATION "DISABLE_SIMULATION"

    cdef cppclass btVector3


    cdef cppclass btIndexedMesh:
        int m_numTriangles
        unsigned char *m_triangleIndexBase
        int m_triangleIndexStride
        PHY_ScalarType m_indexType

        int m_numVertices
        unsigned char *m_vertexBase
        int m_vertexStride
        PHY_ScalarType m_vertexType


    cdef cppclass btQuaternion


    cdef cppclass btStridingMeshInterface:
        int getNumSubParts()


    cdef cppclass btTriangleIndexVertexArray(btStridingMeshInterface):
        btTriangleIndexVertexArray()
        btTriangleIndexVertexArray(
            int numTriangles,
            int *triangleIndexBase,
            int triangleIndexStride,
            int numVertices,
            btScalar *vertexBase,
            int vertexStride)

        void addIndexedMesh(btIndexedMesh &mesh, PHY_ScalarType indexType)


    cdef cppclass btCollisionShape:
        void calculateLocalInertia(btScalar mass, btVector3 &inertia)


    cdef cppclass btConvexShape(btCollisionShape):
        pass

    cdef cppclass btBoxShape(btConvexShape):
        btBoxShape(btVector3 boxHalfExtents)
        btVector3& getHalfExtentsWithoutMargin()

    cdef cppclass btSphereShape(btConvexShape):
        btSphereShape(btScalar radius)

        btScalar getRadius()


    cdef cppclass btCapsuleShape(btConvexShape):
        btCapsuleShape(btScalar radius, btScalar height)


    cdef cppclass btBvhTriangleMeshShape(btConvexShape):
        btBvhTriangleMeshShape(
            btStridingMeshInterface* meshInterface,
            bool useQuantizedAabbCompression,
            bool buildBvh)

        void buildOptimizedBvh()


cdef extern from "BulletCollision/CollisionShapes/btBox2dShape.h":
    cdef cppclass btBox2dShape(btConvexShape):
        btBox2dShape(btVector3 boxHalfExtents)

cdef extern from "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h":
    cdef cppclass btBroadphaseProxy


cdef extern from "btBulletDynamicsCommon.h":
    cdef cppclass btMatrix3x3:
        btMatrix3x3()
        btMatrix3x3(btQuaternion&)
        btMatrix3x3(btScalar&, btScalar&, btScalar&,
                    btScalar&, btScalar&, btScalar&,
                    btScalar&, btScalar&, btScalar&) # N.B. row-major

        btVector3 getColumn(int)
        btVector3& getRow(int)
        
        void setValue(btScalar&, btScalar&, btScalar&,
                      btScalar&, btScalar&, btScalar&,
                      btScalar&, btScalar&, btScalar&) # N.B. row-major
        void setRotation(btQuaternion&)
        void setEulerYPR(btScalar&, btScalar&, btScalar&)
        void setEulerZYX(btScalar, btScalar, btScalar)
        void setIdentity()
        
        void getRotation(btQuaternion&)
        void getEulerYPR(btScalar&, btScalar&, btScalar&)
        void getEulerZYX(btScalar&, btScalar&, btScalar&, int)        
        
        btMatrix3x3 scaled(btVector3&)
        btScalar determinant()
        btMatrix3x3 adjoint()
        btMatrix3x3 absolute()
        btMatrix3x3 transpose()
        btMatrix3x3 inverse()


    cdef cppclass btTransform:
        btVector3 getOrigin()
        void setOrigin(btVector3)
        void setIdentity()
        void setRotation(btQuaternion&)
        btQuaternion getRotation()

        btMatrix3x3& getBasis()
        
        void getOpenGLMatrix(btScalar* arr)

    cdef cppclass btMotionState:
        void getWorldTransform(btTransform &transform)
        void setWorldTransform(btTransform &transform)


    cdef cppclass btDefaultMotionState(btMotionState):
        btDefaultMotionState()


    cdef cppclass btCollisionObject:
        btCollisionObject()

        btCollisionShape* getCollisionShape()
        void setCollisionShape(btCollisionShape*)

        btScalar getFriction()
        void setFriction(btScalar)

        void setRestitution(btScalar)
        btScalar getRestitution()

        btTransform& getInterpolationWorldTransform()
        btTransform& getWorldTransform()
        void setWorldTransform(btTransform& worldTrans)

        void setContactProcessingThreshold(btScalar)
        btScalar getContactProcessingThreshold()

        btBroadphaseProxy* getBroadphaseHandle()

        int getActivationState()
        void setActivationState(int newState)


    cdef cppclass btRigidBody(btCollisionObject)


    cdef cppclass btActionInterface:
        pass


    cdef cppclass btCharacterControllerInterface(btActionInterface):
        void setWalkDirection(btVector3 walkDirection)

        void setVelocityForTimeInterval(
            btVector3 velocity, btScalar timeInterval)

cdef extern from "BulletDynamics/ConstraintSolver/btTypedConstraint.h":

    cdef enum btTypedConstraintType:    
        POINT2POINT_CONSTRAINT_TYPE
        HINGE_CONSTRAINT_TYPE
        CONETWIST_CONSTRAINT_TYPE
        D6_CONSTRAINT_TYPE
        SLIDER_CONSTRAINT_TYPE
        CONTACT_CONSTRAINT_TYPE 

    cdef cppclass btTypedConstraint:
    
        btRigidBody &   getRigidBodyA ()
        btRigidBody &   getRigidBodyB ()
        int     getUserConstraintType () 
        void    setUserConstraintType (int userConstraintType)
        void    setUserConstraintId (int uid)
        int     getUserConstraintId () 
        int     getUid () 
        bool    needsFeedback () 
        void    enableFeedback (bool needsFeedback)
        btScalar    getAppliedImpulse ()
        btTypedConstraintType   getConstraintType ()


cdef extern from "BulletCollision/CollisionShapes/btCompoundShape.h":
    cdef cppclass btCompoundShape(btCollisionShape):
        btCompoundShape(bool)
        
        void addChildShape(btTransform&, btCollisionShape*)
        void removeChildShape(btCollisionShape*)


cdef extern from "BulletCollision/CollisionShapes/btCylinderShape.h":
    cdef cppclass btCylinderShape(btConvexShape):
        btCylinderShape()
        btCylinderShape(btVector3&)
        btVector3& getHalfExtentsWithoutMargin()
        btScalar getRadius()

    cdef cppclass btCylinderShapeX(btCylinderShape):
        btCylinderShapeX(btVector3&)

    cdef cppclass btCylinderShapeZ(btCylinderShape):
        btCylinderShapeZ(btVector3&)



cdef extern from "BulletCollision/CollisionShapes/btStaticPlaneShape.h":
    cdef cppclass btStaticPlaneShape(btCollisionShape):
        btStaticPlaneShape(btVector3 &planeNormal, btScalar planeConstant)



cdef extern from "btBulletCollisionCommon.h":
    cdef cppclass btDispatcher:
        pass



cdef extern from "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h":
    cdef cppclass btOverlappingPairCallback:
        pass


    cdef cppclass btGhostPairCallback(btOverlappingPairCallback):
        pass


    cdef cppclass btOverlappingPairCache:
        void setInternalGhostPairCallback(btOverlappingPairCallback*)


    cdef cppclass btHashedOverlappingPairCache(btOverlappingPairCache):
        pass


cdef extern from "BulletCollision/CollisionDispatch/btGhostObject.h":
    cdef cppclass btPairCachingGhostObject(btCollisionObject):
        pass


cdef extern from "BulletDynamics/Character/btKinematicCharacterController.h":
    cdef cppclass btKinematicCharacterController(btCharacterControllerInterface):
        btKinematicCharacterController(
            btPairCachingGhostObject *ghostObject,
            btConvexShape *convexShape,
            btScalar stepHeight, int upAxis)

        void warp(btVector3 origin)



cdef extern from "btBulletCollisionCommon.h" namespace "btRigidBody":
    cdef cppclass btRigidBodyConstructionInfo:
        btRigidBodyConstructionInfo(
            btScalar mass,
            btMotionState *motionState,
            btCollisionShape *collisionShape)
        btRigidBodyConstructionInfo(
            btScalar mass,
            btMotionState *motionState,
            btCollisionShape *collisionShape,
            btVector3 localInteria)

        btScalar m_additionalAngularDampingFactor
        btScalar m_additionalAngularDampingThresholdSqr
        bool m_additionalDamping
        btScalar m_additionalDampingFactor
        btScalar m_additionalLinearDampingThresholdSqr
        btScalar m_angularDamping
        btScalar m_angularSleepingThreshold
        btCollisionShape* m_collisionShape
        btScalar m_friction
        btScalar m_linearDamping
        btScalar m_linearSleepingThreshold
        btVector3 m_localInertia
        btScalar m_mass
        btMotionState* m_motionState
        btScalar m_restitution
        btTransform m_startWorldTransform



cdef extern from "LinearMath/btIDebugDraw.h":

    cdef cppclass btIDebugDraw:
        pass

cdef extern from "LinearMath/btIDebugDraw.h" namespace "btIDebugDraw":
        cdef enum DebugDrawModes:
            DBG_NoDebug
            DBG_DrawWireframe
            DBG_DrawAabb
            DBG_DrawFeaturesText
            DBG_DrawContactPoints
            DBG_DrawText
            DBG_DrawConstraints
            DBG_DrawConstraintLimits


NO_DEBUG = DBG_NoDebug
DRAW_WIREFRAME = DBG_DrawWireframe
DRAW_AABB = DBG_DrawAabb
DRAW_FEATURES_TEXT = DBG_DrawFeaturesText
DRAW_CONTACT_POINTS = DBG_DrawContactPoints
DRAW_TEXT = DBG_DrawText
DRAW_CONSTRAINTS = DBG_DrawConstraints
DRAW_CONSTRAINT_LIMITS = DBG_DrawConstraintLimits

cdef extern from "btBulletCollisionCommon.h":
    cdef cppclass btCollisionConfiguration:
        pass

    cdef cppclass btDefaultCollisionConfiguration(btCollisionConfiguration):
        pass

    cdef cppclass btDispatcher:
        pass

    cdef cppclass btCollisionDispatcher(btDispatcher):
        btCollisionDispatcher(btCollisionConfiguration*)

    cdef cppclass btVector3:
        btVector3()
        btVector3(btScalar, btScalar, btScalar)

        void setX(btScalar x)
        void setY(btScalar y)
        void setZ(btScalar z)

        btScalar getX()
        btScalar getY()
        btScalar getZ()

        btVector3& normalize()
        btVector3 cross(btVector3&)
        btScalar dot(btVector3&)

        # btVector3& operator+=(btScalar&)
        # btVector3& operator*=(btScalar&)


    cdef cppclass btQuaternion:
        btQuaternion()
        btQuaternion(btScalar x, btScalar y, btScalar z, btScalar w)
        btQuaternion(btVector3 axis, btScalar angle)

        btScalar getX()
        btScalar getY()
        btScalar getZ()
        btScalar getW()

        btVector3 getAxis()
        btScalar getAngle()
        btScalar angle(btQuaternion& other)
        btQuaternion operator* (btQuaternion)


    cdef cppclass btBroadphaseInterface:
        btOverlappingPairCache* getOverlappingPairCache()
    
    cdef cppclass btOverlappingPairCache   #LOOK

    cdef cppclass btAxisSweep3(btBroadphaseInterface):
        btAxisSweep3(btVector3, btVector3, unsigned short int maxHandles,
                     btOverlappingPairCache *pairCache,
                     bool disableRaycastAccelerator)
        btOverlappingPairCache* getOverlappingPairCache()

    cdef cppclass btDbvtBroadphase(btBroadphaseInterface):
        pass


    cdef cppclass btRigidBody(btCollisionObject):
        btRigidBody(btRigidBodyConstructionInfo)

        bool isInWorld()

        btScalar getInvMass()
        btVector3& getInvInertiaDiagLocal()

        btMotionState* getMotionState()

        void setAngularFactor(btScalar angFac)
        btScalar getAngularDamping()
        btScalar getAngularSleepingThreshold()

        void setLinearVelocity(btVector3 velocity)
        btVector3& getLinearVelocity()
        btScalar getLinearDamping()
        btScalar getLinearSleepingThreshold()

        void applyCentralForce(btVector3 force)
        void applyForce(btVector3 force, btVector3 relativePosition)

        void applyCentralImpulse(btVector3 impulse)
        void applyImpulse(btVector3 impulse, btVector3 relativePosition)
        void applyTorque(btVector3& torque)
        void applyTorqueImpulse(btVector3& torque)
        void setCenterOfMassTransform(btTransform& trans)
        btTransform& getCenterOfMassTransform()
        
        void setAngularVelocity(btVector3& velocity)
        btVector3& getAngularVelocity()
        
        btQuaternion getOrientation()

    cdef cppclass btCollisionWorld:
        btCollisionWorld(
            btDispatcher*, btBroadphaseInterface*, btCollisionConfiguration*)

        void setDebugDrawer(btIDebugDraw *debugDrawer)
        void debugDrawWorld()

        btDispatcher *getDispatcher()
        btBroadphaseInterface *getBroadphase()

        int getNumCollisionObjects()

        void addCollisionObject(btCollisionObject*, short int, short int)
        void removeCollisionObject(btCollisionObject*)

cdef extern from "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h":
    cdef cppclass btOverlappingPairCache:

        void cleanProxyFromPairs(btBroadphaseProxy *proxy, btDispatcher *dispatcher)

cdef extern from "btBulletDynamicsCommon.h":
    cdef cppclass btConstraintSolver:
        pass


    cdef cppclass btSequentialImpulseConstraintSolver(btConstraintSolver):
        btSequentialImpulseConstraintSolver()


    cdef cppclass btDynamicsWorld: # (btCollisionWorld):
        btDynamicsWorld(
            btDispatcher*, btBroadphaseInterface*, btCollisionConfiguration*)

        void setGravity(btVector3)
        btVector3 getGravity()
        
        void addConstraint(btTypedConstraint*, bool)
        void removeConstraint(btTypedConstraint*)
        
        void addRigidBody(btRigidBody*)
        void removeRigidBody(btRigidBody*)

        void addAction(btActionInterface*)
        void removeAction(btActionInterface*)

        int stepSimulation(btScalar, int, btScalar)


    cdef cppclass btDiscreteDynamicsWorld: # (byDynamicsWorld)
        btDiscreteDynamicsWorld(
            btDispatcher*, btBroadphaseInterface*,
            btConstraintSolver*, btCollisionConfiguration*)

        btConstraintSolver *getConstraintSolver()


cdef extern from "bulletdebugdraw.h":
    cdef cppclass PythonDebugDraw(btIDebugDraw):
        PythonDebugDraw(PyObject *debugDraw)

cdef extern from "BulletDynamics/Vehicle/btVehicleRaycaster.h" namespace "btVehicleRaycaster":

    cdef cppclass btVehicleRaycasterResult:
        btVector3 m_hitPointInWorld
        btVector3 m_hitNormalInWorld
        btScalar m_distFraction
        

cdef extern from "BulletDynamics/Vehicle/btVehicleRaycaster.h":  
        
    cdef cppclass btVehicleRaycaster:
        pass
            

cdef extern from "BulletDynamics/Vehicle/btWheelInfo.h" namespace "btWheelInfo":
    
    # N.B. the namespace appears to be lost if we just put in "RaycastInfo",
    # so we have to work around it like this. TODO file a bug with cython.
    cdef cppclass btRaycastInfo "btWheelInfo::RaycastInfo":
        btVector3   m_contactNormalWS
        btVector3   m_contactPointWS
        btScalar    m_suspensionLength
        btVector3   m_hardPointWS
        btVector3   m_wheelDirectionWS
        btVector3   m_wheelAxleWS
        bool    m_isInContact
        void *  m_groundObject


cdef extern from "BulletDynamics/Vehicle/btWheelInfo.h":    

    cdef cppclass btWheelInfoConstructionInfo:
        btVector3   m_chassisConnectionCS
        btVector3   m_wheelDirectionCS
        btVector3   m_wheelAxleCS
        btScalar    m_suspensionRestLength
        btScalar    m_maxSuspensionTravelCm
        btScalar    m_wheelRadius
        btScalar    m_suspensionStiffness
        btScalar    m_wheelsDampingCompression
        btScalar    m_wheelsDampingRelaxation
        btScalar    m_frictionSlip
        btScalar    m_maxSuspensionForce
        bool    m_bIsFrontWheel
    
    cdef cppclass btWheelInfo:
        btScalar    getSuspensionRestLength ()
        btWheelInfo (btWheelInfoConstructionInfo &ci)
        void    updateWheel (btRigidBody &chassis, btRaycastInfo &raycastInfo)        
        btRaycastInfo   m_raycastInfo
        btTransform     m_worldTransform
        btVector3   m_chassisConnectionPointCS
        btVector3   m_wheelDirectionCS
        btVector3   m_wheelAxleCS
        btScalar    m_suspensionRestLength1
        btScalar    m_maxSuspensionTravelCm
        btScalar    m_wheelsRadius
        btScalar    m_suspensionStiffness
        btScalar    m_wheelsDampingCompression
        btScalar    m_wheelsDampingRelaxation
        btScalar    m_frictionSlip
        btScalar    m_steering
        btScalar    m_rotation
        btScalar    m_deltaRotation
        btScalar    m_rollInfluence
        btScalar    m_maxSuspensionForce
        btScalar    m_engineForce
        btScalar    m_brake
        bool    m_bIsFrontWheel
        void *  m_clientInfo
        btScalar    m_clippedInvContactDotSuspension
        btScalar    m_suspensionRelativeVelocity
        btScalar    m_wheelsSuspensionForce
        btScalar    m_skidInfo


cdef extern from "BulletDynamics/Vehicle/btRaycastVehicle.h" namespace "btRaycastVehicle":

    cdef cppclass btVehicleTuning:
        btScalar m_suspensionStiffness
        btScalar m_suspensionCompression
        btScalar m_suspensionDamping
        btScalar m_maxSuspensionTravelCm
        btScalar m_frictionSlip
        btScalar m_maxSuspensionForce


cdef extern from "BulletDynamics/Vehicle/btRaycastVehicle.h":

    cdef cppclass btDefaultVehicleRaycaster(btVehicleRaycaster):
        btDefaultVehicleRaycaster(btDynamicsWorld* world)
        void* castRay(btVector3& fromVector, btVector3& toVector, btVehicleRaycasterResult& result)
            
    cdef cppclass btRaycastVehicle(btActionInterface):
        btRaycastVehicle(btVehicleTuning &tuning, btRigidBody *chassis, btVehicleRaycaster *raycaster)

        btScalar rayCast(btWheelInfo &wheel)

        btTransform& getWheelTransformWS(int wheelIndex) 
        btTransform& getChassisWorldTransform()

        void resetSuspension()

        #void setRaycastWheelInfo(int wheelIndex, bool isInContact, btVector3 &hitPoint, btVector3 &hitNormal, btScalar depth)
        btWheelInfo& getWheelInfo(int index) 
        int getNumWheels() 
        btWheelInfo& addWheel(btVector3 &connectionPointCS0, btVector3 &wheelDirectionCS0, btVector3 &wheelAxleCS, btScalar suspensionRestLength, btScalar wheelRadius, btVehicleTuning &tuning, bool isFrontWheel)

        btScalar getSteeringValue(int wheel) 

        void setSteeringValue(btScalar steering, int wheel)
        void applyEngineForce(btScalar force, int wheel)
        void setBrake(btScalar brake, int wheelIndex)
        void setPitchControl(btScalar pitch)

        void updateWheelTransform(int wheelIndex, bool interpolatedTransform)
        void updateWheelTransformsWS(btWheelInfo &wheel, bool interpolatedTransform)
        void updateVehicle(btScalar step)
        void updateSuspension(btScalar deltaTime)
        void updateFriction(btScalar timeStep)

        btRigidBody* getRigidBody()

        btVector3 getForwardVector() 
        btScalar getCurrentSpeedKmHour() 
        
        void setCoordinateSystem(int rightIndex, int upIndex, int forwardIndex)    
        int getRightAxis() 
        int getUpAxis() 
        int getForwardAxis() 


cdef extern from "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h":

    cdef cppclass btRotationalLimitMotor:
        btRotationalLimitMotor ()
        bool    isLimited ()
        bool    needApplyTorques ()
        int     testLimitValue (btScalar test_value)
        
        btScalar    m_loLimit
        btScalar    m_hiLimit
        btScalar    m_targetVelocity
        btScalar    m_maxMotorForce
        btScalar    m_maxLimitForce
        btScalar    m_damping
        btScalar    m_limitSoftness
        btScalar    m_normalCFM
        btScalar    m_stopERP
        btScalar    m_stopCFM
        btScalar    m_bounce
        bool    m_enableMotor
        btScalar    m_currentLimitError
        btScalar    m_currentPosition
        int     m_currentLimit
        btScalar    m_accumulatedImpulse

    cdef cppclass btGeneric6DofConstraint(btTypedConstraint):
        
        btGeneric6DofConstraint() # XXX won't compile without this
        
        btGeneric6DofConstraint(btRigidBody&, btRigidBody&, btTransform&, btTransform&, bool)
        btGeneric6DofConstraint(btRigidBody&, btTransform&, bool)
        void    calculateTransforms (btTransform &transA, btTransform &transB)
        void    calculateTransforms ()
        btTransform &   getCalculatedTransformA ()
        btTransform &   getCalculatedTransformB ()
        btTransform &   getFrameOffsetA ()
        btTransform &   getFrameOffsetB ()
        void    buildJacobian ()

        void    updateRHS (btScalar timeStep)
        btVector3   getAxis (int axis_index)
        btScalar    getAngle (int axis_index)
        btScalar    getRelativePivotPosition (int axis_index)
        bool    testAngularLimitMotor (int axis_index)
        void    setLinearLowerLimit (btVector3 &linearLower)
        void    setLinearUpperLimit (btVector3 &linearUpper)
        void    setAngularLowerLimit (btVector3 &angularLower)
        void    setAngularUpperLimit (btVector3 &angularUpper)
        btRotationalLimitMotor *    getRotationalLimitMotor (int index)
        # TODO btTranslationalLimitMotor *  getTranslationalLimitMotor ()
        void    setLimit (int axis, btScalar lo, btScalar hi)
        bool    isLimited (int limitIndex)
        void    calcAnchorPos()
        bool    getUseFrameOffset ()
        void    setUseFrameOffset (bool frameOffsetOnOff)
        # XXX problem with default args again...
        void    setParam (int num, btScalar value, int axis)
        btScalar    getParam (int num, int axis)


cdef extern from "BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h":

    cdef cppclass btGeneric6DofSpringConstraint(btGeneric6DofConstraint):

        btGeneric6DofSpringConstraint() # XXX won't compile without this
        
        btGeneric6DofSpringConstraint (btRigidBody &rbA, btRigidBody &rbB, btTransform &frameInA, btTransform &frameInB, bool useLinearReferenceFrameA)
        void    enableSpring (int index, bool onOff)
        void    setStiffness (int index, btScalar stiffness)
        void    setDamping (int index, btScalar damping)
        void    setEquilibriumPoint ()
        void    setEquilibriumPoint (int index)        


cdef extern from "BulletDynamics/ConstraintSolver/btHinge2Constraint.h":

    cdef cppclass btHinge2Constraint(btGeneric6DofSpringConstraint):

        btHinge2Constraint (btRigidBody &rbA, btRigidBody &rbB, btVector3 &anchor, btVector3 &axis1, btVector3 &axis2)
        btVector3 &     getAnchor ()
        btVector3 &     getAnchor2 ()
        btVector3 &     getAxis1 ()
        btVector3 &     getAxis2 ()
        btScalar    getAngle1 ()
        btScalar    getAngle2 ()
        void    setUpperLimit (btScalar ang1max)
        void    setLowerLimit (btScalar ang1min)

# Forward declare some things because of circularity in the API.
cdef class CollisionObject
cdef class CollisionDispatcher
cdef class TypedConstraint



cdef class Vector3:
    """
    A Vector3 represents a point or vector in three-dimensional space.

    When dealing with most vector quantities such as velocity, position, and
    force, bullet will use a Vector3.

    This class is loosely a wrapper around btVector3.
    """
    cdef readonly btScalar x
    cdef readonly btScalar y
    cdef readonly btScalar z

    def __cinit__(self, btScalar x, btScalar y, btScalar z):
        self.x = x
        self.y = y
        self.z = z

    def __getitem__(self, ind):
        return [self.x, self.y, self.z][ind]

    def __repr__(self):
        return '<Vector x=%s y=%s z=%s>' % (self.x, self.y, self.z)


    def __mul__(Vector3 self, scale):
        """
        Support multiplication of a Vector3 by a scalar.
        """
        scale = float(scale)
        return Vector3(self.x * scale, self.y * scale, self.z * scale)


    def __add__(Vector3 self, Vector3 other not None):
        """
        Support addition of two Vector3s.
        """
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)


    def __sub__(Vector3 self, Vector3 other not None):
        """
        Support subtraction of two Vector3s.
        """
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)


    def normalized(self):
        """
        Return a new normalized L{Vector3} pointing in the same direction as
        this one.
        """
        cdef btVector3 v = btVector3(self.x, self.y, self.z)
        v.normalize()
        return Vector3(v.getX(), v.getY(), v.getZ())


    def cross(self, Vector3 other not None):
        """
        Return the vector cross product of this vector and the other one.
        """
        cdef btVector3 v1 = btVector3(self.x, self.y, self.z)
        cdef btVector3 v2 = btVector3(other.x, other.y, other.z)
        cdef btVector3 result = v1.cross(v2)
        return Vector3(result.getX(), result.getY(), result.getZ())


    def dot(self, Vector3 other not None):
        """
        Return the dot product of this vector and the other one.
        """
        cdef btVector3 v1 = btVector3(self.x, self.y, self.z)
        cdef btVector3 v2 = btVector3(other.x, other.y, other.z)
        return v1.dot(v2)



cdef class Quaternion:
    """
    A Quaternion represents a point or vector in four-dimensional space.  It is
    frequently also used to represent rotation and orientation in
    three-dimensional space.

    This class is a wrapper around btQuaternion.
    """
    cdef btQuaternion* quaternion

    def __cinit__(self):
        self.quaternion = new btQuaternion()

    def __dealloc__(self):
        del self.quaternion


    @classmethod
    def fromScalars(cls, btScalar x, btScalar y, btScalar z, btScalar w):
        """
        Construct a new Quaternion from four scalar components.
        """
        q = Quaternion()
        q.quaternion = new btQuaternion(x, y, z, w)
        return q


    @classmethod
    def fromAxisAngle(cls, Vector3 axis not None, btScalar angle):
        """
        Construct a new Quaternion from an axis and a rotation around that axis.
        """
        q = Quaternion()
        q.quaternion = new btQuaternion(
            btVector3(axis.x, axis.y, axis.z), angle)
        return q


    def __mul__(Quaternion self, Quaternion other not None):
        """
        Support multiplication of two Quaternions.
        """
        cdef btQuaternion product = self.quaternion[0] * other.quaternion[0]
        result = Quaternion()
        result.quaternion = new btQuaternion(
            product.getAxis(), product.getAngle())
        return result


    def getX(self):
        """
        Get the X component of the Quaternion.
        """
        return self.quaternion.getX()


    def getY(self):
        """
        Get the Y component of the Quaternion.
        """
        return self.quaternion.getY()


    def getZ(self):
        """
        Get the Z component of the Quaternion.
        """
        return self.quaternion.getZ()


    def getW(self):
        """
        Get the W component of the Quaternion.
        """
        return self.quaternion.getW()


    def getAxis(self):
        """
        Return the axis of rotation represented by this quaternion.
        """
        cdef btVector3 axis = self.quaternion.getAxis()
        return Vector3(axis.getX(), axis.getY(), axis.getZ())


    def getAngle(self):
        """
        Return the angle of rotation represented by this quaternion.
        """
        return self.quaternion.getAngle()



cdef class CollisionShape:
    """
    A CollisionShape defines the shape of an object in a CollisionWorld.  Shapes
    are used for collision detection.

    This class is a wrapper around btCollisionShape.
    """
    cdef btCollisionShape *thisptr


    def __dealloc__(self):
        del self.thisptr



cdef class ConvexShape(CollisionShape):
    """
    A ConvexShape is a shape that curves outwards only.  This is a base class
    for many other types of shapes which are convex.

    This class is loosely a wrapper around btConvexShape.
    """



cdef class Box2dShape(ConvexShape):
    """
    A Box2dShape is a box primitive around the origin, its sides axis aligned
    with length specified by half extents, in local shape coordinates.  It has
    size only in the X and Y dimensions.

    This class is a wrapper around btBox2dShape.
    """
    def __cinit__(self, Vector3 boxHalfExtents):
        self.thisptr = new btBox2dShape(
            btVector3(boxHalfExtents.x, boxHalfExtents.y, boxHalfExtents.z))



cdef class BoxShape(ConvexShape):
    """
    A Box2dShape is a box primitive around the origin, its sides axis aligned
    with length specified by half extents, in local shape coordinates.  It has
    size only in the X and Y dimensions.

    This class is a wrapper around btBox2dShape.
    """
    def __cinit__(self, Vector3 boxHalfExtents):
        self.thisptr = new btBoxShape(
            btVector3(boxHalfExtents.x, boxHalfExtents.y, boxHalfExtents.z))



cdef class SphereShape(ConvexShape):
    """
    A SphereShape is a sphere around the origin, with a specified radius.

    This class is a wrapper around btSphereShape.
    """
    def __cinit__(self, btScalar radius):
        self.thisptr = new btSphereShape(radius)


    def getRadius(self):
        """
        Return the radius of the sphere.
        """
        return (<btSphereShape*>self.thisptr).getRadius()



cdef class CapsuleShape(ConvexShape):
    """
    A CapsuleShape is a capsule around the Y axis.  A capsule is a cylinder with
    a sphere on each end.

    This class is a wrapper around btCapsuleShape.
    """
    def __cinit__(self, btScalar radius, btScalar height):
        self.thisptr = new btCapsuleShape(radius, height)



cdef class CylinderShape(ConvexShape):
    """
    A CylinderShape is a cylinder with its central axis aligned with the Y axis.

    This class is a wrapper around btCylinderShape.
    """
    def __init__(self, Vector3 halfExtents not None):
        self.thisptr = new btCylinderShape(
            btVector3(halfExtents.x, halfExtents.y, halfExtents.z))


    def getRadius(self):
        return (<btCylinderShape*>self.thisptr).getRadius()


    def getHalfExtentsWithoutMargin(self):
        cdef btVector3 v = (<btCylinderShape*>self.thisptr).getHalfExtentsWithoutMargin()
        return Vector3(v.getX(), v.getY(), v.getZ())



cdef class CylinderShapeX(CylinderShape):
    """
    A CylinderShapeX is a cylinder with its central axis aligned with the X
    axis.

    This class is a wrapper around btCylinderShapeX.
    """
    def __init__(self, Vector3 halfExtents not None):
        self.thisptr = new btCylinderShapeX(
            btVector3(halfExtents.x, halfExtents.y, halfExtents.z))



cdef class CylinderShapeZ(CylinderShape):
    """
    A CylinderShapeZ is a cylinder with its central axis aligned with the Z
    axis.

    This class is a wrapper around btCylinderShapeZ.
    """
    def __init__(self, Vector3 halfExtents not None):
        self.thisptr = new btCylinderShapeZ(
            btVector3(halfExtents.x, halfExtents.y, halfExtents.z))



cdef class StaticPlaneShape(CollisionShape):
    """
    A StaticPlaneShape is an immobile plane.

    This class is a wrapper around btStaticPlaneShape.
    """
    def __cinit__(self, Vector3 normal not None, btScalar constant):
        self.thisptr = new btStaticPlaneShape(
            btVector3(normal.x, normal.y, normal.z), constant)



cdef class IndexedMesh:
    """
    An IndexedMesh is a vertex array and an array of index data into that
    vertex array.  It defines a mesh of triangles composed of triples of vertex
    data given by sequential triples of indices from the index array.  For
    example, an IndexedMesh defining two triangles would use an vertex array
    like::

        numpy.array([0, 0, 0,
                     1, 0, 0,
                     1, 0, 1,
                     0, 0, 1], 'f')

    and an index array like this::

        numpy.array([0, 1, 2,
                     2, 3, 0], 'i')

    This class is a wrapper around btIndexedMesh.
    """
    cdef btIndexedMesh* thisptr

    cdef PHY_ScalarType _dtypeToScalarType(self, numpy.ndarray array):
        cdef char *dname = array.dtype.char
        cdef char dtype = dname[0]

        if dtype == 'f':
            return PHY_FLOAT
        elif dtype == 'd':
            return PHY_DOUBLE
        elif dtype == 'i':
            return PHY_INTEGER
        elif dtype == 'h':
            return PHY_SHORT
        return <PHY_ScalarType>-1


    def __cinit__(self):
        self.thisptr = new btIndexedMesh()
        self.thisptr.m_numTriangles = 0
        self.thisptr.m_triangleIndexBase = NULL
        self.thisptr.m_triangleIndexStride = 0
        self.thisptr.m_numVertices = 0
        self.thisptr.m_vertexBase = NULL
        self.thisptr.m_vertexStride = 0
        self.thisptr.m_indexType = PHY_FLOAT
        self.thisptr.m_vertexType = PHY_FLOAT


    def setIndices(self, int numTriangles, int indexStride,
                   numpy.ndarray indexBase not None):
        """
        Specify the index data for for this IndexedMesh.

        numTriangles specifies the total number of triangles this mesh will
        contain.

        indexStride gives the distance in bytes between the start of each triple
        of values defining a triangle.

        indexBase is a numpy array giving the index data itself.
        """
        cdef PHY_ScalarType indexType = self._dtypeToScalarType(indexBase)
        if indexType == -1:
            raise ValueError("Unsupported index array type")

        self.thisptr.m_numTriangles = numTriangles
        self.thisptr.m_triangleIndexStride = indexStride
        self.thisptr.m_triangleIndexBase = <unsigned char*>indexBase.data
        self.thisptr.m_indexType = indexType


    def setVertices(self, int numVertices, int vertexStride,
                    numpy.ndarray vertexBase not None):
        """
        Specify the vertex data for this IndexedMesh.

        numVertices specifies the total number of vertices this mesh will
        contain.

        vertexStride gives the distance in bytes between the start of each
        triple of values defining a vertex.

        vertexBase is a numpy array giving the vertex data itself.
        """
        cdef PHY_ScalarType vertexType = self._dtypeToScalarType(vertexBase)
        if vertexType == -1:
            raise ValueError("Unsupported index array type")

        self.thisptr.m_numVertices = numVertices
        self.thisptr.m_vertexStride = vertexStride
        self.thisptr.m_vertexBase = <unsigned char*>vertexBase.data
        self.thisptr.m_vertexType = vertexType


    def __dealloc__(self):
        del self.thisptr



cdef class StridingMeshInterface:
    """
    A StridingMeshInterface is an object suitable for use in defining a triangle
    mesh for BvhTriangleMeshShape.

    This class is loosely a wrapper around btStridingMeshInterface.

    XXX THIS WRAPPER MAY CAUSE SEGFAULTS.  Use TriangleIndexVertexArray instead.
    """
    cdef btStridingMeshInterface *thisptr

    def __dealloc__(self):
        del self.thisptr


    def getNumSubParts(self):
        """
        Return the number of separate continuous vertex arrays are part of this
        StridingMeshInterface.
        """
        return self.thisptr.getNumSubParts()



cdef class TriangleIndexVertexArray(StridingMeshInterface):
    """
    A TriangleIndexVertexArray is a striding mesh defined in terms of an array
    of IndexedMesh instances.

    Construct a TriangleIndexVertexArray and add one or more IndexedMesh
    instances to it to define a triangle mesh for a BvhTriangleMeshShape.

    This class is a wrapper around btTriangleIndexVertexArray.
    """
    def __cinit__(self):
        self.thisptr = new btTriangleIndexVertexArray()


    def addIndexedMesh(self, IndexedMesh mesh not None):
        """
        Add another IndexedMesh to this index/vertex array.

        XXX When is it necessary to rebuild the optimized bvh on
        BvhTriangleMeshShape with respect to changes to this
        TriangleIndexVertexArray.
        """
        if mesh.thisptr.m_vertexType == PHY_INTEGER:
            raise ValueError("XXX")
        cdef btTriangleIndexVertexArray *array
        array = <btTriangleIndexVertexArray*>self.thisptr
        array.addIndexedMesh(mesh.thisptr[0], mesh.thisptr.m_indexType)



cdef class BvhTriangleMeshShape(ConvexShape):
    """
    A BvhTriangleMeshShape is a shape defined by an array of triangle vertex
    data and an array of indices against those triangles.

    This class is a wrapper around btBvhTriangleMeshShape.
    """
    cdef StridingMeshInterface stride

    def __init__(self, StridingMeshInterface mesh not None):
        self.stride = mesh
        self.thisptr = new btBvhTriangleMeshShape(mesh.thisptr, True, False)


    def buildOptimizedBvh(self):
        """
        Build the internal optimized Bounding Volume Hierarchy structure to
        allow fast collision detection between this and other shapes.

        XXX You probably have to call this if you ever change the underlying
        mesh.  But I don't know.

        XXX You also have to call it before you try to use this shape for
        collision detection.

        XXX You also must put data into the underlying mesh or an assert will
        fail in bullet.
        """
        # XXX This is executed by the test suite, but it's not actually tested.
        (<btBvhTriangleMeshShape*>self.thisptr).buildOptimizedBvh()



cdef class Transform:
    """
    A Transform represents an a translation and rotation.

    When dealing with the location and orientation of an object in a
    CollisionWorld, Bullet will use a Transform instance.

    This class is a wrapper around btTransform.  Unlike btTransform, a Transform
    always starts off set to the identity.
    """
    cdef btTransform *thisptr

    def __cinit__(self):
        self.thisptr = new btTransform()
        self.thisptr.setIdentity()


    def __dealloc__(self):
        del self.thisptr


    def getOrigin(self):
        """
        Get the origin vector translation as a Vector3.
        """
        cdef btVector3 origin = self.thisptr.getOrigin()
        return Vector3(origin.getX(), origin.getY(), origin.getZ())


    def setOrigin(self, Vector3 origin not None):
        """
        Set the origin vector translation for this Transform.
        """
        self.thisptr.setOrigin(btVector3(origin.x, origin.y, origin.z))


    def setRotation(self, Quaternion rot not None):
        """
        Set the rotation of this Transform using a Quaternion.
        """
        cdef btQuaternion *quat = rot.quaternion
        self.thisptr.setRotation(quat[0])


    def getRotation(self):
        """
        Get the rotation of this Transform as a Quaternion.
        """
        cdef btQuaternion quat = self.thisptr.getRotation()
        return Quaternion.fromScalars(
            quat.getX(), quat.getY(), quat.getZ(), quat.getW())


    def setIdentity(self):
        """
        Set this Transform to be the identity.
        """
        self.thisptr.setIdentity()


ACTIVE_TAG = _ACTIVE_TAG
ISLAND_SLEEPING = _ISLAND_SLEEPING
WANTS_DEACTIVATION = _WANTS_DEACTIVATION
DISABLE_DEACTIVATION = _DISABLE_DEACTIVATION
DISABLE_SIMULATION = _DISABLE_SIMULATION


cdef class CollisionObject:
    """
    A CollisionObject is something which can be collided with when added to a
    CollisionWorld.  It has a position and orientation as well as a
    CollisionShape.

    This class is a wrapper around btCollisionObject.
    """
    cdef btCollisionObject *thisptr
    cdef CollisionShape _shape

    def __init__(self):
        self.thisptr = new btCollisionObject()


    def __dealloc__(self):
        del self.thisptr


    def getFriction(self):
        """
        Return the friction value for this L{CollisionObject}.
        """
        return self.thisptr.getFriction()


    def setFriction(self, btScalar friction):
        """
        Change the friction value for this L{CollisionObject}.
        """
        self.thisptr.setFriction(friction)


    def setRestitution(self, btScalar restitution):
        """
        Specify a scaling factor to be applied to the normal impulse computed
        when this object collides with another.
        """
        self.thisptr.setRestitution(restitution)


    def getRestitution(self):
        """
        Get the restitution value for this object.
        """
        return self.thisptr.getRestitution()


    def getCollisionShape(self):
        """
        Get the CollisionShape associated with this object.
        """
        return self._shape


    def setCollisionShape(self, CollisionShape collisionShape):
        """
        Replace this object's CollisionShape.
        """
        self.thisptr.setCollisionShape(collisionShape.thisptr)
        self._shape = collisionShape


    def getWorldTransform(self):
        """
        Get a copy of the transformation for this CollisionObject as a Transform
        instance.
        """
        cdef Transform transform = Transform()
        # TODO This leaks memory, and then corrupts memory and crashes, I think.
        transform.thisptr[0] = self.thisptr.getWorldTransform()
        return transform


    def setWorldTransform(self, Transform transform not None):
        """
        Replace the transformation for this CollisionObject.
        """
        self.thisptr.setWorldTransform(transform.thisptr[0])


    def getActivationState(self):
        """
        Return the current activation state of this object.
        """
        return self.thisptr.getActivationState()


    def setActivationState(self, int newState):
        """
        Change the activation state of this object.  newState must be one of:

          - ACTIVE_TAG
          - ISLAND_SLEEPING
          - WANTS_DEACTIVATION
          - DISABLE_DEACTIVATION
          - DISABLE_SIMULATION
        """
        self.thisptr.setActivationState(newState)


cdef class MotionState:
    """
    A MotionState is a primarily an object to provide callback methods to the
    dynamics implementation.

    A MotionState instance can be set on a RigidBody to receive world transform
    updates for that object.  Bullet may call the setWorldTransform callback
    only for objects which move to avoid the cost of doing so where no new
    information will be provided.

    This class is a wrapper around btMotionState.  This wrapping isn't entirely
    sensible, so it may change in a future release.

    XXX THIS WRAPPER MAY CAUSE SEGFAULTS.  Use DefaultMotionState instead.
    """
    cdef btMotionState *thisptr

    def __dealloc__(self):
        del self.thisptr


    def getWorldTransform(self):
        """
        Get a copy of the transformation for this motion state as a Transform
        instance.
        """
        transform = Transform()
        self.thisptr.getWorldTransform(transform.thisptr[0])
        return transform


    def setWorldTransform(self, Transform centerOfMassWorldTrans not None):
        """
        Set the transformation for this motion state.  This will probably have
        no meaningful effect on the state of the world containing the object
        this MotionState is associated with.

        XXX If this method is overridden in a subclass, the overridden method
        will not be called by Bullet.
        """
        self.thisptr.setWorldTransform(centerOfMassWorldTrans.thisptr[0])



cdef class DefaultMotionState(MotionState):
    """
    A DefaultMotionState is a MotionState which keeps track of the last
    transformation provided to it.

    A DefaultMotionState can be used with each RigidBody so that the position
    can be queried after a simulation step.

    This class is a wrapper around btDefaultMotionState.  Like MotionState, this
    wrapping doesn't really make sense and will probably be changed.
    """
    def __cinit__(self):
        self.thisptr = new btDefaultMotionState()



cdef class RigidBody(CollisionObject):
    """
    A RigidBody is an object which can be added to a DynamicsWorld and involved
    in dynamics (ie, it can move because of physics).

    Like a CollisionObject, a RigidBody has a CollisionShape.  Additionally, it
    may also have a MotionState to keep track of its movements.  It also has a
    mass which will influence in the usual way how much acceleration it
    experiences due to forces acting on it.

    This class is a wrapper around btRigidBody.
    """
    cdef MotionState motion
    cdef CollisionShape shape

    def __init__(self,
                 MotionState motion = None,
                 CollisionShape shape = None,
                 btScalar mass = 0.0):
        if motion is None:
            motion = DefaultMotionState()
        if shape is None:
            shape = BoxShape(Vector3(0.5, 0.5, 0.5))

        self.motion = motion
        self.shape = shape

        cdef btVector3 inertia = btVector3(0, 0, 0)
        # TODO This is a weak heuristic to avoid using calculateLocalInertia on
        # a shape that does not support it (and will probably SIGABRT the
        # process).  To be really safe, it will probably be necessary to
        # explicitly list the shapes which cannot have their local inertia
        # calculated.
        if mass != 0.0:
            shape.thisptr.calculateLocalInertia(mass, inertia)

        cdef btRigidBodyConstructionInfo* info
        info = new btRigidBodyConstructionInfo(
            mass, self.motion.thisptr, self.shape.thisptr, inertia)
        self.thisptr = new btRigidBody(info[0])
        del info


    @classmethod
    def fromConstructionInfo(cls, MotionState motion,
                             CollisionShape shape, btScalar mass,
                             Vector3 inertia, Transform worldTransform,
                             btScalar linearDamping, btScalar angularDamping,
                             btScalar friction, btScalar restitution,
                             btScalar linearSleepingThreshold,
                             btScalar angularSleepingThreshold):
        """
        Create a new L{RigidBody} instance, specifying all of its parameters.

        @param motion: A L{MotionState} instance which specifies the body's
            initial world transform and which will receive motion updates for
            the body.  If specified, worldTransform is ignored.

        @param shape: A L{CollisionShape} instance which specifies the body's
            shape for collision detection.

        @param mass: The body's mass.  Setting this to C{0} creates a fixed
            (static; non-dynamic; stationary) body.

        @param worldTransform: A L{Transform} which specifies the body's initial
            position and orientation in the world.  Only used if C{motion} is
            not given.

        @param linearDamping: A value between 0 and 1 which is used to dampen
            linear velocity.  Higher values dampen more.

        @param angularDamping: A value between 0 and 1 which is used to dampen
            angular velocity.  Higher values dampen more.

        @param friction: The friction value this body constributes to friction
            calculates.

        @param restitution: The coefficient of restitution for this body, giving
            the ratio of speed after and before a collision with another object.

        @param linearSleepingThreshold: See L{getLinearSleepingThreshold}.

        @param angularSleepingThreshold: See L{getAngularSleepingThreshold}.
        """
        cdef RigidBody body = cls.__new__(cls)
        cdef btRigidBodyConstructionInfo *info
        cdef btMotionState *motionState

        if motion is not None:
            motionState = motion.thisptr
        else:
            motionState = <btMotionState*>0

        info = new btRigidBodyConstructionInfo(
            mass, motionState, shape.thisptr,
            btVector3(inertia.x, inertia.y, inertia.z))

        info.m_mass = mass
        info.m_startWorldTransform = worldTransform.thisptr[0]
        info.m_linearDamping = linearDamping
        info.m_angularDamping = angularDamping
        info.m_friction = friction
        info.m_restitution = restitution
        info.m_linearSleepingThreshold = linearSleepingThreshold
        info.m_angularSleepingThreshold = angularSleepingThreshold

        body.thisptr = new btRigidBody(info[0])
        body.motion = motion
        body._shape = shape
        del info
        return body


    def isInWorld(self):
        """
        Return a boolean indicating whether or not this RigidBody has been added
        to a CollisionWorld.
        """
        cdef btRigidBody *body
        body = <btRigidBody*>self.thisptr
        return body.isInWorld()


    def getInvMass(self):
        """
        Return the inverse of the mass of this L{RigidBody}.
        """
        cdef btRigidBody *body
        body = <btRigidBody*>self.thisptr
        return body.getInvMass()


    def getInvInertiaDiagLocal(self):
        """
        Return the inverse of the local inertia vector.
        """
        cdef btRigidBody *body
        body = <btRigidBody*>self.thisptr
        cdef btVector3 inertia = body.getInvInertiaDiagLocal()
        return Vector3(inertia.getX(), inertia.getY(), inertia.getZ())


    def getMotionState(self):
        """
        Return the MotionState associated with this RigidBody.
        """
        return self.motion


    def setAngularFactor(self, btScalar angularFactor):
        """
        Specify whether this object will be allowed to rotate or not.  If the
        given angularFactor argument is 0, rotation will not be allowed.  If it
        is 1, rotation will be allowed.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        body.setAngularFactor(angularFactor)


    def setLinearVelocity(self, Vector3 v not None):
        """
        Change the linear velocity of this RigidBody for at least a single
        simulation step.  It is unspecified whether the change will persist for
        more than one simulation step.  For reliable and reproducable results,
        you must set the linear velocity before each simulation tick.  This is
        best done in the physics tick callback.

        XXX The physics tick callback is presently unexposed.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        cdef btVector3 vel = btVector3(v.x, v.y, v.z)
        body.setLinearVelocity(vel)


    def getLinearVelocity(self):
        """
        Retrieve the current linear velocity of this RigidBody as a Vector3.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        cdef btVector3 vel = body.getLinearVelocity()
        return Vector3(vel.getX(), vel.getY(), vel.getZ())


    def getLinearDamping(self):
        """
        Return the linear velocity damping value for this RigidBody.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        return body.getLinearDamping()


    def getLinearSleepingThreshold(self):
        """
        Return the linear velocity threshold below which the body will not be
        deactivated.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        return body.getLinearSleepingThreshold()


    def getAngularSleepingThreshold(self):
        """
        Return the linear velocity threshold below which the body will not be
        deactivated.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        return body.getAngularSleepingThreshold()


    def getAngularDamping(self):
        """
        Return the angular velocity damping value for this RigidBody.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        return body.getAngularDamping()


    def applyCentralForce(self, Vector3 f not None):
        """
        Apply a force to the center of mass of this RigidBody.  The resulting
        acceleration is computed in the usual way, dividing the force by the
        mass of the RigidBody.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        cdef btVector3 force = btVector3(f.x, f.y, f.z)
        body.applyCentralForce(force)


    def applyForce(self, Vector3 f not None, Vector3 relativePosition not None):
        """
        Apply a force to this RigidBody at a specified offset from its center of
        mass.  The resulting acceleration is computed in the usual way, dividing
        the force by the mass of the RigidBody.  A force applied off-center may
        also result in rotation.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        cdef btVector3 force = btVector3(f.x, f.y, f.z)
        cdef btVector3 pos = btVector3(
            relativePosition.x, relativePosition.y, relativePosition.z)
        body.applyForce(force, pos)


    def applyCentralImpulse(self, Vector3 i not None):
        """
        Apply an impulse to this RigidBody.  The resulting acceleration will be
        equal to the impulse.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        cdef btVector3 impulse = btVector3(i.x, i.y, i.z)
        body.applyCentralImpulse(impulse)


    def applyImpulse(self, Vector3 i not None, Vector3 relativePosition not None):
        """
        Apply an impulse to this RigidBody at a specified offset from its center
        of mass.  The resulting accleration will be equal to the impulse.  An
        impulse applied off-center may also result in rotation.
        """
        cdef btRigidBody* body = <btRigidBody*>self.thisptr
        cdef btVector3 impulse = btVector3(i.x, i.y, i.z)
        cdef btVector3 pos = btVector3(
            relativePosition.x, relativePosition.y, relativePosition.z)
        body.applyImpulse(impulse, pos)
    
    def applyTorque(self, Vector3 t not None):
        """
        Apply torque to this RigidBody.
        """
        (<btRigidBody*>self.thisptr).applyTorque(btVector3(t.x, t.y, t.z))


    def applyTorqueImpulse(self, Vector3 ti not None):
        """
        Apply a torque impulse to this RigidBody.
        """
        (<btRigidBody*>self.thisptr).applyTorqueImpulse(btVector3(ti.x, ti.y, ti.z))


    def setCenterOfMassTransform(self, Transform trans not None):
        """
        Set the transform that describes the center of mass of this body.
        """
        (<btRigidBody*>self.thisptr).setCenterOfMassTransform(trans.thisptr[0])


    def getCenterOfMassTransform(self):
        """
        Returns a Transform that describes the center of mass of this body.
        """
        cdef Transform transform = Transform()
        transform.thisptr[0] = (<btRigidBody*>self.thisptr).getCenterOfMassTransform()
        return transform    


    def setAngularVelocity(self, Vector3 velocity not None):
        """
        Change the angular velocity of this RigidBody for at least a single
        simulation step.  It is unspecified whether the change will persist for
        more than one simulation step.  For reliable and reproducable results,
        you must set this before each simulation tick.
        """
        (<btRigidBody*>self.thisptr).setAngularVelocity(
            btVector3(velocity.x, velocity.y, velocity.z))
    
    
    def getAngularVelocity(self):
        """
        Returns the instantaneous angular velocity of this body as a Vector3.
        """
        cdef btVector3 velocity = (<btRigidBody*>self.thisptr).getAngularVelocity()
        return Vector3(velocity.getX(), velocity.getY(), velocity.getZ())

    
    def getOrientation(self):
        """
        Returns the orientation of this object (world-space).
        """
        cdef Quaternion orientation = Quaternion()
        orientation.quaternion[0] = (<btRigidBody*>self.thisptr).getOrientation()
        return orientation



cdef class ActionInterface:
    """
    ActionInterface is a base class for objects in a DynamicsWorld which are not
    purely dynamic.

    This class is a wrapper around btActionInterface.

    XXX THIS WRAPPER MAY CAUSE SEGFAULTS.  Use a subclass instead.
    """
    cdef btActionInterface *thisptr

    def __dealloc__(self):
        del self.thisptr



cdef class CharacterControllerInterface(ActionInterface):
    """
    A CharacterControllerInterface is an ActionInterface which allows walking
    motion to be applied to an object in a DynamicsWorld without physically
    simulating that walking.

    This class is a wrapper around btCharacterControllerInterface.

    XXX THIS WRAPPER MAY CAUSE SEGFAULTS.  Use a subclass instead.
    """
    def setWalkDirection(self, Vector3 walkDirection):
        """
        Set the movement direction for this CharacterControllerInterface.  The
        magnitude and the direction of the given walkDirection will determine
        how this object moves in the world.

        Calling setWalkDirection will cause a previous call to
        setVelocityForTimeInterval to be disregarded.
        """
        cdef btCharacterControllerInterface *controller
        controller = <btCharacterControllerInterface*>self.thisptr
        controller.setWalkDirection(
            btVector3(walkDirection.x, walkDirection.y, walkDirection.z))


    def setVelocityForTimeInterval(self,
                                   Vector3 velocity not None,
                                   btScalar timeInterval):
        """
        Set the movement direction for this CharacterControllerInterface.  The
        direction of the given velocity will determine the direction of this
        object's movement in the world, while the timeInterval will determine
        the rate of movement.

        Calling setVelocityForTimeInterval will cause a previous call to
        setWalkDirection to be disregarded.
        """
        cdef btCharacterControllerInterface *controller
        controller = <btCharacterControllerInterface*>self.thisptr
        controller.setVelocityForTimeInterval(
            btVector3(velocity.x, velocity.y, velocity.z), timeInterval)



cdef class PairCachingGhostObject(CollisionObject):
     """
     A PairCachingGhostObject is a CollisionObject which keeps track of all the
     objects it is overlapping.

     This class is a wrapper around btPairCachingGhostObject.
     """
     def __init__(self):
         self.thisptr = new btPairCachingGhostObject()

cdef class CompoundShape(CollisionShape):
    """
    A compound shape ties a number of transformed collision shapes together.
    """

    def __init__(self, bool enableDynamicAabbTree=True):
        self.thisptr = new btCompoundShape(enableDynamicAabbTree)

    def addChildShape(self, Transform localTransform not None, CollisionShape shape):    
        cdef btCompoundShape* cs = <btCompoundShape*>self.thisptr
        cs.addChildShape(localTransform.thisptr[0], shape.thisptr)
        
    def removeChildShape(self, CollisionShape shape):
        cdef btCompoundShape* cs = <btCompoundShape*>self.thisptr
        cs.removeChildShape(shape.thisptr)

cdef class KinematicCharacterController(CharacterControllerInterface):
    """
    A KinematicCharacterController is a CharacterControllerInterface which
    implements movement according to the standard rules of kinematics.  That is,
    it will move based on a velocity and direction in the usual way, but without
    regard for interactions with other objects in the world.

    This class is a wrapper around btKinematicCharacterController.
    """
    cdef readonly PairCachingGhostObject ghost
    cdef ConvexShape shape

    def __init__(self, ConvexShape shape not None, float stepHeight, int upAxis):
        self.shape = shape
        self.ghost = PairCachingGhostObject()
        self.thisptr = new btKinematicCharacterController(
            <btPairCachingGhostObject*>self.ghost.thisptr,
            <btConvexShape*>self.shape.thisptr, stepHeight, upAxis)


    def warp(self, Vector3 origin not None):
        """
        Change the location of this object to the given point without traversing
        the intermediate distance.
        """
        cdef btKinematicCharacterController *controller
        controller = <btKinematicCharacterController*>self.thisptr
        controller.warp(btVector3(origin.x, origin.y, origin.z))



cdef class CollisionDispatcher:
    """
    A CollisionDispatcher implements pairwise collision detection.  It supports
    convex-convex collision detection and convex-concave collision detection.

    Roughly speaking, this is the only CollisionDispatcher available, so you
    don't have to understand it, just use it.  Or better yet, ignore it and let
    CollisionWorld (or a subclass) create it for you.

    This class is a wrapper around btCollisionDispatcher.

    XXX Try not to instantiate too many of these, each one leaks some memory.
    """
    cdef btCollisionConfiguration *config
    cdef btDispatcher *thisptr

    def __cinit__(self):
        # XXX btDefaultCollisionConfiguration leaks I suppose.
        self.config = new btDefaultCollisionConfiguration()
        self.thisptr = new btCollisionDispatcher(self.config)


    def __dealloc__(self):
        del self.thisptr
        del self.config



cdef class OverlappingPairCache:
    """
    An OverlappingPairCache manages the addition, removal, and storage of
    overlapping pairs.

    This class is a wrapper around btOverlappingPairCache.

    XXX This wrapper will cause segfaults.  Use one of the subclasses instead.
    """
    cdef btOverlappingPairCache *thisptr

    def __dealloc__(self):
        del self.thisptr


    def setInternalGhostPairCallback(self):
        """
        Set the internal ghost pair callback to a new btGhostPairCallback.

        The corresponding C++ API accepts the callback as a parameter.  This API
        can do that when I learn why you would ever pass something other than a
        new btGhostPairCallback.
        """
        self.thisptr.setInternalGhostPairCallback(new btGhostPairCallback())



cdef class HashedOverlappingPairCache(OverlappingPairCache):
    """
    A HashedOverlappingPairCache manages the addition, removal, and storage of
    overlapping pairs using a hash table.

    This class is a wrapper around btHashedOverlappingPairCache.
    """
    def __cinit__(self):
        self.thisptr = new btHashedOverlappingPairCache()



cdef class BroadphaseInterface:
    """
    A BroadphaseInterface generates lists of potentially colliding pairs.  It is
    intended to be faster than CollisionDispatcher, but it may include pairs of
    objects which do not actually collide.  It is applied first to narrow the
    list of pairs the CollisionDispatcher will need to examine.

    This class is loosely a wrapper around btBroadphaseInterface.

    XXX THIS WRAPPER MAY CAUSE SEGFAULTS.  Use one of the subclasses instead.
    """
    cdef btBroadphaseInterface *thisptr
    cdef readonly OverlappingPairCache _paircache

    def __dealloc__(self):
        del self.thisptr


    def getOverlappingPairCache(self):
        """
        Return the OverlappingPairCache used by this BroadphaseInterface.
        """
        # Subclasses must take care to set this during their initialization.
        return self._paircache



cdef class AxisSweep3(BroadphaseInterface):
    """
    An AxisSweep3 is a BroadphaseInterface implemented using the sweep and prune
    algorithm.  AxisSweep3 is a good general purpose BroadphaseInterface,
    particularly for worlds where most objects have little or no motion.
    However, it can only detect collisions within specified fixed bounds.

    This class is a wrapper around btAxisSweep3.
    """
    def __cinit__(self, Vector3 lower, Vector3 upper):
        self._paircache = HashedOverlappingPairCache()
        self.thisptr = new btAxisSweep3(
            btVector3(lower.x, lower.y, lower.z),
            btVector3(upper.x, upper.y, upper.z),
            16384, self._paircache.thisptr, False)



cdef class ConstraintSolver:
    """
    A ConstraintSolver determines what contact forces to apply.

    This class is loosely a wrapper around btConstraintSolver.

    XXX THIS WRAPPER MAY CAUSE SEGFAULTS.  Use one of the subclasses instead.
    """
    cdef btConstraintSolver *thisptr

    def __dealloc__(self):
        del self.thisptr



cdef class SequentialImpulseConstraintSolver(ConstraintSolver):
    """
    A SequentialImpulseConstraintSolver is a ConstraintSolver based on a fast
    SIMD implementation of the Projected Gauss Seidel method.

    This is basically the only ConstraintSolver available, so whether you
    understood that sentence or not, use this class with your DynamicsWorld.  Or
    just ignore ConstraintSolvers entirely and let the DynamicsWorld create one.

    This class is a wrapper around btSequentialImpulseConstraintSolver.
    """
    def __cinit__(self):
        self.thisptr = new btSequentialImpulseConstraintSolver()



cdef class CollisionWorld:
    """
    A CollisionWorld is a container for CollisionObjects which can detect
    collisions between those objects.

    This class is a wrapper around btCollisionWorld.
    """
    cdef btCollisionWorld *thisptr
    cdef PythonDebugDraw *debugDraw

    cdef _object *dispatcher
    cdef _object *broadphase

    def __init__(self,
                 CollisionDispatcher dispatcher = None,
                 BroadphaseInterface broadphase = None):
        if dispatcher is None:
            dispatcher = CollisionDispatcher()
        if broadphase is None:
            broadphase = AxisSweep3(Vector3(0, 0, 0), Vector3(10, 10, 10))

        Py_INCREF(dispatcher)
        self.dispatcher = <_object*>dispatcher

        Py_INCREF(broadphase)
        self.broadphase = <_object*>broadphase

        # Allow subclasses to initialize this differently.
        if self.thisptr == NULL:
            self.thisptr = new btCollisionWorld(
                dispatcher.thisptr, broadphase.thisptr, dispatcher.config)


    def __dealloc__(self):
        del self.thisptr
        Py_DECREF(<object>self.dispatcher)
        Py_DECREF(<object>self.broadphase)


    def setDebugDrawer(self, debugDrawer):
        """
        Specify a debug drawer object to use for L{debugDrawWorld} calls.

        The debug drawer must have all of the required drawing callback methods.
        TODO Document them.  See the debugdraw.py demo for now.
        """
        self.debugDraw = new PythonDebugDraw(<PyObject*>debugDrawer);
        self.thisptr.setDebugDrawer(self.debugDraw)


    def debugDrawWorld(self):
        """
        Draw the current state of the world using the debug drawer provided by a
        previous call to L{setDebugDrawer}.
        """
        self.thisptr.debugDrawWorld()


    def getNumCollisionObjects(self):
        """
        Return a count of the number of CollisionObjects which are part of this
        CollisionWorld.
        """
        return self.thisptr.getNumCollisionObjects()


    def addCollisionObject(self, CollisionObject collisionObject):
        """
        Add a new CollisionObject to this CollisionWorld.

        If you have a RigidBody to add, you should add it using
        DynamicsWorld.addRigidBody instead.
        """
        if collisionObject.thisptr.getCollisionShape() == NULL:
            raise ValueError(
                "Cannot add CollisionObject without a CollisionShape")
        self.thisptr.addCollisionObject(collisionObject.thisptr, 0, 0)


    def removeCollisionObject(self, CollisionObject collisionObject):
        """
        Remove a CollisionObject from this CollisionWorld.
        """
        self.thisptr.removeCollisionObject(collisionObject.thisptr)

    def getDispatcher(self):
        """
        Returns the CollisionDispatcher that this world uses.
        """
        return <CollisionDispatcher>self.dispatcher



cdef class DynamicsWorld(CollisionWorld):
    """
    A DynamicsWorld is a container for RigidBodies which implements dynamics (ie
    physics) for those bodies.

    For a dynamics world in which simulation time can actually pass, see one of
    the subclasses of this class.

    This class is a wrapper around btDynamicsWorld.
    """
    cdef list _rigidBodies
    cdef list _constraints

    def __init__(self,
                 CollisionDispatcher dispatcher = None,
                 BroadphaseInterface broadphase = None):
        CollisionWorld.__init__(self, dispatcher, broadphase)
        self._rigidBodies = []
        self._constraints = []

    def addConstraint(self, TypedConstraint constraint, bool disableCollisionsBetweenLinkedBodies=False):
        """
        Add a new constraint to this DynamicsWorld.
        """
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        world.addConstraint(constraint.thisptr, disableCollisionsBetweenLinkedBodies)
        self._constraints.append(constraint)


    def removeConstraint(self, TypedConstraint constraint):
        """
        Remove a constraint from this DynamicsWorld.
        """
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        self._constraints.remove(constraint)
        world.removeConstraint(constraint.thisptr)


    def addRigidBody(self, RigidBody body not None):
        """
        Add a new RigidBody to this DynamicsWorld.
        """
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        world.addRigidBody(<btRigidBody*>body.thisptr)
        self._rigidBodies.append(body)


    def removeRigidBody(self, RigidBody body not None):
        """
        Remove a RigidBody from this DynamicsWorld.
        """
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        self._rigidBodies.remove(body)
        world.removeRigidBody(<btRigidBody*>body.thisptr)


    def addAction(self, ActionInterface action not None):
        """
        Add a new ActionInterface to this DynamicsWorld.
        """
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        world.addAction(<btActionInterface*>action.thisptr)
        self._rigidBodies.append(action)


    def removeAction(self, ActionInterface action not None):
        """
        Remove an ActionInterface which was previously added to this
        DynamicsWorld.
        """
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        world.removeAction(<btActionInterface*>action.thisptr)
        self._rigidBodies.append(action)



cdef class DiscreteDynamicsWorld(DynamicsWorld):
    """
    A DiscreteDynamicsWorld is a DynamicsWorld in which time passes in fixed
    increments.

    This class is a wrapper around btDiscreteDynamicsWorld.
    """
    cdef ConstraintSolver solver

    def __init__(self,
                 CollisionDispatcher dispatcher = None,
                 BroadphaseInterface broadphase = None,
                 ConstraintSolver solver = None):

        if dispatcher is None:
            dispatcher = CollisionDispatcher()
        if solver is None:
            solver = SequentialImpulseConstraintSolver()
        if broadphase is None:
            broadphase = AxisSweep3(Vector3(-100, -100, -100), Vector3(100, 100, 100)) #this was fixed in trunk rev 32

        self.solver = solver
        self.thisptr = <btCollisionWorld*>new btDiscreteDynamicsWorld(
            dispatcher.thisptr, broadphase.thisptr,
            solver.thisptr, dispatcher.config)

        DynamicsWorld.__init__(self, dispatcher, broadphase)


    def setGravity(self, Vector3 gravity):
        """
        Set the gravity in this world.
        XXX This belongs on DynamicsWorld.
        """
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        world.setGravity(btVector3(gravity.x, gravity.y, gravity.z))


    def getGravity(self):
        """
        Get the gravity in this world.
        XXX This belongs on DynamicsWorld.
        """
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        cdef btVector3 gravity = world.getGravity()
        return Vector3(gravity.getX(), gravity.getY(), gravity.getZ())


    def stepSimulation(self,
                       btScalar timeStep,
                       int maxSubSteps = 1,
                       btScalar fixedTimeStep = 1. / 60.):
        """
        Advance time in the simulation.

        timeStep specifies the amount of simulation time which will pass during
        this call.

        maxSubSteps specifies the maximum number of simulation steps which will
        be taken trying to reach the point in the future specified by timeStep.

        fixedTimeStep specifies the size of each simulation step.  Bullet works
        best if this value is always the same.

        Observe that if fixedTimeStep * maxSubSteps < timeStep, simulation time
        is lost and the simulation will not advance as far as you intended it
        to.

        The number of simulation steps taken is returned.
        """
        cdef btDynamicsWorld *world = <btDynamicsWorld*>self.thisptr
        return world.stepSimulation(timeStep, maxSubSteps, fixedTimeStep)
#OLD ENF OF FILE
cdef class VehicleRaycasterResult:
    """ An internal class used by the raycast vehicle. """
    
    cdef btVehicleRaycasterResult *thisptr
    
    def __init__(self):
        self.thisptr = new btVehicleRaycasterResult()
        
    def __dealloc__(self):
        del self.thisptr

    property hitPointInWorld:
        def __get__(self):
            return Vector3(self.thisptr.m_hitPointInWorld.getX(),
                           self.thisptr.m_hitPointInWorld.getY(), 
                           self.thisptr.m_hitPointInWorld.getZ())                               
    
    property hitNormalInWorld:
        def __get__(self):
            return Vector3(self.thisptr.m_hitNormalInWorld.getX(),
                           self.thisptr.m_hitNormalInWorld.getY(), 
                           self.thisptr.m_hitNormalInWorld.getZ())   
            
    property distFraction:
        def __get__(self):
            return self.thisptr.m_distFraction



cdef class VehicleRaycaster:
    """ A vehicle raycaster simply casts rays down from a vehicle
        to stop it from colliding with other world bodies.
        Used by the RaycastVehicle. """

    cdef btVehicleRaycaster *thisptr

    def __dealloc__(self):
        del self.thisptr



cdef class DefaultVehicleRaycaster(VehicleRaycaster):
    """ Concrete class defining the Vehicle Raycaster. """

    cdef DynamicsWorld world

    def __cinit__(self, DynamicsWorld world):
        self.thisptr = new btDefaultVehicleRaycaster((<btDynamicsWorld*>world.thisptr))
        self.world = world

    def castRay(self, fromVector, toVector):
        """ Casts a ray and returns a VehicleRaycaster.Result object. """
        cdef btVector3 _fromVector = btVector3(fromVector.x, fromVector.y, fromVector.z)
        cdef btVector3 _toVector = btVector3(toVector.x, toVector.y, toVector.z)        
        cdef VehicleRaycasterResult result = VehicleRaycasterResult()
        (<btDefaultVehicleRaycaster*>self.thisptr).castRay(_fromVector, _toVector, result.thisptr[0])
        return result
        
        

cdef class RaycastInfo:
    """ An internal class used by the raycast vehicle. """
    
    cdef btRaycastInfo *thisptr
    
    def __cinit__(self):
        self.thisptr = new btRaycastInfo()
        
    def __dealloc__(self):
        del self.thisptr
        
    property contactNormalWS:
        def __get__(self):
            return Vector3(self.thisptr.m_contactNormalWS.getX(),
                           self.thisptr.m_contactNormalWS.getY(), 
                           self.thisptr.m_contactNormalWS.getZ()) 
            
    property contactPointWS:
        def __get__(self):
            return Vector3(self.thisptr.m_contactPointWS.getX(),
                           self.thisptr.m_contactPointWS.getY(), 
                           self.thisptr.m_contactPointWS.getZ()) 
                             
    property hardPointWS:
        def __get__(self):
            return Vector3(self.thisptr.m_hardPointWS.getX(),
                           self.thisptr.m_hardPointWS.getY(), 
                           self.thisptr.m_hardPointWS.getZ()) 
                             
    property wheelDirectionWS:
        def __get__(self):
            return Vector3(self.thisptr.m_wheelDirectionWS.getX(),
                           self.thisptr.m_wheelDirectionWS.getY(), 
                           self.thisptr.m_wheelDirectionWS.getZ()) 
                               
    property wheelAxleWS:
        def __get__(self):
            return Vector3(self.thisptr.m_wheelAxleWS.getX(),
                           self.thisptr.m_wheelAxleWS.getY(), 
                           self.thisptr.m_wheelAxleWS.getZ()) 
                                       
    property suspensionLength:
        def __get__(self):
            return self.thisptr.m_suspensionLength

    property isInContact:
        def __get__(self):
            return self.thisptr.m_isInContact                               

    # TODO void * m_groundObject



cdef class WheelInfo:
    """
    Describes a wheel on a car. Used in the RaycastVehicle, and perhaps elsewhere.
    
    Note that the C++ class WheelInfoConstructionInfo has been folded into
    this one as keyword arguments into the initialiser. Any members of that class
    can be passed into it, for example::
    
        wheelInfo = WheelInfo(maxSuspensionTravelCm=10.0, wheelRadius=25.0, wheelDirectionCS=Vector3(1, 0, 0))
        
    As the C++ structure WheelInfoConstructionInfo provides no default values,
    neither does pyBullet. It is recommended that you inherit from WheelInfo 
    to provide the defaults that you need.
    """

    cdef btWheelInfo *thisptr
    
    def __cinit__(self, chassisConnectionCS=Vector3(0,0,0), wheelDirectionCS=Vector3(0,0,0), wheelAxleCS=Vector3(0,0,0),
                        suspensionRestLength=1.0, maxSuspensionTravelCm=1.0, wheelRadius=1.0,
                        suspensionStiffness=1.0, wheelsDampingCompression=1.0, wheelsDampingRelaxation=1.0,
                        frictionSlip=0.0, maxSuspensionForce=0.0, bIsFrontWheel=False):
                       
        cdef btWheelInfoConstructionInfo* cons = new btWheelInfoConstructionInfo()
        cons.m_chassisConnectionCS = btVector3(chassisConnectionCS.x, chassisConnectionCS.y, chassisConnectionCS.z)
        cons.m_wheelDirectionCS = btVector3(wheelDirectionCS.x, wheelDirectionCS.y, wheelDirectionCS.z)
        cons.m_wheelAxleCS = btVector3(wheelAxleCS.x, wheelAxleCS.y, wheelAxleCS.z)
        cons.m_suspensionRestLength = suspensionRestLength
        cons.m_maxSuspensionTravelCm = maxSuspensionTravelCm
        cons.m_wheelRadius = wheelRadius
        cons.m_suspensionStiffness = suspensionStiffness
        cons.m_wheelsDampingCompression = wheelsDampingCompression
        cons.m_wheelsDampingRelaxation = wheelsDampingRelaxation
        cons.m_frictionSlip = frictionSlip
        cons.m_maxSuspensionForce = maxSuspensionForce
        cons.m_bIsFrontWheel = bIsFrontWheel
        
        self.thisptr = new btWheelInfo(cons[0])
        del cons
        
        
    def getSuspensionRestLength(self):
        """ Get the length of the wheel's suspension at rest. """
        return self.thisptr.getSuspensionRestLength()
    
    def updateWheel(self, RigidBody chassis, RaycastInfo raycastInfo):
        """ """
        self.thisptr.updateWheel((<btRigidBody*>chassis.thisptr)[0], raycastInfo.thisptr[0])
        
    property raycastInfo:
        """  """
        def __get__(self):
            cdef RaycastInfo raycastInfo = RaycastInfo()
            raycastInfo.thisptr[0] = self.thisptr.m_raycastInfo
            return raycastInfo
            
    property worldTransform:
        def __get__(self):
            cdef Transform worldTransform = Transform()
            worldTransform.thisptr[0] = self.thisptr.m_worldTransform
            return worldTransform

    property chassisConnectionPointCS:
        def __get__(self):
            return Vector3(self.thisptr.m_chassisConnectionPointCS.getX(),
                           self.thisptr.m_chassisConnectionPointCS.getY(), 
                           self.thisptr.m_chassisConnectionPointCS.getZ()) 
        def __set__(self, v):
            self.thisptr.m_chassisConnectionPointCS = btVector3(v.x, v.y, v.z)
            
    property wheelDirectionCS:
        def __get__(self):
            return Vector3(self.thisptr.m_wheelDirectionCS.getX(),
                           self.thisptr.m_wheelDirectionCS.getY(), 
                           self.thisptr.m_wheelDirectionCS.getZ()) 
        def __set__(self, v):
            self.thisptr.m_wheelDirectionCS = btVector3(v.x, v.y, v.z)
                             
    property wheelAxleCS:
        def __get__(self):
            return Vector3(self.thisptr.m_wheelAxleCS.getX(),
                           self.thisptr.m_wheelAxleCS.getY(), 
                           self.thisptr.m_wheelAxleCS.getZ()) 
        def __set__(self, v):
            self.thisptr.m_wheelAxleCS = btVector3(v.x, v.y, v.z)                         
                           
    property suspensionRestLength1:
        def __get__(self):
            return self.thisptr.m_suspensionRestLength1
        def __set__(self, v):
            self.thisptr.m_suspensionRestLength1 = v

    property maxSuspensionTravelCm:
        def __get__(self):
            return self.thisptr.m_maxSuspensionTravelCm
        def __set__(self, v):
            self.thisptr.m_maxSuspensionTravelCm = v

    property wheelsRadius:
        def __get__(self):
            return self.thisptr.m_wheelsRadius
        def __set__(self, v):
            self.thisptr.m_wheelsRadius = v

    property suspensionStiffness:
        def __get__(self):
            return self.thisptr.m_suspensionStiffness
        def __set__(self, v):
            self.thisptr.m_suspensionStiffness = v

    property wheelsDampingCompression:
        def __get__(self):
            return self.thisptr.m_wheelsDampingCompression
        def __set__(self, v):
            self.thisptr.m_wheelsDampingCompression = v

    property wheelsDampingRelaxation:
        def __get__(self):
            return self.thisptr.m_wheelsDampingRelaxation
        def __set__(self, v):
            self.thisptr.m_wheelsDampingRelaxation = v

    property frictionSlip:
        def __get__(self):
            return self.thisptr.m_frictionSlip
        def __set__(self, v):
            self.thisptr.m_frictionSlip = v

    property steering:
        def __get__(self):
            return self.thisptr.m_steering
        def __set__(self, v):
            self.thisptr.m_steering = v

    property rotation:
        def __get__(self):
            return self.thisptr.m_rotation
        def __set__(self, v):
            self.thisptr.m_rotation = v

    property deltaRotation:
        def __get__(self):
            return self.thisptr.m_deltaRotation
        def __set__(self, v):
            self.thisptr.m_deltaRotation = v

    property rollInfluence:
        def __get__(self):
            return self.thisptr.m_rollInfluence
        def __set__(self, v):
            self.thisptr.m_rollInfluence = v

    property maxSuspensionForce:
        def __get__(self):
            return self.thisptr.m_maxSuspensionForce
        def __set__(self, v):
            self.thisptr.m_maxSuspensionForce = v
            
    property engineForce:
        def __get__(self):
            return self.thisptr.m_engineForce
        def __set__(self, v):
            self.thisptr.m_engineForce = v
            
    property brake:
        def __get__(self):
            return self.thisptr.m_brake
        def __set__(self, v):
            self.thisptr.m_brake = v
            
    property bIsFrontWheel:
        def __get__(self):
            return self.thisptr.m_bIsFrontWheel
        def __set__(self, v):
            self.thisptr.m_bIsFrontWheel = v            
            
    property clippedInvContactDotSuspension:
        def __get__(self):
            return self.thisptr.m_clippedInvContactDotSuspension
        def __set__(self, v):
            self.thisptr.m_clippedInvContactDotSuspension = v            
            
    property suspensionRelativeVelocity:
        def __get__(self):
            return self.thisptr.m_suspensionRelativeVelocity            
        def __set__(self, v):
            self.thisptr.m_suspensionRelativeVelocity = v            
            
    property wheelsSuspensionForce:
        def __get__(self):
            return self.thisptr.m_wheelsSuspensionForce
        def __set__(self, v):
            self.thisptr.m_wheelsSuspensionForce = v            
            
    property skidInfo:
        def __get__(self):
            return self.thisptr.m_skidInfo            
        def __set__(self, v):
            self.thisptr.m_skidInfo = v                     
                           
    # TODO this property void * m_clientInfo



cdef class VehicleTuning:
    """
    Tuning parameters for the RaycastVehicle.
    """

    cdef btVehicleTuning *thisptr

    def __cinit__(self):
        self.thisptr = new btVehicleTuning()

    def __dealloc__(self):
        del self.thisptr

    property suspensionStiffness:
        def __get__(self):
            return self.thisptr.m_suspensionStiffness
        def __set__(self, value):
            self.thisptr.m_suspensionStiffness = value    

    property suspensionCompression:
        def __get__(self):
            return self.thisptr.m_suspensionCompression
        def __set__(self, value):
            self.thisptr.m_suspensionCompression = value    
        
    property suspensionDamping:
        def __get__(self):
            return self.thisptr.m_suspensionDamping
        def __set__(self, value):
            self.thisptr.m_suspensionDamping = value    
        
    property maxSuspensionTravelCm:
        def __get__(self):
            return self.thisptr.m_maxSuspensionTravelCm
        def __set__(self, value):
            self.thisptr.m_maxSuspensionTravelCm = value    

    property frictionSlip:
        def __get__(self):
            return self.thisptr.m_frictionSlip
        def __set__(self, value):
            self.thisptr.m_frictionSlip = value    

    property maxSuspensionForce:
        def __get__(self):
            return self.thisptr.m_maxSuspensionForce
        def __set__(self, value):
            self.thisptr.m_maxSuspensionForce = value    
                  
                  
                  
cdef class RaycastVehicle(ActionInterface):
    """
    The raycast vehicle is a simple n-wheeled vehicle that casts rays down from
    wheel connection points to collide with other world bodies. As such, it is
    not a physically-accurate simulation, but rather a simpler one that allows
    for a more arcade-style feel.
    
    See demos/vehicledemo.py for a simple example of how to use this class.
    """

    cdef RigidBody chassis
                      
    def __cinit__(self, VehicleTuning tuning, RigidBody chassis, VehicleRaycaster raycaster):
        self.thisptr = new btRaycastVehicle(tuning.thisptr[0], (<btRigidBody*>chassis.thisptr), raycaster.thisptr)        
        self.chassis = chassis
                      
    def rayCast(self, WheelInfo wheelInfo):
        return (<btRaycastVehicle*>self.thisptr).rayCast(wheelInfo.thisptr[0])

    def resetSuspension(self):
        (<btRaycastVehicle*>self.thisptr).resetSuspension()
        
    # XXX this function, apparently, does not even exist.
    # http://www.ogre3d.org/addonforums/viewtopic.php?f=3&t=11787
    #def setRaycastWheelInfo(self, wheelIndex, isInContact, hitPoint, hitNormal, depth):
    #    cdef btVector3 _hitPoint = btVector3(hitPoint.x, hitPoint.y, hitPoint.z)
    #    cdef btVector3 _hitNormal = btVector3(hitNormal.x, hitNormal.y, hitNormal.z)        
    #    (<btRaycastVehicle*>self.thisptr).setRaycastWheelInfo(wheelIndex, isInContact, _hitPoint, _hitNormal, depth)
    
    def getWheelInfo(self, wheelIndex):
        """ Returns the WheelInfo structure for the given wheel. """
        cdef WheelInfo wheelInfo = WheelInfo()
        wheelInfo.thisptr[0] = (<btRaycastVehicle*>self.thisptr).getWheelInfo(wheelIndex)
        return wheelInfo

    def getNumWheels(self):
        return (<btRaycastVehicle*>self.thisptr).getNumWheels()    

    def addWheel(self, connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, VehicleTuning tuning, isFrontWheel):
        cdef btVector3 _connectionPointCS0 = btVector3(connectionPointCS0.x, connectionPointCS0.y, connectionPointCS0.z)
        cdef btVector3 _wheelDirectionCS0 = btVector3(wheelDirectionCS0.x, wheelDirectionCS0.y, wheelDirectionCS0.z)      
        cdef btVector3 _wheelAxleCS = btVector3(wheelAxleCS.x, wheelAxleCS.y, wheelAxleCS.z)  
        
        cdef WheelInfo wheelInfo = WheelInfo()
        wheelInfo.thisptr[0] = (<btRaycastVehicle*>self.thisptr).addWheel(_connectionPointCS0, _wheelDirectionCS0, _wheelAxleCS, suspensionRestLength, wheelRadius, tuning.thisptr[0], isFrontWheel)
        return wheelInfo

    def getSteeringValue(self, wheelIndex):
        return (<btRaycastVehicle*>self.thisptr).getSteeringValue(wheelIndex)

    def setSteeringValue(self, steering, wheelIndex):
        (<btRaycastVehicle*>self.thisptr).setSteeringValue(steering, wheelIndex)

    def applyEngineForce(self, force, wheelIndex):
        (<btRaycastVehicle*>self.thisptr).applyEngineForce(force, wheelIndex)

    def setBrake(self, brake, wheelIndex):
        (<btRaycastVehicle*>self.thisptr).setBrake(brake, wheelIndex)

    def setPitchControl(self, pitch):
        (<btRaycastVehicle*>self.thisptr).setPitchControl(pitch)

    def getWheelTransformWS(self, wheelIndex):
        """ Returns a Transform describing the wheel's transformation in world space. """
        cdef Transform t = Transform()
        t.thisptr[0] = (<btRaycastVehicle*>self.thisptr).getWheelTransformWS(wheelIndex)
        return t

    def getChassisWorldTransform(self): 
        """ Returns a Transform describing the chassis' transformation. """
        cdef Transform t = Transform()
        t.thisptr[0] = (<btRaycastVehicle*>self.thisptr).getChassisWorldTransform()
        return t 
                
    def updateWheelTransform(self, wheelIndex, interpolatedTransform=True):
        (<btRaycastVehicle*>self.thisptr).updateWheelTransform(wheelIndex, interpolatedTransform)

    def updateWheelTransformsWS(self, WheelInfo wheel, interpolatedTransform=True):
        (<btRaycastVehicle*>self.thisptr).updateWheelTransformsWS(wheel.thisptr[0], interpolatedTransform)
    
    def updateVehicle(self, step):
        (<btRaycastVehicle*>self.thisptr).updateVehicle(step)
    
    def updateSuspension(self, deltaTime):
        (<btRaycastVehicle*>self.thisptr).updateSuspension(deltaTime)
    
    def updateFriction(self, timeStep):
        (<btRaycastVehicle*>self.thisptr).updateFriction(timeStep)
    
    def getRigidBody(self):
        return self.chassis
    
    def getForwardVector(self):    
        """ Worldspace forward vector. """
        cdef btVector3 forwardVector = (<btRaycastVehicle*>self.thisptr).getForwardVector()
        return Vector3(forwardVector.getX(), forwardVector.getY(), forwardVector.getZ())
    
    def getCurrentSpeedKmHour(self):
        """ Velocity of vehicle (positive if velocity vector has same direction as foward vector). """    
        return (<btRaycastVehicle*>self.thisptr).getCurrentSpeedKmHour()
    
    def setCoordinateSystem(self, rightIndex, upIndex, forwardIndex):
        (<btRaycastVehicle*>self.thisptr).setCoordinateSystem(rightIndex, upIndex, forwardIndex)
    
    def getRightAxis(self):
        return (<btRaycastVehicle*>self.thisptr).getRightAxis()

    def getUpAxis(self):
        return (<btRaycastVehicle*>self.thisptr).getUpAxis()
        
    def getForwardAxis(self):
        return (<btRaycastVehicle*>self.thisptr).getForwardAxis()        



# FIXME create + inherit from TypedObject
cdef class TypedConstraint:
    """
    This is the base class for all constraints.
    """
    
    cdef btTypedConstraint* thisptr

    def __dealloc__(self):
        del self.thisptr


    def getRigidBodyA(self):
        cdef RigidBody rb = RigidBody()
        rb.thisptr[0] = self.thisptr.getRigidBodyA()
        return rb


    def getRigidBodyB(self):
        cdef RigidBody rb = RigidBody()
        rb.thisptr[0] = self.thisptr.getRigidBodyB()
        return rb


    def getUserConstraintType(self):
        return self.thisptr.getUserConstraintType()
        
        
    def setUserConstraintType(self, int userConstraintType):
        self.thisptr.setUserConstraintType(userConstraintType)
        
        
    def getUserConstraintId(self):
        return self.thisptr.getUserConstraintId()
        
        
    def setUserConstraintId(self, int uid):
        self.thisptr.setUserConstraintId(uid)


    def getUid(self):
        return self.thisptr.getUid()


    def needsFeedback(self):
        return self.thisptr.needsFeedback()
        
        
    def enableFeedback(self, bool needsFeedback):
        """
        enableFeedback enables reading the applied linear and angular impulses.
        Use getAppliedImpulse, getAppliedLinearImpulse and getAppliedAngularImpulse
        to read feedback information.
        """
        self.thisptr.enableFeedback(needsFeedback)
        
        
    def getAppliedImpulse(self):
        """
        Returns an estimated total applied impulse. 
        """
        return self.thisptr.getAppliedImpulse()
        

    def getConstraintType(self):
        return self.thisptr.getConstraintType()

    
    
cdef class Generic6DofConstraint(TypedConstraint):
    """
    A generic constraint with 6 possible degrees of freedom.
    The first three degrees of freedom are linear (X, Y, Z), and then the
    second three degrees of freedom are rotational (X, Y, Z).
    """

    def __init__(self, *args):
        cdef RigidBody a, b
        cdef Transform frameInA, frameInB
        cdef bool useLinearReferenceFrameA, useLinearReferenceFrameB
        if len(args) == 3:
            b = args[0]
            frameInB = args[1]
            useLinearReferenceFrameB = args[2]
            self.thisptr = new btGeneric6DofConstraint((<btRigidBody*>b.thisptr)[0], frameInB.thisptr[0],
                useLinearReferenceFrameB)
        else:
            assert len(args) == 5
            a = args[0]
            b = args[1]
            frameInA = args[2]
            frameInB = args[3]
            useLinearReferenceFrameA = args[4]
            self.thisptr = new btGeneric6DofConstraint((<btRigidBody*>a.thisptr)[0], (<btRigidBody*>b.thisptr)[0],
                frameInA.thisptr[0], frameInB.thisptr[0], useLinearReferenceFrameA)            


    def calculateTransforms(self):
        """
        Calculates global transform of the offsets.
        """
        cdef Transform a = Transform()
        cdef Transform b = Transform()
        (<btGeneric6DofConstraint*>self.thisptr).calculateTransforms(a.thisptr[0], b.thisptr[1])
        return a, b
    
    
    def getCalculatedTransformA(self):
        """
        Returns the global transform of the offset for body A. 
        """
        cdef Transform t = Transform()
        t.thisptr[0] = (<btGeneric6DofConstraint*>self.thisptr).getCalculatedTransformA()
        return t


    def getCalculatedTransformB(self):
        """
        Returns the global transform of the offset for body B. 
        """
        cdef Transform t = Transform()
        t.thisptr[0] = (<btGeneric6DofConstraint*>self.thisptr).getCalculatedTransformB()
        return t        
    
    
    def getFrameOffsetA(self):
        """
        Returns the global transform of the offset for body A. 
        """
        cdef Transform t = Transform()
        t.thisptr[0] = (<btGeneric6DofConstraint*>self.thisptr).getFrameOffsetA()
        return t


    def getFrameOffsetB(self):
        """
        Returns the global transform of the offset for body B. 
        """
        cdef Transform t = Transform()
        t.thisptr[0] = (<btGeneric6DofConstraint*>self.thisptr).getFrameOffsetB()
        return t        

    
    def buildJacobian(self):
        """ 
        Performs Jacobian calculation, and also calculates angle differences and axis.
        """
        (<btGeneric6DofConstraint*>self.thisptr).buildJacobian()
        
        
    def updateRHS(self, btScalar timeStep):
        (<btGeneric6DofConstraint*>self.thisptr).updateRHS(timeStep)
        

    def getAxis(self, int axis_index):
        """
        Returns the rotation axis as a Vector3, in global coordinates.
        N.B. Use axis_index in [4, 5, 6]
        """
        cdef btVector3 v = (<btGeneric6DofConstraint*>self.thisptr).getAxis(axis_index)
        return Vector3(v.getX(), v.getY(), v.getZ())
    
    
    def getAngle(self, int axis_index):
        """
        Returns the relative euler angle rotation of the given axis.
        N.B. Use axis_index in [1, 2, 3]
        """
        return (<btGeneric6DofConstraint*>self.thisptr).getAngle(axis_index)
        
        
    def getRelativePivotPosition(self, int axis_index):
        """
        Returns the relative position of the constraint pivot.
        """
        return (<btGeneric6DofConstraint*>self.thisptr).getRelativePivotPosition(axis_index)
        

    def testAngularLimitMotor(self, int axis_index):
        """
        Tests angular limit.
        """
        return (<btGeneric6DofConstraint*>self.thisptr).testAngularLimitMotor(axis_index)

    
    def setLinearLowerLimit(self, Vector3 linearLower):
        (<btGeneric6DofConstraint*>self.thisptr).setLinearLowerLimit(
            btVector3(linearLower.x, linearLower.y, linearLower.z))
            

    def setLinearUpperLimit(self, Vector3 linearUpper):
        (<btGeneric6DofConstraint*>self.thisptr).setLinearUpperLimit(
            btVector3(linearUpper.x, linearUpper.y, linearUpper.z))


    def setAngularLowerLimit(self, Vector3 angularLower):
        (<btGeneric6DofConstraint*>self.thisptr).setAngularLowerLimit(
            btVector3(angularLower.x, angularLower.y, angularLower.z))
            

    def setAngularUpperLimit(self, Vector3 angularUpper):
        (<btGeneric6DofConstraint*>self.thisptr).setAngularUpperLimit(
            btVector3(angularUpper.x, angularUpper.y, angularUpper.z))
       

    def getRotationalLimitMotor(self, int index):
        # TODO without the creation of the extra btRotationalLimitMotor
        cdef RotationalLimitMotor rlm = RotationalLimitMotor()
        del rlm.thisptr
        rlm.thisptr = (<btGeneric6DofConstraint*>self.thisptr).getRotationalLimitMotor(index)
        rlm.ownsptr = False
        return rlm
        

    # TODO
    '''
    def getTranslationalLimitMotor(self, int index):
        # TODO without the creation of the extra btTranslationalLimitMotor
        TranslationalLimitMotor tlm = TranslationalLimitMotor()
        del tlm.thisptr
        tlm.thisptr = (<btGeneric6DofConstraint*>self.thisptr).getTranslationalLimitMotor(index)
        return tlm
    '''

    def setLimit(self, int axis, btScalar lo, btScalar hi):
        (<btGeneric6DofConstraint*>self.thisptr).setLimit(axis, lo, hi)

    
    def isLimited(self, int limitIndex):
        """
        Test limit.
        """
        return (<btGeneric6DofConstraint*>self.thisptr).isLimited(limitIndex)


    def calcAnchorPos(self):
        (<btGeneric6DofConstraint*>self.thisptr).calcAnchorPos()


    def getUseFrameOffset(self):
        return (<btGeneric6DofConstraint*>self.thisptr).getUseFrameOffset()
        
        
    def setUseFrameOffset(self, bool frameOffsetOnOff):
        (<btGeneric6DofConstraint*>self.thisptr).setUseFrameOffset(frameOffsetOnOff)
    
   
    def setParam(self, int num, btScalar value, int axis=-1):
        """
        Overrides the default global value of a parameter (such as ERP or CFM).
        If axis is not specified, sets the parameter on all axes.
        """
        (<btGeneric6DofConstraint*>self.thisptr).setParam(num, value, axis)

    
    def getParam(self, int num, int axis=-1):
        return (<btGeneric6DofConstraint*>self.thisptr).getParam(num, axis)

        
    
cdef class Generic6DofSpringConstraint(Generic6DofConstraint):
    """
    The Generic 6 degrees-of-freedom spring constraint adds the ability to
    add spring motors to any translational or rotational degree of freedom.
    
    From the Bullet docs:
    DOF index used in enableSpring() and setStiffness() means:
        0: translation X
        1: translation Y
        2: translation Z
        3: rotation X (3rd Euler rotational around new position of X axis, range [-PI+epsilon, PI-epsilon] )
        4: rotation Y (2nd Euler rotational around new position of Y axis, range [-PI/2+epsilon, PI/2-epsilon] )
        5: rotation Z (1st Euler rotational around Z axis, range [-PI+epsilon, PI-epsilon] )
    """
    
    def __init__(self, RigidBody a, RigidBody b, Transform frameInA, Transform frameInB, bool useLinearReferenceFrameA):
        self.thisptr = new btGeneric6DofSpringConstraint((<btRigidBody*>a.thisptr)[0], (<btRigidBody*>b.thisptr)[0],
            frameInA.thisptr[0], frameInB.thisptr[0], useLinearReferenceFrameA)
    
    def enableSpring(self, int index, bool onOff):
        (<btGeneric6DofSpringConstraint*>self.thisptr).enableSpring(index, onOff)
    
    def setStiffness(self, int index, btScalar stiffness):
        (<btGeneric6DofSpringConstraint*>self.thisptr).setStiffness(index, stiffness)
    
    def setDamping(self, int index, btScalar damping):
        (<btGeneric6DofSpringConstraint*>self.thisptr).setDamping(index, damping)
    
    def setEquilibriumPoint(self, index=None):
        """
        Sets the equilibrium points of this constraint.
        If index is None, sets for all. Otherwise, only sets the constraint for
        the given degree of freedom. 
        """
        if index is None:
            (<btGeneric6DofSpringConstraint*>self.thisptr).setEquilibriumPoint()
        else:
            (<btGeneric6DofSpringConstraint*>self.thisptr).setEquilibriumPoint(index)            


    
cdef class Hinge2Constraint(Generic6DofSpringConstraint):
    """
    The hinge-2 constraint has its origins in ODE, and can be used to effectively
    the constraint between a car body and its wheels. It models a springy suspension
    and 2 axes of rotational freedom.
    
    In terms of a vehicle simulation, the first axis re-points the wheel (i.e.
    steers the wheel, e.g. the Y axis). The second axis is the drive axis (i.e.
    moves the wheel forward or backward on its current path).
    """
    
    def __init__(self, RigidBody a, RigidBody b, Vector3 anchor, Vector3 axis1, Vector3 axis2):
        cdef btVector3 *_anchor = new btVector3(anchor.x, anchor.y, anchor.z)
        cdef btVector3 *_axis1 = new btVector3(axis1.x, axis1.y, axis1.z)
        cdef btVector3 *_axis2 = new btVector3(axis2.x, axis2.y, axis2.z)          
        self.thisptr = new btHinge2Constraint(
            (<btRigidBody*>a.thisptr)[0],
            (<btRigidBody*>b.thisptr)[0],
            _anchor[0], _axis1[0], _axis2[0])
        del _anchor
        del _axis1
        del _axis2

    def getAnchor(self):
        cdef btVector3 v = (<btHinge2Constraint*>self.thisptr).getAnchor()
        return Vector3(v.getX(), v.getY(), v.getZ())

    def getAnchor2(self):
        cdef btVector3 v = (<btHinge2Constraint*>self.thisptr).getAnchor2()
        return Vector3(v.getX(), v.getY(), v.getZ())

    def getAxis1(self):
        cdef btVector3 v = (<btHinge2Constraint*>self.thisptr).getAxis1()
        return Vector3(v.getX(), v.getY(), v.getZ())

    def getAxis2(self):
        cdef btVector3 v = (<btHinge2Constraint*>self.thisptr).getAxis2()
        return Vector3(v.getX(), v.getY(), v.getZ())

    def getAngle1(self):
        return (<btHinge2Constraint*>self.thisptr).getAngle1()
        
    def getAngle2(self):
        return (<btHinge2Constraint*>self.thisptr).getAngle2()
        
    def setUpperLimit(self, btScalar maximumAngle):
        (<btHinge2Constraint*>self.thisptr).setUpperLimit(maximumAngle)

    def setLowerLimit(self, btScalar minimumAngle):
        (<btHinge2Constraint*>self.thisptr).setUpperLimit(minimumAngle)
                
    
    
# TODO TranslationalLimitMotor    
cdef class RotationalLimitMotor:

    cdef btRotationalLimitMotor *thisptr
    cdef bool ownsptr
    
    def __cinit__(self):
        self.thisptr = new btRotationalLimitMotor()
        self.ownsptr = True
    
        
    def __dealloc__(self):
        if self.ownsptr:
            del self.thisptr
        
    
    def isLimited(self):
        return self.thisptr.isLimited()
        
        
    def needApplyTorques(self):
        """
        Returns True iff correction is needed (based on the ERP parameters).
        """        
        return self.thisptr.needApplyTorques()
        
        
    def testLimitValue(self, btScalar test_value):
        return self.thisptr.testLimitValue(test_value)

    # TODO figure out if we need to implement solveAngularLimits.
    # Looks like it is called internally by the engine.

    property loLimit:
        def __get__(self):
            return self.thisptr.m_loLimit
        def __set__(self, value):
            self.thisptr.m_loLimit = value

    property hiLimit:
        def __get__(self):
            return self.thisptr.m_hiLimit
        def __set__(self, value):
            self.thisptr.m_hiLimit = value

    property targetVelocity:
        def __get__(self):
            return self.thisptr.m_targetVelocity
        def __set__(self, value):
            self.thisptr.m_targetVelocity = value

    # according to one source, this is apparently not a force
    # but in fact angular momentum (Newton-metre-seconds)
    property maxMotorForce:
        def __get__(self):
            return self.thisptr.m_maxMotorForce
        def __set__(self, value):
            self.thisptr.m_maxMotorForce = value

    property maxLimitForce:
        def __get__(self):
            return self.thisptr.m_maxLimitForce
        def __set__(self, value):
            self.thisptr.m_maxLimitForce = value

    property limitSoftness:
        def __get__(self):
            return self.thisptr.m_limitSoftness
        def __set__(self, value):
            self.thisptr.m_limitSoftness = value

    property normalCFM:
        def __get__(self):
            return self.thisptr.m_normalCFM
        def __set__(self, value):
            self.thisptr.m_normalCFM = value

    property stopERP:
        """
        Error tolerance factor when joint is at limit. 
        """
        def __get__(self):
            return self.thisptr.m_stopERP
        def __set__(self, value):
            self.thisptr.m_stopERP = value

    property stopCFM:
        """    
        Constraint force mixing factor when joint is at limit. 
        """
        def __get__(self):
            return self.thisptr.m_stopCFM
        def __set__(self, value):
            self.thisptr.m_stopCFM = value

    property enableMotor:
        def __get__(self):
            return self.thisptr.m_enableMotor
        def __set__(self, value):
            self.thisptr.m_enableMotor = value
            
    property damping:
        def __get__(self):
            return self.thisptr.m_damping
        def __set__(self, value):
            self.thisptr.m_damping = value
            
    property bounce:
        def __get__(self):
            return self.thisptr.m_bounce
        def __set__(self, value):
            self.thisptr.m_bounce = value
            
    # XXX documentation says it's a temporary variable, I don't know
    # if we need it.
    property currentLimitError:
        def __get__(self):
            return self.thisptr.m_currentLimitError
        def __set__(self, value):
            self.thisptr.m_currentLimitError = value
            
    property currentPosition:
        def __get__(self):
            return self.thisptr.m_currentPosition
        def __set__(self, value):
            self.thisptr.m_currentPosition = value
            
    property currentLimit:
        def __get__(self):
            return self.thisptr.m_currentLimit
        def __set__(self, value):
            self.thisptr.m_currentLimit = value                                                

    property accumulatedImpulse:
        def __get__(self):
            return self.thisptr.m_accumulatedImpulse
        def __set__(self, value):
            self.thisptr.m_accumulatedImpulse = value                                                


