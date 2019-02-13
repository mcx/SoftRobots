# -*- coding: utf-8 -*-
"""
Step 9: Adding the rigidification to the tutorial.

"""
from splib.objectmodel import setData
from splib.numerics import sin, cos, to_radians, Quat, Vec3
from stlib.scene import Scene
from stlib.physics.mixedmaterial import Rigidify
from actuatedarm import ActuatedArm
from tripod import ElasticBody
from tripodcontroller import MyController


def Effector(tripod, positions):
    e = tripod.createChild("Effector")
    # Select the center points for the effector.
    s = e.createObject("SphereROI", name="roi", template="Rigid3",
                       position=positions,
                       # TODO (damien) Why the radii is not correct in the scene ? 
                       centers=[0, 30.0, 0], radii=[10.0], drawSphere=True)
    s.init()
    return e


def EffectorGoal(parentNode):
    goal = parentNode.createChild('EffectorGoal')
    goal.createObject('EulerImplicit', firstOrder='1')
    goal.createObject('CGLinearSolver', iterations='200', threshold="1e-5", tolerance="1e-5")
    goal.createObject('MechanicalObject', name='dofs',
                      showObject="1",
                      showObjectScale="3",
                      drawMode="1",
                      position = "0 32 0")
    goal.createObject('UncoupledConstraintCorrection')
    return goal

import Sofa
class MyController2(Sofa.PythonScriptController):
    """Copy the inverse angle into the servo motor's 
    """
    def __init__(self, node, tripod):
        self.name = "CopyBack"
        self.tripod = tripod
        
    def onEndAnimationStep(self, dt):
        #print("COUCOU " + str(self.tripod.RigidifiedStructure.RigidParts.actuator0.displacement))
        #elf.tripod.ActuatedArm0.ServoMotor.angle = self.tripod.RigidifiedStructure.RigidParts.actuator0.displacement
        pass 
# Let's define a Tripod prefab now, that we can later call in the createScene
# function
def Tripod(parent, name="Tripod", radius=55, numMotors=3, angleShift=180.0, goal=None):
    tripod = parent.createChild(name)

    # It is using the ElasticBody that was made in the previous step, and that
    # has also been included in the tripod.py script.
    body = ElasticBody(tripod)
    body.init()
    # The actuated arms are positionned around the silicone piece using a loop
    # structure
    dist = radius
    numstep = numMotors
    b = []
    arms = []
    angles = []
    for i in range(0, numstep):
        name = "ActuatedArm"+str(i)
        fi = float(i)
        fnumstep = float(numstep)
        angle = fi*360/fnumstep
        angle2 = fi*360/fnumstep+angleShift
        eulerRotation = [0, angle, 0]
        angles.append([0, to_radians(angle), 0])
        translation = [dist*sin(to_radians(angle2)),
                       -1.35,
                       dist*cos(to_radians(angle2))]

        c = ActuatedArm(tripod, name=name,
                        translation=translation, eulerRotation=eulerRotation)

        c.addBox(body.ElasticMaterialObject.dofs.getData("rest_position"),
                 translation, eulerRotation)
        arms.append(c)
        b.append(map(lambda x: x[0], c.Box.BoxROI.indices))
    
    e = Effector(tripod, body.ElasticMaterialObject.dofs.getData("rest_position"))
    b.append(map(lambda x: x[0], tripod.Effector.roi.indices))
    
    o = Rigidify(tripod,
                 body.ElasticMaterialObject,
                 name="RigidifiedStructure",
                 frameOrientation=angles+[[0.0,0.0,0.0]],
                 groupIndices=b)

    o.createObject('MechanicalMatrixMapper',
                        template='Vec3,Rigid3',
                        object1=o.DeformableParts.getLinkPath(),
                        object2=o.RigidParts.dofs.getLinkPath(),
                        nodeToParse = o.RigidParts.RigidifiedParticules.ElasticMaterialObject.getLinkPath())

    
    ##########
    # mapping of effector point
    ##########
    e.createObject('MechanicalObject', template='Vec3d', name="effector", position='0 10 0')
    e.createObject('RigidMapping', template="Rigid3, Vec3", rigidIndexPerPoint='3',
                   input=tripod.RigidifiedStructure.RigidParts.getLinkPath())
    
    e.createObject('PositionEffector', template='Vec3d', indices="0", effectorGoal=goal.dofs.getData("position").getLinkPath(), useDirections='1 1 1')

    for i in range(0, numMotors):
        a = arms[i].ServoArm.createChild("Attach")
        a.createObject("MechanicalObject", template="Rigid3d", name="dofs",
                       showObject=True, showObjectScale=10,
                       position=[0.0, 25.0, 10.0, 0, 0, 0, 1])
        a.createObject("RigidRigidMapping")

        if i == 0:
            rotatedPoint = Vec3(1.0,0.0,0.0).rotateFromEuler(angles[i])
        if i == 1:
            rotatedPoint = Vec3(1.0,0.0,0.0).rotateFromEuler(angles[2])
        if i == 2:
            rotatedPoint = Vec3(1.0,0.0,0.0).rotateFromEuler(angles[1])

        o.RigidParts.createObject('SlidingActuator',
                                  name="actuator"+str(i), template="Rigid3",
                                  direction=[0, 0, 0]+list(rotatedPoint), indices=i,
                                  maxPositiveDisp=0.6,
                                  maxNegativeDisp=0.4,
                                  maxDispVariation=0.05)

        o.RigidParts.createObject('RestShapeSpringsForceField',
                                  external_rest_shape=arms[i].ServoArm.Attach.dofs.getLinkPath(),
                                  points=[i], external_points=[0],
                                  angularStiffness=1e8, stiffness=1e10)

    for i in range(0, numMotors):
        arms[i].createObject("FixedConstraint")
        arms[i].ServoMotor.ServoWheel.createObject("FixedConstraint")

    setData(o.RigidParts.dofs,
            showObject=True, showObjectScale=10.0, drawMode=0)
    setData(o.RigidParts.RigidifiedParticules.dofs, showObject=True,
            showObjectScale=5.0, drawMode=0, showColor=[1.0, 1.0, 0.0, 1.0])
    setData(o.DeformableParts.dofs, showObject=True,
            showObjectScale=1.0, drawMode=2)

    tripod.removeChild(body)
    return tripod


def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots", "SoftRobots.Inverse"])
    scene.VisualStyle.displayFlags = "showBehavior"
    scene.createObject("FreeMotionAnimationLoop")
    scene.createObject('QPInverseProblemSolver', printLog='0', epsilon='0.01')

    helper = scene.createChild("GroundPrint")
    helper.createObject("MeshSTLLoader", name="loader",
                       filename="data/mesh2/blueprint.stl")
    helper.createObject("OglModel", src="@loader")

    # Create the effector goal
    goal = EffectorGoal(scene)
    
    # Create the tripod object.
    tripod = Tripod(rootNode, goal=goal)

    # The regular controller that is being used for the last 2 steps
    MyController(rootNode, [tripod.ActuatedArm0,
                            tripod.ActuatedArm1,
                            tripod.ActuatedArm2])

    #MyController2(rootNode, tripod)

    simulation = scene.createChild("Simulation")
    simulation.createObject("EulerImplicitSolver", firstOrder=0)
    simulation.createObject("SparseLDLSolver")
    simulation.createObject("GenericConstraintCorrection")
    simulation.addChild(tripod)      


    
