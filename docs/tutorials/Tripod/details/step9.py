# -*- coding: utf-8 -*-
"""
Step 9: Adding the rigidification to the tutorial.

"""
from splib.objectmodel import setData
from splib.numerics import sin, cos, to_radians
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
                       centers=[0, 30.0, 0], radii=[10.0], drawSphere=True)
    s.init()
    return e


# Let's define a Tripod prefab now, that we can later call in the createScene function
def Tripod(parent, name="Tripod", radius=55, numMotors=3, angleShift=180.0):
    tripod = parent.createChild(name)

    # It is using the ElasticBody that was made in the previous step, and that has also been included in the tripod.py script.
    body = ElasticBody(tripod)
    body.init()
    # The actuated arms are positionned around the silicone piece using a loop structure
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
        translation = [dist*sin(to_radians(angle2)), -1.35, dist*cos(to_radians(angle2))]

        c = ActuatedArm(tripod, name=name,
                        translation=translation, eulerRotation=eulerRotation)

        c.addBox(body.ElasticMaterialObject.dofs.getData("rest_position"), translation, eulerRotation)
        arms.append(c)
        b.append(map(lambda x: x[0], c.Box.BoxROI.indices))

    o = Rigidify(tripod,
                 body.ElasticMaterialObject,
                 name="RigidifiedStructure",
                 frameOrientation=angles,
                 groupIndices=b)

    o.createObject('MechanicalMatrixMapper', template='Vec3,Rigid3',
                   object1=tripod.RigidifiedStructure.DeformableParts.getLinkPath(), object2=tripod.RigidifiedStructure.RigidParts.getLinkPath(),
                   nodeToParse=tripod.RigidifiedStructure.DeformableParts.ElasticMaterialObject.getLinkPath())

    for i in range(0, 3):
        a = arms[i].ServoArm.createChild("Attach")
        a.createObject("MechanicalObject", template="Rigid3d", name="dofs", showObject=True, showObjectScale=10,
                       position=[0.0, 25.0, 10.0, 0, 0, 0, 1])
        a.createObject("RigidRigidMapping")

    o.RigidParts.createObject('RestShapeSpringsForceField',
                              external_rest_shape=arms[0].ServoArm.Attach.dofs.getLinkPath(), points=[0], external_points=[0],
                              angularStiffness=1e8, stiffness=1e10)
    o.RigidParts.createObject('RestShapeSpringsForceField',
                              external_rest_shape=arms[1].ServoArm.Attach.dofs.getLinkPath(), points=[1], external_points=[0],
                              angularStiffness=1e8, stiffness=1e10)
    o.RigidParts.createObject('RestShapeSpringsForceField',
                              external_rest_shape=arms[2].ServoArm.Attach.dofs.getLinkPath(), points=[2], external_points=[0],
                              angularStiffness=1e8, stiffness=1e10)

    for i in range(0, 3):
        arms[i].createObject("FixedConstraint")
        arms[i].ServoMotor.ServoWheel.createObject("FixedConstraint")

    setData(o.RigidParts.dofs, showObject=True, showObjectScale=10.0, drawMode=0)
    setData(o.RigidParts.RigidifiedParticules.dofs, showObject=True, showObjectScale=5.0, drawMode=0, showColor=[1.0, 1.0, 0.0, 1.0])
    setData(o.DeformableParts.dofs, showObject=True, showObjectScale=1.0, drawMode=2)

    tripod.removeChild(body)
    return tripod


def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, -981.0, 0.0])
    scene.VisualStyle.displayFlags = "showBehavior"

    scene.createObject("MeshSTLLoader", name="loader", filename="data/mesh2/blueprint.stl")
    scene.createObject("OglModel", src="@loader")

    tripod = Tripod(rootNode)

    # The regular controller that is being used for the last 2 steps
    # MyController(rootNode, [tripod.ActuatedArm0,
    #                         tripod.ActuatedArm1,
    #                         tripod.ActuatedArm2])

    simulation = scene.createChild("Simulation")
    simulation.createObject("EulerImplicitSolver", firstOrder=0)
    simulation.createObject("SparseLDLSolver")

    simulation.addChild(tripod)
