# -*- coding: utf-8 -*-
import splib
from splib.numerics import Vec3, Quat, sdiv, RigidDof, getOrientedBoxFromTransform 
from stlib.scene import MainHeader, ContactHeader
from stlib.physics.deformable import ElasticMaterialObject
from stlib.physics.constraints import FixedBox

## TODO List
## Crash when root contains MechanicalObject

# TODO(dmarchal) Add gather the Rigid DOF-mapping inside rigidified part.
# TODO(dmarchal) Instead of boxframe 
def ElastoRigidObject(targetObject, sourceObject, frameOrientation, orientedBoxes):
        """
            param vertexGroups: 
            param framesOrientation: array of orientation. The length of the array should be equal to the number 
                                    of rigid component. The orientation are given in eulerAngles (in radians) by passing 
                                    three value or using a quaternion by passing four.
                                    [[0,10,20], [0.1,0.5,0.3,0.4]]
                                    
                                    
        """
        sourceObject.init()
        ero = targetObject.createChild("ElastoRigidObject")

        allPositions = sourceObject.container.position
        allIndices = map( lambda x: x[0], sourceObject.container.points )
        
        centers = []
        selectedIndices=[]
        indicesMap=[]
        for index in range(len(orientedBoxes)):
                orientedBox = orientedBoxes[index]
                boxFrame = frameOrientation[index]
                box = ero.createObject("BoxROI", name="filters",
                               orientedBox=orientedBox,
                               position=sourceObject.container.position, 
                               drawBoxes=True, drawPoints=True, drawSize=2.0)
                box.init()
                orientation = Quat.createFromEuler(boxFrame)
                center = sdiv( sum(map(Vec3, box.pointsInROI)), float(len(box.pointsInROI))) + list(orientation)
                centers.append(center)
                selectedIndices += map(lambda x: x[0], box.indices)
                indicesMap += [index] * len(box.indices)
 
        maps = []
        
        otherIndices = filter(lambda x: x not in selectedIndices, allIndices)                   
        Kd = {v:None for k,v in enumerate(allIndices)}        
        Kd.update( {v:[0,k] for k,v in enumerate(otherIndices)} )
        Kd.update( {v:[1,k] for k,v in enumerate(selectedIndices)} )
        indexPairs = [v for kv in Kd.values() for v in kv]
        #print("KV: "+str(selectedIndices))
        #print("KD: "+str(Kd))
        freeParticules = ero.createChild("DeformableParts")
        freeParticules.createObject("MechanicalObject", template="Vec3", name="freedofs",
                                                        position=[allPositions[i] for i in otherIndices],
                                                        showObject=True, showObjectScale=5, showColor=[1.0,0.0,1.0,1.0])
        
        rigidParts = ero.createChild("RigidParts")
        rigidParts.createObject("MechanicalObject", template="Rigid", name="dofs", reserve=len(centers), 
                                showObject=True, showObjectScale=15, position=centers)
        
        #for center in centers:                         
        #        RigidDof(rigidParts.dofs).setPosition(center)
        
        rigidifiedParticules=rigidParts.createChild("RigidifiedParticules")
        rigidifiedParticules.createObject("MechanicalObject", template="Vec3", name="dofs",
                                                        position=[allPositions[i] for i in selectedIndices],
                                                        showObject=True, showObjectScale=5, showColor=[1.0,1.0,0.0,1.0])
        rigidifiedParticules.createObject("RigidMapping", globalToLocalCoords='true', rigidIndexPerPoint=indicesMap)

        interactions = ero.createChild("MaterialCoupling")
        c = sourceObject.container                          
        sourceObject.removeObject(sourceObject.solver)
        sourceObject.removeObject(sourceObject.integration)
        sourceObject.removeObject(sourceObject.LinearSolverConstraintCorrection)
        
        #sourceObject.removeObject(c)
        #sourceObject.removeObject(sourceObject.forcefield)        
        #sourceObject.removeObject(sourceObject.dofs)
        
        interactions.createObject("MechanicalObject", 
                                  template="Vec3", name="dofs", 
                                  position=sourceObject.container.position)
      
        interactions.createObject("SubsetMultiMapping", template="Vec3,Vec3", 
                                  input=freeParticules.freedofs.getLinkPath()+" "+rigidifiedParticules.dofs.getLinkPath(), 
                                  output='@.', 
                                  indexPairs=indexPairs)
                                  
        interactions.createObject("TetrahedronFEMForceField")  
        interactions.createObject("UniformMass", name="mass", vertexMass=sourceObject.mass.vertexMass)
        
        interactions.addObject(c)
                           
        rigidifiedParticules.addChild(interactions)
        freeParticules.addChild(interactions)
        ero.removeChild(interactions)
        sourceObject.node.activated=False
        
        return ero


#    DeformableObjec
#      Volume linear elastic object. 
#        vec3
#        virtual rigidify
        
#    ElasticMaterialObject: DeformableObject
#        lawComportement
#        type élément: déduit du format de fichier. 
#        add: TetraForceField 
#    Comme(ElasticMatirielObject, remplace("TetrahedronFEMFORCE", "FEM"))
        
def createScene(rootNode):
        MainHeader(rootNode, plugins=["SofaSparseSolver"])
        rootNode.VisualStyle.displayFlags="showBehavior"
        rootNode.createObject("DefaultAnimationLoop")
        rootNode.createObject("DefaultVisualManagerLoop")
        
        elasticobject = ElasticMaterialObject(rootNode, 
                                              rotation=[90,0,0], 
                                              volumeMeshFileName="data/tripod_low.gidmsh", 
                                              youngModulus=100, poissonRatio=0.4)
    
        simulation = rootNode.createChild("Simulation")
        simulation.createObject("EulerImplicitSolver", rayleighStiffness=0.01)
        simulation.createObject("CGLinearSolver")
                
        o = ElastoRigidObject(simulation, elasticobject, 
                         frameOrientation = [[0,00,0], [00,00,0], [0,0,0]],
                         orientedBoxes=[ getOrientedBoxFromTransform(translation=[20,0,10],
                                                                     eulerRotation=[0,90,0], 
                                                                     scale=[30.0,20.0,30.0]),
                                         getOrientedBoxFromTransform(translation=[-35,-00,25],
                                                                     eulerRotation=[0,90,0], 
                                                                     scale=[30.0,20.0,30.0]),
                                         getOrientedBoxFromTransform(translation=[0,0,0],
                                                                     eulerRotation=[0,00,0], 
                                                                     scale=[30.0,20.0,30.0])
                                                                      ])
        o.RigidParts.createObject("FixedConstraint", indices=0)
        