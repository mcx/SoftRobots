# -*- coding: utf-8 -*-
import splib
from splib.numerics import Vec3, Quat, sdiv, RigidDof, getOrientedBoxFromTransform 
from stlib.scene import MainHeader, ContactHeader
from stlib.physics.deformable import ElasticMaterialObject
from stlib.physics.constraints import FixedBox

## TODO List
## Crash when root contains MechanicalObject



def ElastoRigidObject(targetObject, sourceObject, boxFrames, orientedBoxes):
        sourceObject.init()
        ero = targetObject.createChild("ElastoRigidObject")

        allPositions = sourceObject.container.position
        allIndices = map( lambda x: x[0], sourceObject.container.points )
        
        centers = []
        selectedIndices=[]
        for index in range(len(orientedBoxes)):
                orientedBox = orientedBoxes[index]
                boxFrame = boxFrames[index]
                box = ero.createObject("BoxROI", name="filters",
                               orientedBox=orientedBox,
                               position=sourceObject.container.position, 
                               drawBoxes=True, drawPoints=True, drawSize=2.0)
                box.init()
                orientation = Quat.createFromEuler(boxFrame)
                center = sdiv( sum(map(Vec3, box.pointsInROI)), float(len(box.pointsInROI))) + list(orientation)
                centers.append(center)
                selectedIndices += map(lambda x: x[0], box.indices)
 
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
        
        print("Center"+repr(centers))
        
        rigidParts = ero.createChild("RigidParts")
        rigidParts.createObject("MechanicalObject", template="Rigid", name="dofs", reserve=len(centers), 
                                showObject=True, showObjectScale=15, position=centers)
        
        #for center in centers:                         
        #        RigidDof(rigidParts.dofs).setPosition(center)
        
        rigidifiedParticules=rigidParts.createChild("RigidifiedParticules")
        rigidifiedParticules.createObject("MechanicalObject", template="Vec3", name="dofs",
                                                        position=[allPositions[i] for i in selectedIndices],
                                                        showObject=True, showObjectScale=5, showColor=[1.0,1.0,0.0,1.0])
        rigidifiedParticules.createObject("RigidMapping", globalToLocalCoords='true')

        interactions = ero.createChild("MaterialCoupling")
        c = sourceObject.container                          
        sourceObject.removeObject(c)
        sourceObject.removeObject(sourceObject.forcefield)
        interactions.addObject(c)
        mass = sourceObject.mass
        sourceObject.removeObject(mass)
        interactions.addObject(mass)
        mass.init()
        sourceObject.removeObject(sourceObject.dofs)
        
        interactions.createObject("MechanicalObject", 
                                  template="Vec3", name="dofs", 
                                  position=sourceObject.container.position)
      
        interactions.createObject("SubsetMultiMapping", template="Vec3,Vec3", 
                                  input=freeParticules.freedofs.getLinkPath()+" "+rigidifiedParticules.dofs.getLinkPath(), 
                                  output='@.', 
                                  indexPairs=indexPairs)
                                  
        interactions.createObject("TetrahedronFEMForceField")  
                                  
        rigidifiedParticules.addChild(interactions)
        freeParticules.addChild(interactions)
        ero.removeChild(interactions)
        return ero

        
def createScene(rootNode):
        MainHeader(rootNode, plugins=["SofaSparseSolver"])
        rootNode.VisualStyle.displayFlags="showBehavior"
        rootNode.createObject("DefaultAnimationLoop")
        rootNode.createObject("DefaultVisualManagerLoop")
        
        rootNode.createObject("EulerImplicitSolver", rayleighStiffness=0.01)
        rootNode.createObject("CGLinearSolver")
        elasticobject = ElasticMaterialObject(rootNode, 
                                              rotation=[90,0,0], 
                                              volumeMeshFileName="data/tripod_low.gidmsh", youngModulus=100, poissonRatio=0.4)
        
        fixingboxroi = FixedBox(elasticobject, atPositions=[-9.0, -9.0, -60.0, 9.0, 9.0, -40.0])
        fixingboxroi.BoxROI.drawBoxes=True
        
        
        #z = rootNode.createChild("Zut")                                      
        #frame = z.createObject("MechanicalObject", name="frames", template="Rigid", showObject=True, showObjectScale=10)
        ElastoRigidObject(rootNode, elasticobject, 
                         boxFrames = [[90,60,0], [10,45,0]][1:],
                         orientedBoxes=[ getOrientedBoxFromTransform(translation=[30,0,10],
                                                                     eulerRotation=[0,90,0], 
                                                                     scale=[30.0,20.0,30.0]),
                                         getOrientedBoxFromTransform(translation=[-35,-00,25],
                                                                     eulerRotation=[0,90,0], 
                                                                     scale=[30.0,20.0,30.0])
                                                                      ][1:])
                    
        
        #roi1 = BoxROI(elasticobject)
        #roi2 = BoxROI(elasticobject)
        
        #rigidify(elasticobject, roi1)
        
#   CompositeObject:
#        Topology:
#            MeshTopology
#        ElasticPart:
#            MechnicalObject<Vec3> set1                  
#        RigidPart:
#            MechanicalObject<Rigid>
#            RigidMapping           
#               MechnicalObject<Vec3> set2                  
#        Compound:
#               SubsetMultiMapping<>
#               TetrahedronFEMForceField
