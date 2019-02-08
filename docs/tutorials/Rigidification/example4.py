# -*- coding: utf-8 -*-
import splib
from splib.numerics import Vec3, Quat, sdiv, RigidDof, getOrientedBoxFromTransform 
from stlib.scene import MainHeader, ContactHeader
from stlib.physics.deformable import ElasticMaterialObject
from stlib.physics.constraints import FixedBox
from mixedmaterial import Rigidify, boxFilter

def addToSimulation(simulationNode, modelNode):
        simulationNode.addChild(modelNode)
        
def addSubPointToRigid(c, p):
        s = c.createChild("subpoints")
        s.createObject("MechanicalObject", name="dofs", position=p)        
        s.createObject("RigidMapping", globalToLocalCoords=True)

def setData(d, **kwargs):
        for k in kwargs:
            print("k.V: "+str((k)))
            d.getData(str(k)).value = kwargs[k]
            #d.showObject=True
            #d.showObjectScale=0.2
            #d.drawMode=2
        
def getAnimationLoop(node):
        loops = ["DefaultAnimationLoop","FreeMotionAnimationLoop"]
        def filterLoop(node, loops):
                for obj in node.getObjects():
                        print("O ", obj.getCategories())
                        c=obj.getCategories()
                        
                        if "BaseConstraint" in c:
                                loops.remove("DefaultAnimationLoop")
                        
                        if "BaseContactManager" in c:
                                print("RESPONSE: ", c.response)
                                loops.remove("DefaultAnimationLoop")
                         
                for child in node.getChildren():             
                        filterLoop(child, loops)
                        
        filterLoop(node, loops)
        return loops
        
        
def createScene(rootNode):
        MainHeader(rootNode, plugins=["SofaSparseSolver"])
        rootNode.VisualStyle.displayFlags="showBehavior"
        rootNode.createObject("DefaultAnimationLoop")
        rootNode.createObject("DefaultVisualManagerLoop")
        
        modelNode = rootNode.createChild("Modeling")        
        elasticobject = ElasticMaterialObject(modelNode, 
                                              rotation=[90,0,0], 
                                              volumeMeshFileName="data/tripod_low.gidmsh", 
                                              youngModulus=100, poissonRatio=0.4)
    
        b = boxFilter(modelNode, elasticobject, 
                                         orientedBoxes=[ 
                                         getOrientedBoxFromTransform(translation=[20,0,0],
                                                                     eulerRotation=[0,0,0], 
                                                                     scale=[10.0,10.0,40.0]),
                                         getOrientedBoxFromTransform(translation=[-35,-00,25],
                                                                     eulerRotation=[0,0,0], 
                                                                     scale=[30.0,20.0,30.0]),
                                         getOrientedBoxFromTransform(translation=[0,0,0],
                                                                     eulerRotation=[0,00,0], 
                                                                     scale=[30.0,20.0,30.0])
                                                                      ])    
        o = Rigidify(modelNode,
                     elasticobject,
                     name="RigidifiedStructure", 
                     frameOrientation = [[0.0,0.0,0.0], [0.0,0.0,0.0], [0.0,0.0,0.0]],
                     #groupIndices=b.getIndices())
                     groupIndices=b)
        
        setData(o.RigidParts.dofs, showObject=True, showObjectScale=10.0,drawMode=2)
        setData(o.RigidParts.RigidifiedParticules.dofs, showObject=True, showObjectScale=5.0, drawMode=0, showColor=[1.0,1.0,0.0,1.0])
        setData(o.DeformableParts.dofs, showObject=True, showObjectScale=1.0, drawMode=2)
        
        o.RigidParts.createObject("FixedConstraint", indices=0)
        
        simulation = rootNode.createChild("Simulation")
        #simulation.createObject("DefaultAnimationLoop")
        #simulation.createObject("DefaultVisualManagerLoop")
        simulation.createObject("EulerImplicitSolver", name="integrationscheme", rayleighStiffness=0.01)
        simulation.createObject("CGLinearSolver", name="solver")
        addToSimulation(simulation, o)            
        
