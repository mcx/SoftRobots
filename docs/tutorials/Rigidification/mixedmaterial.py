# -*- coding: utf-8 -*-
"""
Created on Fri Feb  8 13:06:35 2019

@author: dmarchal
"""
from splib.numerics import Vec3, Quat, sdiv, RigidDof, getOrientedBoxFromTransform 
from stlib.scene import MainHeader, ContactHeader
from stlib.physics.deformable import ElasticMaterialObject
from stlib.physics.constraints import FixedBox

def Rigidify(targetObject, sourceObject, frameOrientation, groupIndices, name=None):
        """
            param vertexGroups: 
            param framesOrientation: array of orientation. The length of the array should be equal to the number 
                                    of rigid component. The orientation are given in eulerAngles (in radians) by passing 
                                    three value or using a quaternion by passing four.
                                    [[0,3.14/2.0,3.14/3.0], [0.1,0.5,0.3,0.4]]
                                    
                                    
        """
        if name==None:
                name = sourceObject.name
                
        sourceObject.init()
        ero = targetObject.createChild(name)

        allPositions = sourceObject.container.position
        allIndices = map( lambda x: x[0], sourceObject.container.points )
        
        centers = []
        indicesMap=[]
        def mfilter(si, ai, pts):
                tmp=[]
                for index in ai:
                        if index in si:
                                tmp.append(pts[index])
                return tmp;        
        
        # get all the points from the source.         
        sourcePoints = map(Vec3, sourceObject.dofs.position) 
        selectedIndices=[]
        for index in range(len(groupIndices)):
                selectedPoints = mfilter(groupIndices[index], allIndices, sourcePoints)                 
                                
        
                if len(frameOrientation[index])==3:
                        orientation = Quat.createFromEuler(frameOrientation[index])
                else:
                        orientation = frameOrientation[index]
                
                poscenter = [0.0,0.0,0.0]
                if len(selectedPoints) != 0:
                        poscenter = sdiv( sum(selectedPoints), float(len(selectedPoints)))                 
                center = poscenter + list(orientation)
                centers.append(center)
                
                selectedIndices += map(lambda x: x, groupIndices[index])
                indicesMap += [index] * len(groupIndices[index])
 
        maps = []
        
        otherIndices = filter(lambda x: x not in selectedIndices, allIndices)                   
        Kd = {v:None for k,v in enumerate(allIndices)}        
        Kd.update( {v:[0,k] for k,v in enumerate(otherIndices)} )
        Kd.update( {v:[1,k] for k,v in enumerate(selectedIndices)} )
        indexPairs = [v for kv in Kd.values() for v in kv]
        
        freeParticules = ero.createChild("DeformableParts")
        freeParticules.createObject("MechanicalObject", template="Vec3", name="dofs",
                                                        position=[allPositions[i] for i in otherIndices])
        #                                                showObject=True, showObjectScale=5, showColor=[1.0,0.0,1.0,1.0])
        #sourceObject.getData("name").value = "Coupling"        
        
        rigidParts = ero.createChild("RigidParts")
        rigidParts.createObject("MechanicalObject", template="Rigid", name="dofs", reserve=len(centers), position=centers)
                                #showObject=True, showObjectScale=15, )
           
        rigidifiedParticules=rigidParts.createChild("RigidifiedParticules")
        rigidifiedParticules.createObject("MechanicalObject", template="Vec3", name="dofs",
                                                        position=[allPositions[i] for i in selectedIndices])
                                                        #showObject=True, showObjectScale=5, showColor=[1.0,1.0,0.0,1.0])
        rigidifiedParticules.createObject("RigidMapping", name="mapping", globalToLocalCoords='true', rigidIndexPerPoint=indicesMap)

        sourceObject.removeObject(sourceObject.solver)
        sourceObject.removeObject(sourceObject.integration)
        sourceObject.removeObject(sourceObject.LinearSolverConstraintCorrection)
      
        #sourceObject.dofs.position=sourceObject.container.position
        #coupling = freeParticules.createChild("Coupling")
        coupling = sourceObject.node
        #coupling.addChild(sourceObject.node)
        coupling.createObject("SubsetMultiMapping", name="mapping", template="Vec3,Vec3", 
                                 input=freeParticules.dofs.getLinkPath()+" "+rigidifiedParticules.dofs.getLinkPath(), 
                                 output=sourceObject.dofs.getLinkPath(), 
                                 indexPairs=indexPairs)
        
        #rigidifiedParticules.addChild(sourceObject.node)
        rigidifiedParticules.addChild(coupling)
        freeParticules.addChild(coupling)
        #sourceObject.node.getParents()[0].removeChild(sourceObject.node)
        return ero


class BoxROI(object):
        def __init__(self, node, sourceObject, orientedBoxes):      
                node = node.createChild("Select")          
                self.sourceObject = sourceObject
                self.boxes = []
                for orientedBox in orientedBoxes:
                        box = node.createObject("BoxROI", name="filters",
                               orientedBox=orientedBox,
                               position=sourceObject.dofs.findData("position").getLinkPath(), 
                               drawBoxes=True, drawPoints=True, drawSize=2.0)
                        self.boxes.append(box)
                        
        def getIndices(self):
                self.sourceObject.init()
                indices = []
                for box in self.boxes:
                        box.init()
                        indices.append(map(lambda x: x[0], box.indices))
                print("STR: "+str(indices))
                return indices

def boxFilter(node, sourceObject, orientedBoxes):
                sourceObject.init()
   
                selectnode = node.createChild("Select")          
                boxes = []
                for orientedBox in orientedBoxes:
                        box = selectnode.createObject("BoxROI", name="filters",
                               orientedBox=orientedBox,
                               position=sourceObject.dofs.findData("position").getLinkPath(), 
                               drawBoxes=True, drawPoints=True, drawSize=2.0)
                        boxes.append(box)
                        
                indices = []
                for box in boxes:
                        box.init()
                        indices.append(map(lambda x: x[0], box.indices))
                node.removeChild(selectnode)
                return indices
