# -*- coding: utf-8 -*-

import os
path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', pluginName='SoftRobots SofaOpenglVisual')
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')

    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1])
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    ##########################################
    # FEM Model                              #
    ##########################################
    finger = rootNode.addChild('finger')
    finger.addObject('EulerImplicitSolver')
    finger.addObject('CGLinearSolver', iterations=25, tolerance=1e-5, threshold=1e-5)

    finger.addObject('MeshVTKLoader', name='loader', filename=path + 'finger.vtk')
    finger.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetras='@loader.tetras', name='container')
    finger.addObject('TetrahedronSetTopologyModifier')

    # Add a mechanicaobject component to stores the DoFs of the model
    finger.addObject('MechanicalObject', name='tetras', template='Vec3')
    finger.addObject('VolumeFromTetrahedrons')
    finger.addObject('TetrahedronFEMForceField', poissonRatio=0.3, youngModulus=500)

    ##########################################
    # Visualization                          #
    ##########################################
    fingerVisu = finger.addChild('visu')
    fingerVisu.addObject('MeshSTLLoader', filename=path + "finger.stl", name="loader")
    fingerVisu.addObject('OglModel', src="@loader", color=[0.0, 0.7, 0.7, 0.5])
    fingerVisu.addObject('BarycentricMapping')

    return rootNode