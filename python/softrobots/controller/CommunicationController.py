import os
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

def CommunicationController(name="communicationController", 
	listening='1', 
	job="receiver", 
	port="5558", 
	nbDataField="4", 
	pattern="0"):

    """Creates and adds a communication controller.

    The constraint apply to a parent mesh.

    Args:
        name (str): TODO.

        listening (int): TODO.

        job (str): either "force" or "displacement". Default is displacement.

        port (int): TODO.

        nbDataField (int): TODO.

        pattern (int): TODO.


    Structure:
        .. sourcecode:: qml

            Node : {
                    name : "communicationController"
            }

    """

    
def createScene(node):
    from stlib.scene import MainHeader
    from stlib.physics.deformable import ElasticMaterialObject
    from stlib.physics.constraints import FixedBox

    mainHeader = MainHeader(node, "0.0, -981, 0.0", 0.01, ["SoftRobots"])
    mainHeader.createObject('BackgroundSetting', color="0 0.168627 0.211765")


    ElasticMaterialObject(fromVolumeMesh=path+'Springy.vtk',
        withTotalMass=0.03,
        attachedTo=node,
        withName="accordion",
        withYoungModulus=500)

    FixedBox(atPosition="-2 -2 0 2 2 0.5",
        applyTo="accordion",
        withName="ROI1",
        withVisualization(true))


