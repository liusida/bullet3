import pybullet as p
from PyBulletWrapper.pybullet_wrapper.base import BaseWrapperPyBullet
from PyBulletWrapper.pybullet_wrapper.pretty_float import PrettyFloatPyBullet
from PyBulletWrapper.pybullet_wrapper.handy import HandyPyBullet

p = BaseWrapperPyBullet(p)
p = PrettyFloatPyBullet(p)
p = HandyPyBullet(p)

p.start()

boneId = p.createCollisionShapePy(shapeType=p.GEOM_BOX, halfExtents=[0.1, 0.1, 1])
linkPositions = []
linkOrientations = []
linkCollisionShapeIndices = []
linkParentIndices = []
linkMasses = []
linkJointTypes = []
linkJointAxis = []
linkVisualShapeIndices = []
linkOrientations = []
linkInertialFramePositions = []
linkInertialFrameOrientations = []

linkMasses.append(1)
linkCollisionShapeIndices.append(boneId)
linkVisualShapeIndices.append(-1)
linkPositions.append([0, 0, 2])
linkOrientations.append([0, 0, 0, 1])
linkParentIndices.append(0)
linkJointTypes.append(p.JOINT_SPHERICAL)
linkJointAxis.append([1, 0, 0])
linkInertialFramePositions.append([0,0,0])
linkInertialFrameOrientations.append([0, 0, 0, 1])

robot = p.createMultiBodyPy(
    baseMass=1, baseCollisionShapeIndex=boneId, basePosition=[0, 0, 1],
    baseVisualShapeIndex=-1, baseOrientation=[0, 0, 0, 1], baseInertialFramePosition=[0.1,0.1,1],
    linkPositions=linkPositions, linkOrientations=linkOrientations, linkCollisionShapeIndices=linkCollisionShapeIndices, linkParentIndices=linkParentIndices,
    linkMasses=linkMasses, linkJointTypes=linkJointTypes, linkJointAxis=linkJointAxis, linkVisualShapeIndices=linkVisualShapeIndices,
    linkInertialFramePositions=linkInertialFramePositions, linkInertialFrameOrientations=linkInertialFrameOrientations)

p.setJointMotorControl2

while 1:
    p.stepSimulation()
    p.sleep(.001)
