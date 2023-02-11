from photonvision import PhotonCamera
from robotpy_toolkit_7407.sensors.odometry import VisionEstimator
from wpilib import Timer
from wpimath.geometry import Transform3d

import constants


class Camera:
    def __init__(self, camera_name):
        self.camera = PhotonCamera(cameraName=camera_name)
        self.offset = constants.kCameras[camera_name][0]

    def get_estimated_transforms(self) -> list[tuple[int, Transform3d]]:
        """
        :return: A list of tuples of the form (tag_id, transform_camera_to_target)
        :rtype: list[tuple[int, Transform3d]]
        """
        cameraToTargets = []
        photonResult = self.camera.getLatestResult()
        if photonResult.hasTargets():
            cameraToTargets += [
                (target.getFiducialId(), target.getBestCameraToTarget())
                for target in photonResult.getTargets()
            ]
        return cameraToTargets


class PV_Cameras(VisionEstimator):
    def __init__(self):
        super().__init__()
        self.cameras = [Camera(camera_name) for camera_name in constants.kCameras]

    def get_estimated_robot_pose(self) -> list[tuple[Transform3d, float]]:
        """
        Gets the estimated robot pose from the cameras with FPGA Timestamp
        :return: A list of tuples of the form (pose, timestamp)
        :rtype: list[tuple[Transform3d, float]]
        """
        derivedRobotPoses = []
        for camera in self.cameras:
            cameraToTargets = camera.get_estimated_transforms()

            if cameraToTargets != 0:
                derivedRobotPoses += [
                    (
                        constants.ApriltagPositionDict[tag_id]
                        + point.inverse()
                        + camera.offset.inverse(),
                        Timer.getFPGATimestamp(),
                    )
                    for tag_id, point in cameraToTargets
                ]

        return derivedRobotPoses
