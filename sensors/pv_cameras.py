from photonvision import PhotonCamera
from robotpy_toolkit_7407.sensors.odometry import VisionEstimator
from wpilib import Timer

import constants


class PV_Cameras(VisionEstimator):
    def __init__(self, april_tags):
        super().__init__()
        self.cameras = [
            (
                PhotonCamera(cameraName=camera_name),
                constants.kCameras[camera_name][0],
            )
            for camera_name in constants.kCameras
        ]
        self.april_tags = april_tags

    def get_estimated_robot_pose(self):
        """this function returns a list of the type (target_id, transformCameraToTarget) for every target"""
        derivedRobotPoses = []
        for camera in self.cameras:  # Iterate through all cameras
            cameraToTargets = []
            photonResult = camera[0].getLatestResult()
            if photonResult.hasTargets():  # If we have any targets
                cameraToTargets += [
                    (target.getFiducialId(), target.getBestCameraToTarget())
                    for target in photonResult.getTargets()
                    if target.getPoseAmbiguity() < 0.4
                ]

            if cameraToTargets != 0:  # If we have any targets
                derivedRobotPoses += [
                    (
                        self.april_tags[tag_id] + point.inverse() + camera[1].inverse(),
                        Timer.getFPGATimestamp(),
                    )
                    for tag_id, point in cameraToTargets
                ]

        return derivedRobotPoses
