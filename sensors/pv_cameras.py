import math

from networktables import NetworkTables
import constants
from wpilib import Timer
from robotpy_toolkit_7407.utils.units import m, deg, ft, inch, rad, radians, meters
from photonvision import PhotonCamera
from robotpy_toolkit_7407.sensors.odometry import VisionEstimator


class PV_Cameras(VisionEstimator):
    def __init__(self):
        super().__init__()
        self.cameras = [
            (PhotonCamera(cameraName=camera_name), constants.kCameras[camera_name])  # (camera_object, camera_tranform)
            for camera_name in constants.kCameras
        ]

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
                    (constants.kApriltagPositionDict[tag_id]
                     + point.inverse()
                     + camera[1].inverse(), Timer.getFPGATimestamp())
                    for tag_id, point in cameraToTargets
                ]

        return derivedRobotPoses
