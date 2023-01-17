import math

from networktables import NetworkTables
import constants
from wpilib import Timer
from robotpy_toolkit_7407.utils.units import m, deg, ft, inch, rad, radians, meters
from photonvision import PhotonCamera

class PV_Cameras:
    def __init__(self):
        self.cameras = [
          (PhotonCamera(cameraName=camera_name), constants.kCameras[camera_name]) #(camera_object, camera_tranform)
          for camera_name in constants.kCameras
        ]

    def getPoses(self):
      """this function returns a list of the type (target_id, transformCameraToTarget) for every target"""
      derivedRobotPoses = []
      for camera in self.cameras:
        cameraToTargets = []
        photonResult = camera.getLatestResult()
        if photonResult.hasTargets():
            cameraToTargets += [
                (target.getFiducialId(), target.getBestCameraToTarget())
                for target in photonResult.getTargets()
                if target.getPoseAmbiguity() < constants.kPhotonvisionAmbiguityCutoff
            ]
        if cameraToTargets != 0: 
          derivedRobotPoses += [
              constants.kApriltagPositionDict[tag_id]
              + point.inverse()
              + constants.kLimelightRelativeToRobotTransform.inverse()
              for tag_id, point in cameraToTargets
          ]
        
      return derivedRobotPoses


    def updatePoses(self): #Call this periodically
        poses = self.getPoses()
        if len(poses) != 0:
          for pose in poses:
            self.poseEstimator.addVisionMeasurement(
                pose.toPose2d(), Timer.getFPGATimestamp()
            )

        # estimatedPosition = self.drive.getPose()