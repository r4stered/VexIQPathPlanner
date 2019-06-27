# VEX IQ Python-Project
import sys
import vexiq
import math
import timer

#region config
leftMotor  = vexiq.Motor(1)
rightMotor = vexiq.Motor(2)
gyro       = vexiq.Gyro(3)
#endregion config

import math

class PathPlanner:
  originalPath = None
  nodeOnlyPath = None
  smoothPath = None
  leftPath = None
  rightPath = None
  
  origCenterVelocity = None
  origRightVelocity = None
  origLeftVelocity = None
  
  smoothCenterVelocity = None
  smoothRightVelocity = None
  smoothLeftVelocity = None
  
  accumHeading = None
  
  totalTime = 0
  totalDistance = 0
  numFinalPoints = 0
  
  pathAlpha = 0.7
  pathBeta = 0.3
  pathTolerance = 0.0001
  
  velocityAlpha = 0.1
  velocityBeta = 0.3
  velocityTolerance = 0.0001
  
  
  def __init__(self, path):
    self.originalPath = [row[:] for row in path]

  def printArr(self, path):
    print("X:, Y:")
    for val in path:
      print(val)

  def print2DArr(self, path):
    print("X:, Y:")
    for val in path:
      print(str(val[0]) + "," + str(val[1]))

  def inject(self, orig, numToInject):
    lengthOfArray = len(orig) + ((numToInject)*(len(orig)-1))
    morePoints = [[0 for w in range(2)] for h in range(lengthOfArray)]
    index = 0

    for i in range(len(orig)-1):
      morePoints[index][0] = orig[i][0]
      morePoints[index][1] = orig[i][1]
      index = index + 1
      for j in range(1, numToInject + 1):
        morePoints[index][0] = j * ((orig[i+1][0]-orig[i][0])/(numToInject+1)) + orig[i][0]
        morePoints[index][1] = j * ((orig[i+1][1]-orig[i][1])/(numToInject+1)) + orig[i][1]
        index = index + 1

    morePoints[index][0] = orig[-1][0]
    morePoints[index][1] = orig[-1][1]
    index = index + 1

    return morePoints

  def smoother(self, path, weight_data, weight_smooth, tolerance):
    newPath = [row[:] for row in path]
    change = tolerance
    while change >= tolerance:
      change = 0.0
      for i in range(1, len(path)-1):
        for j in range(len(path[i])):
          aux = newPath[i][j]
          newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]))
          change += abs(aux - newPath[i][j])

    return newPath

  def nodeOnlyWaypoints(self, path):
    li = []
    li.append(path[0])

    for i in range(1, len(path) - 1):
      vector1 = math.atan2((path[i][1]-path[i-1][1]), path[i][0] - path[i-1][0])
      vector2 = math.atan2((path[i+1][1]-path[i][1]), path[i+1][0] - path[i][0])

      if abs(vector2-vector1) >= 0.01:
        li.append(path[i])

    li.append(path[-1])

    return li

  def velocity(self, smoothPath, timestep):
    dxdt = [None] * len(smoothPath);
    dydt = [None] * len(smoothPath);
    velocity = [[0 for w in range(2)] for h in range(len(smoothPath))]

    dxdt[0] = 0
    dydt[0] = 0
    velocity[0][0] = 0
    velocity[0][1] = 0
    self.accumHeading[0][1] = 0

    for i in range(1, len(smoothPath)):
      dxdt[i] = (smoothPath[i][0]-smoothPath[i-1][0])/timestep
      dydt[i] = (smoothPath[i][1]-smoothPath[i-1][1])/timestep

      velocity[i][0] = velocity[i-1][0] + timestep
      self.accumHeading[i][0] = self.accumHeading[i-1][0] + timestep

      velocity[i][1] = math.sqrt(math.pow(dxdt[i], 2) + math.pow(dydt[i], 2))

    return velocity

  def velocityFix(self, smoothVelocity, origVelocity, tolerance):
    difference = self.errorSum(origVelocity, smoothVelocity)
    fixVel = [[0 for w in range(2)] for h in range(len(smoothVelocity))]

    for i in range(len(smoothVelocity)):
      fixVel[i][0] = smoothVelocity[i][0]
      fixVel[i][1] = smoothVelocity[i][1]

    increase = 0
    while abs(difference[-1]) > tolerance:
      increase = difference[-1]/1/50
      for i in range(1, len(fixVel)):
        fixVel[i][1] = fixVel[i][1] - increase

      difference = self.errorSum(origVelocity, fixVel)

    return fixVel
  
  def errorSum(self, origVelocity, smoothVelocity):
    tempOrigDist = [None] * len(origVelocity)
    tempSmoothDist = [None] * len(smoothVelocity)
    difference = [None ] * len(smoothVelocity)
    timeStep = origVelocity[1][0] - origVelocity[0][0]

    tempOrigDist[0] = origVelocity[0][1]
    tempSmoothDist[0] = smoothVelocity[0][1]

    for i in range(1, len(origVelocity)):
      tempOrigDist[i] = origVelocity[i][1] * timeStep + tempOrigDist[i-1]
      tempSmoothDist[i] = smoothVelocity[i][1] * timeStep + tempSmoothDist[i-1]
      difference[i] = tempSmoothDist[i] - tempOrigDist[i]

    return difference

  def injectionCounter2Step(self, numNodeOnlyPoints, maxTimeToCompelete, timestep):
    first = 0
    second = 0
    third = 0
    oldPointsTotal = 0
    numFinalPoints = 0
    ret = []
    totalPoints = maxTimeToCompelete / timestep

    if totalPoints < 100:
      pointsFirst = 0
      pointsTotal = 0

      for i in range(4, 7):
        for j in range(1, 9):
          pointsFirst = i * (numNodeOnlyPoints-1) + numNodeOnlyPoints
          pointsTotal = (j*(pointsFirst-1)+pointsFirst)
          if pointsTotal <= totalPoints and pointsTotal > oldPointsTotal:
            first = i
            second = j
            numFinalPoints = pointsTotal
            oldPointsTotal = pointsTotal

      ret.append(first)
      ret.append(second)
      ret.append(third)
    else:
      pointsFirst = 0
      pointsSecond = 0
      pointsTotal = 0

      for i in range(1, 6):
        for j in range(1, 9):
          for k in range(1, 8):
            pointsFirst = i * (numNodeOnlyPoints-1) + numNodeOnlyPoints;
            pointsSecond = (j * (pointsFirst - 1) + pointsFirst)
            pointsTotal = (k * (pointsSecond - 1) + pointsSecond)

            if pointsTotal <= totalPoints:
              first = i
              second = j
              third = k
              numFinalPoints = pointsTotal

      ret.append(first)
      ret.append(second)
      ret.append(third)

    return ret

  def leftRight(self, smoothPath, robotTrackWidth):
    leftPath = [[0 for w in range(2)] for h in range(len(smoothPath))]
    rightPath = [[0 for w in range(2)] for h in range(len(smoothPath))]
    gradient = [[0 for w in range(2)] for h in range(len(smoothPath))]

    for i in range(len(smoothPath) - 1):
      gradient[i][1] = math.atan2(smoothPath[i+1][1] - smoothPath[i][1], smoothPath[i+1][0] - smoothPath[i][0])

    gradient[-1][1] = gradient[-2][1]

    for i in range(len(gradient)):
      leftPath[i][0] = (robotTrackWidth / 2 * math.cos(gradient[i][1] + math.pi/2)) + smoothPath[i][0]
      leftPath[i][1] = (robotTrackWidth / 2 * math.sin(gradient[i][1] + math.pi/2)) + smoothPath[i][1]

      rightPath[i][0] = (robotTrackWidth / 2 * math.cos(gradient[i][1] - math.pi/2)) + smoothPath[i][0]
      rightPath[i][1] = (robotTrackWidth / 2 * math.sin(gradient[i][1] - math.pi/2)) + smoothPath[i][1]

      deg = math.degrees(gradient[i][1])
      gradient[i][1] = deg

      if i > 0:
        if deg-gradient[i-1][1] > 180:
          gradient[i][1] = -360+deg
        if deg-gradient[i-1][1] < -180:
          gradient[i][1] = 360+deg

    self.accumHeading = gradient
    self.leftPath = leftPath
    self.rightPath = rightPath

  def calculate(self, totalTime, timestep, robotTrackWidth):
    print("hello")
    self.nodeOnlyPath = self.nodeOnlyWaypoints(self.originalPath)
    inject = self.injectionCounter2Step(len(self.nodeOnlyPath), totalTime, timestep)
    print("got past injection counter")
    for i in range(len(inject)):
      if i == 0:
        self.smoothPath = self.inject(self.nodeOnlyPath, inject[0])
        self.smoothPath = self.smoother(self.smoothPath, self.pathAlpha, self.pathBeta, self.pathTolerance)
      else:
        self.smoothPath = self.inject(self.smoothPath, inject[i])
        self.smoothPath = self.smoother(self.smoothPath, 0.1, 0.3, 0.0001)

    print("got past inject and smooth")

    self.leftRight(self.smoothPath, robotTrackWidth)

    self.origCenterVelocity = self.velocity(self.smoothPath, timestep)
    self.origLeftVelocity = self.velocity(self.leftPath, timestep)
    self.origRightVelocity = self.velocity(self.rightPath, timestep)

    self.smoothCenterVelocity = [row[:] for row in self.origCenterVelocity]
    self.smoothLeftVelocity = [row[:] for row in self.origLeftVelocity]
    self.smoothRightVelocity = [row[:] for row in self.origRightVelocity]

    self.smoothCenterVelocity[-1][1] = 0.0
    self.smoothLeftVelocity[-1][1] = 0.0
    self.smoothRightVelocity[-1][1] = 0.0

    self.smoothCenterVelocity = self.smoother(self.smoothCenterVelocity, self.velocityAlpha, self.velocityBeta, self.velocityTolerance)
    self.smoothLeftVelocity = self.smoother(self.smoothLeftVelocity, self.velocityAlpha,self.velocityBeta, self.velocityTolerance)    
    self.smoothRightVelocity = self.smoother(self.smoothRightVelocity, self.velocityAlpha, self.velocityBeta, self.velocityTolerance)

    self.smoothCenterVelocity = self.velocityFix(self.smoothCenterVelocity, self.origCenterVelocity, 0.0001)
    self.smoothLeftVelocity = self.velocityFix(self.smoothLeftVelocity, self.origLeftVelocity, 0.0001)
    self.smoothRightVelocity = self.velocityFix(self.smoothRightVelocity, self.origRightVelocity, 0.0001)
    
    print("done")


waypoints = []
waypoints.append([7,16])
waypoints.append([11,16])
waypoints.append([17,28])
waypoints.append([23,28])


totalTime = 4
timestep = 0.1
robotTrackWidth = 7

planner = PathPlanner(waypoints)

planner.calculate(totalTime, timestep, robotTrackWidth)

print("Smooth Path")
planner.print2DArr(planner.smoothPath)

print("Velocity Path")
planner.print2DArr(planner.smoothCenterVelocity)

print("Left Velocity Path")
planner.print2DArr(planner.smoothLeftVelocity)

print("Right Velocity Path")
planner.print2DArr(planner.smoothRightVelocity)

timeCounter = timer.Timer()
timeCounter.start()

wheel_radius = 1.25319
wheel_radius_meters = wheel_radius * .0254

i = 0

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


while(True):
    currentTime = timeCounter.elapsed_time()
    if currentTime >= 0.1:
        if i < len(planner.smoothRightVelocity):
            
            leftMotorMPS = planner.smoothLeftVelocity[i][1] * 0.0254
            rightMotorMPS = planner.smoothRightVelocity[i][1] * 0.0254
            
            leftmotorRadiansPerSecond = leftMotorMPS / wheel_radius_meters
            rightmotorRadiansPerSecond = rightMotorMPS / wheel_radius_meters
            
            leftMotorRPM = leftmotorRadiansPerSecond * 9.549
            rightMotorRPM = rightmotorRadiansPerSecond * 9.549
            
        
            leftMotorSpeed = translate(leftMotorRPM, -120, 120, -100, 100)
            rightMotorSpeed = translate(rightMotorRPM, -120, 120, -100, 100)
    
            leftMotor.run(-leftMotorSpeed)
            rightMotor.run(rightMotorSpeed)
        else:
            leftMotor.run(0)
            rightMotor.run(0)
        
        timeCounter.reset()
        timeCounter.start()
        i = i + 1
