import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy import interpolate
from car import Car
from spring import spring
from collections import deque

car = Car()
zWheelDatum = car.lowestPoint - car.groundClearance + car.wheelRadius

fig, ax = plt.subplots()
chassis, = ax.plot(car.chassisCoords[0,:], car.chassisCoords[1,:],
    c='k', lw=2)
chassisCOG, = ax.plot(0, 0, c='k', marker='o')
frontTire, = ax.plot(car.wheel[0,:] + car.l_f,
    car.wheel[1,:] + zWheelDatum, c='g', lw=2, zorder=2)
frontHub, = ax.plot(car.hub[0,:] + car.l_f,
    car.hub[1,:] + zWheelDatum, c='g', lw=2, zorder=2)
rearTire, = ax.plot(car.wheel[0,:] - car.l_r,
    car.wheel[1,:] + zWheelDatum, c='b', lw=2)
rearHub, = ax.plot(car.hub[0,:] - car.l_r,
    car.hub[1,:] + zWheelDatum, c='b', lw=2)
frontHubLine, = ax.plot([car.l_f, car.l_f + car.hubRadius],
    [zWheelDatum, zWheelDatum], c='g', lw=1)
rearHubLine, = ax.plot([-car.l_r, -car.l_r + car.hubRadius],
    [zWheelDatum, zWheelDatum], c='b', lw=1)
frontSuspension, = ax.plot([], [], c='k', lw=1, zorder=1)
rearSuspension, = ax.plot([], [], c='k', lw=1, zorder=1)
ax.set_aspect('equal')
yLimits = (-2, 2)
xLimits = (-3, 2.5)
ax.set_ylim(yLimits)
ax.set_xlim(xLimits)

roadDatum = car.lowestPoint - car.groundClearance
roadResolution = 300    # points per meter
numRoadPoints = int((xLimits[1] - xLimits[0]) * roadResolution)
xRoad = np.linspace(xLimits[0], xLimits[1], numRoadPoints)
zRoad = deque(roadDatum * np.ones((numRoadPoints,)), maxlen=numRoadPoints)
road, = ax.plot(xRoad, zRoad, c='brown', lw=1.5)
roadMarker, = ax.plot(xLimits[1], -.75, c='brown', marker='^')

timeDisp = ax.annotate('', xy=(0.01, 0.02), xycoords='axes fraction')
brakeDisp = ax.annotate('', xy=(0.20, 0.18), xycoords='axes fraction',
    color='r')
accDisp = ax.annotate('', xy=(0.20, 0.14), xycoords='axes fraction')
speedDisp = ax.annotate('', xy=(0.20, 0.10), xycoords='axes fraction')
speedDispKPH = ax.annotate('', xy=(0.267, 0.06), xycoords='axes fraction')
speedDispMPH = ax.annotate('', xy=(0.267, 0.02), xycoords='axes fraction')
distDisp = ax.annotate('', xy=(0.50, 0.10), xycoords='axes fraction')
distDispFt = ax.annotate('', xy=(0.664, 0.06), xycoords='axes fraction')
distDispMi = ax.annotate('', xy=(0.664, 0.02), xycoords='axes fraction')

def transformation_matrix(angle, xOffset, zOffset):
    T = np.array([
        [math.cos(angle), -math.sin(angle), xOffset],
        [math.sin(angle), math.cos(angle), zOffset],
        [0, 0, 1]])
    return T

def update_anim(dummyFrameArg):
    # Chassis transformations.
    z_c = car.positionVector[0,0]
    phi = car.positionVector[1,0]
    z_f = car.positionVector[2,0]
    z_r = car.positionVector[3,0]
    T = transformation_matrix(phi, 0, z_c)
    transformedChassisCoords = T.dot(car.chassisCoords)
    chassis.set_data(transformedChassisCoords[0,:],
        transformedChassisCoords[1,:])
    chassisCOG.set_ydata(z_c)

    # Suspension.
    displaySuspSprings = True
    if displaySuspSprings:
        transformedFrontWellTop = T.dot(car.frontWellTop)
        transformedRearWellTop = T.dot(car.rearWellTop)

        frontSuspension.set_data(spring(transformedFrontWellTop[:2,:],
            np.array([[car.l_f], [zWheelDatum + z_f]]), 10, .2))
        rearSuspension.set_data(spring(transformedRearWellTop[:2,:],
            np.array([[-car.l_r], [zWheelDatum + z_r]]), 10, .2))

    # Wheels.
    frontTire.set_ydata(car.wheel[1,:] + zWheelDatum + z_f)
    frontHub.set_ydata(car.hub[1,:] + zWheelDatum + z_f)
    rearTire.set_ydata(car.wheel[1,:] + zWheelDatum + z_r)
    rearHub.set_ydata(car.hub[1,:] + zWheelDatum + z_r)

    wheelAngle = car.distTraveled / car.wheelRadius
    frontHubLine.set_data(
        [car.l_f, car.l_f + car.hubRadius * math.cos(-wheelAngle)],
        [zWheelDatum + z_f, zWheelDatum + z_f + car.hubRadius * math.sin(-wheelAngle)])
    rearHubLine.set_data(
        [-car.l_r, -car.l_r + car.hubRadius * math.cos(-wheelAngle)],
        [zWheelDatum + z_r, zWheelDatum + z_r + car.hubRadius * math.sin(-wheelAngle)])

    # Road.
    road.set_ydata(zRoad)

def generate_road(lastDistance, distanceSinceLast):
    global roadResolution, zRoad, roadDatum, roadType
    numNewPoints = int(distanceSinceLast * roadResolution)
    if distanceSinceLast > 0 and numNewPoints == 0: numNewPoints = 1
    for i in range(numNewPoints):
        zRoad.popleft()
        if roadType == 'sine':
            nextPoint = roadDatum + 0.30*math.sin(
                0.04*(lastDistance + (i / roadResolution)))
        elif roadType == 'step':
            if lastDistance > 3:
                nextPoint = roadDatum + 0.20
            else:
                nextPoint = roadDatum
        elif roadType == 'flat':
            nextPoint = roadDatum
        elif roadType == 'square':
            if int(car.distTraveled) % 20 < 10:
                nextPoint = roadDatum + 0.025
            else:
                nextPoint = roadDatum
        zRoad.append(nextPoint)

saveFigs = False
timeStep = 0.0002
def recalculate_state():
    global xRoad, zRoad, timeStep
    t = 0   # elapsed time (sec)
    i = 0
    ii = 0  # figure counter (if saving figures)
    while True:
        # SET DESIRED ACCELERATION/DECELERATION HERE USING CONDITIONALS.
        # Return if desired, to signal end of animation, else loop infinitely.
        # To manually set velocity (or to set initial velocity), use
        # car.horizontalVelocity, e.g., car.horizontalVelocity = 20.
        # Car will not accelerate past max speed (which is set below).
        if t == 0:
            # Initial velocity and acceleration
            car.horizontalAccel = 3
            car.horizontalVelocity = 0
        elif 4 <= t < 5:
            car.horizontalAccel = -5
        elif 5 <= t < 12:
            car.horizontalAccel = 4
        elif 18 <= t < 18.1:
            car.horizontalAccel = -9
        elif t > 26 and car.horizontalVelocity == 0:
            return
            
        # Set max speed (in m/s) and min speed (0); reversing is not supported.
        if car.horizontalVelocity >= 60 and car.horizontalAccel > 0:
            car.horizontalVelocity = 60
            car.horizontalAccel = 0
        elif car.horizontalVelocity < 0:
            car.horizontalVelocity = 0
            car.horizontalAccel = 0
        
        car.accelerationVector = (car.stiffnessMatrix.dot(car.positionVector)
            + car.dampingMatrix.dot(car.velocityVector)
            + car.roadStiffnessMatrix.dot(car.roadPositionVector)
            + car.roadDampingMatrix.dot(car.roadVelocityVector)
            + car.normal_force_vector())

        # Clamp pitch angle to +/- 5 deg.
        if car.positionVector[1] > math.radians(5):
            car.positionVector[1] = math.radians(5)
            car.velocityVector[1] = 0
        elif car.positionVector[1] < math.radians(-5):
            car.positionVector[1] = math.radians(-5)
            car.velocityVector[1] = 0

        # Update displacements and velocities.
        car.positionVector += car.velocityVector * timeStep
        car.velocityVector += car.accelerationVector * timeStep

        # Calculate distance traveled since last iteration.
        distSinceLast = car.horizontalVelocity * timeStep

        # Generate (shift) road based on distance traveled since last iteration.
        generate_road(car.distTraveled, distSinceLast)

        # Update total distance traveled and horizontal velocity.
        car.distTraveled += distSinceLast
        car.horizontalVelocity += car.horizontalAccel * timeStep

        # Determine new road height at each contact point.
        interp = interpolate.interp1d(xRoad, zRoad - roadDatum)
        newRoadPosition = interp([car.l_f, -car.l_r]).reshape(2,1)
        car.roadVelocityVector = newRoadPosition - car.roadPositionVector
        car.roadPositionVector = newRoadPosition

        # Due to potentially large number of time steps, only update plot
        # every 100 steps to improve performance.
        if i % 100 == 0:
            if saveFigs:
                plt.savefig('./imgs/{:d}.png'.format(ii))
                ii += 1
            timeDisp.set_text('T = {:.2f} s'.format(t))
            accDisp.set_text('a = {:.2f} m/s^2'.format(car.horizontalAccel))
            speedDisp.set_text('v =  {:.2f} m/s'.format(car.horizontalVelocity))
            speedDispKPH.set_text('{:.1f} kph'.format(
                car.horizontalVelocity * 3.6))
            speedDispMPH.set_text('{:.1f} mph'.format(
                car.horizontalVelocity * 3.6 * 0.621))
            distDisp.set_text('Distance = {:.1f} m'.format(car.distTraveled))
            distDispFt.set_text(
                '{:.1f} ft'.format(car.distTraveled * 3.28))
            distDispMi.set_text(
                '{:.2f} mi'.format(car.distTraveled * 3.28 / 5280))
            if car.horizontalAccel < 0:
                brakeDisp.set_text('BRAKE')
            elif car.horizontalVelocity == 60:
                brakeDisp.set_text('MAX SPEED')
            else:
                brakeDisp.set_text('')

            xRoadMarker = xLimits[1] - car.distTraveled % 10
            if xLimits[0] <= xRoadMarker <= xLimits[1]:
                zRoadMarker = interp(xRoadMarker) + roadDatum - 0.15
            else:
                xRoadMarker = xLimits[0] - 1
                zRoadMarker = roadDatum - 0.15
            roadMarker.set_data(xRoadMarker, zRoadMarker)
            yield

        t += timeStep
        i += 1

# Define road type ('square', 'sine', or 'flat').
#roadType = 'square'
roadType = 'sine'
#roadType = 'flat'

ani = animation.FuncAnimation(fig, update_anim, frames=recalculate_state,
        interval=10, repeat=False)

saveMovie = False
if saveMovie:
    ffmpegWriterClass = animation.writers['ffmpeg']
    ffmpegWriterObj = ffmpegWriterClass(
            fps=(1/timeStep), extra_args=['-vcodec', 'h264'])
    ani.save('output.mp4', writer=ffmpegWriterObj)

plt.show()
