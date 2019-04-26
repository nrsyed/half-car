class Road:
    def __init__(self, resolution, mode="flat"):
        self.resolution = resolution
        self.mode = mode

    def 

    def __iter__(self):
        return self

    def __next__(self):
        return None


"""
roadDatum = car.lowestPoint - car.groundClearance
roadResolution = 300    # points per meter
numRoadPoints = int((xLimits[1] - xLimits[0]) * roadResolution)
xRoad = np.linspace(xLimits[0], xLimits[1], numRoadPoints)
zRoad = deque(roadDatum * np.ones((numRoadPoints,)), maxlen=numRoadPoints)
road, = ax.plot(xRoad, zRoad, c='brown', lw=1.5)
roadMarker, = ax.plot(xLimits[1], -.75, c='brown', marker='^')

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
"""
