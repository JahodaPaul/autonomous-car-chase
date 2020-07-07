import numpy as np
import math

class SemanticSegmentation:
    def __init__(self):
        self.counter = 0
        self.imageWidth = 0
        self.imageHeight = 0

        self.CoordRectangles = [] #y,x

        self.lastN = 5
        self.lastNX = []
        self.lastNY = []
        self.exponentialMovingAverageX = 0
        self.exponentialMovingAverageY = 0
        self.alpha = 0.5

        self.size_w = 0
        self.size_h = 0

        self.bboxInARow = 0

    def EuclidianDistance(self,x1,x2,y1,y2):
        return math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))


    def BresenhamLineSample(self,arr, k):
        if k >= len(arr):
            return arr
        else:
            x0 = 0
            x1 = k - 1
            y0 = 0
            y1 = len(arr) - (k + 1)

        dx = x1 - x0
        dy = abs(y1 - y0)
        D = 2 * dy - dx
        y = y0
        res = []
        counter = 0

        for x in range(x0, x1 + 1):
            res.append(arr[counter])
            counter += 1
            while D > 0 and (x != x1 or y != y1):
                y = y + (1 if y1 >= y0 else -1)
                counter += 1
                D = D - 2 * dx
            D = D + 2 * dy
        return res

    def BresenhamLine(self,x0, y0, x1, y1):
        counter = 0
        if x0 > x1:
            tmpX = x1
            tmpY = y1
            x1 = x0
            x0 = tmpX
            y1 = y0
            y0 = tmpY

        coords = []
        dx = x1 - x0
        dy = abs(y1 - y0)
        D = 2 * dy - dx
        y = y0

        for x in range(x0, x1 + 1):
            # if counter%3==0:
            coords.append([x, y])
            counter += 1
            while D > 0 and (x!=x1 or y != y1):
                y = y + (1 if y1 >= y0 else -1)
                # if counter % 3 == 0:
                coords.append([x, y])
                counter += 1
                D = D - 2 * dx
            D = D + 2 * dy
        return coords

    def LimitAngles(self,angle):
        return min(max(angle,-175),175)

    def GetPercentage(self,middleX,xCoord, currentPredictedX, otherSide=False):
        overallDistX = abs(currentPredictedX - middleX)
        distFromMiddle = abs(middleX - xCoord)
        percentage = distFromMiddle / overallDistX
        # percentage = percentage * percentage
        if otherSide:
            percentage = -1 * percentage
        return percentage

    def FindClosestRect(self,x,y):
        smallestVal = 10000000
        smallestIndex = 0
        for i in range(len(self.CoordRectangles)):
            if abs(self.CoordRectangles[i][0] - y) + abs(self.CoordRectangles[i][1] - x) < smallestVal:
                smallestVal = abs(self.CoordRectangles[i][0] - y) + abs(self.CoordRectangles[i][1] - x)
                smallestIndex = i
        return smallestIndex

    def FindPossibleAngle(self,bbox,maxAngle,drivableIndexes,width,height):
        x_Middle = 0
        y_Middle = 0

        n_cols = 10
        n_rows = 10
        self.size_w = width // n_cols
        self.size_h = height // n_rows

        rows = height // self.size_h
        cols = width // self.size_w
        if len(self.CoordRectangles) == 0:
            for i in range(rows):
                for j in range(cols):
                    self.CoordRectangles.append([i * self.size_h + (self.size_h//2),j * self.size_w + (self.size_w//2)])


        if len(bbox) != 0:
            # print(bbox)
            # points = [[int(bbox[i, 0]), int(bbox[i, 1])] for i in range(len(bbox))]
            x_Middle = (bbox[0] + bbox[2]) // 2
            y_Middle = int(bbox[3]) # max
            self.lastNX.append(x_Middle)
            self.lastNY.append(y_Middle)
            if len(self.lastNX) > self.lastN:
                self.lastNX = self.lastNX[1:]
                self.lastNY = self.lastNY[1:]

            if self.bboxInARow == 0:
                self.bboxInARow = 1
                self.lastNX.append(x_Middle)
                self.lastNY.append(y_Middle)
                if len(self.lastNX) > self.lastN:
                    self.lastNX = self.lastNX[1:]
                    self.lastNY = self.lastNY[1:]

            alpha = self.alpha if len(self.lastNX) > 1 else 1
            self.exponentialMovingAverageX = alpha * x_Middle + (1 - alpha) * self.exponentialMovingAverageX
            self.exponentialMovingAverageY = alpha * y_Middle + (1 - alpha) * self.exponentialMovingAverageY

        self.imageHeight = height
        self.imageWidth = width

        if self.counter >= 30:
            #Find bounding box
            if len(bbox) != 0:
                x_Middle = (bbox[0] + bbox[2]) // 2
                y_Middle = int(bbox[3])  # max
            else:
                if len(self.lastNX) >= 2:
                    self.bboxInARow = 0
                    x_Middle = 2 * self.lastNX[-1] - self.lastNX[-2]  # simple extrapolation
                    y_Middle = 2 * self.lastNY[-1] - self.lastNY[-2]  # simple extrapolation

                    self.exponentialMovingAverageX = self.alpha * x_Middle + (1 - self.alpha) * self.exponentialMovingAverageX
                    self.exponentialMovingAverageY = self.alpha * y_Middle + (1 - self.alpha) * self.exponentialMovingAverageY

                    self.lastNX.append(x_Middle)
                    self.lastNY.append(y_Middle)
                    if len(self.lastNX) > self.lastN:
                        self.lastNX = self.lastNX[1:]
                        self.lastNY = self.lastNY[1:]
                    x_Middle = self.exponentialMovingAverageX
                    y_Middle = self.exponentialMovingAverageY
            # y_Middle += self.size_h

            closestRectIndex = self.FindClosestRect(x_Middle, y_Middle)
            tmp = closestRectIndex%10

            coords = self.BresenhamLine(self.imageWidth // 2, self.imageHeight - 1, self.CoordRectangles[closestRectIndex][1], self.CoordRectangles[closestRectIndex][0])
            coords = self.BresenhamLineSample(coords,8)
            possible = True
            for i in range(len(coords)):
                closestRectIndex = self.FindClosestRect(coords[i][0], coords[i][1])
                if drivableIndexes[closestRectIndex] == 0:
                    possible = False
            if possible:
                # Can drive straight
                return maxAngle, drivableIndexes
            else:
                # Need to find another path
                closestRectIndex = self.FindClosestRect(x_Middle, y_Middle)
                line = closestRectIndex//10 #TODO if the number of rectangles changes
                if line == 9:
                    return 0, drivableIndexes
                drivability = []
                closeness = []

                goodnessScore = []
                mostDrivableIndex = 0
                minn = 0; maxx = 10
                if tmp < 4:
                    minn = tmp
                elif tmp > 5:
                    maxx = tmp
                for j in range(minn,maxx):
                    closestRectIndex = line*10+j
                    coords = self.BresenhamLine(self.imageWidth // 2, self.imageHeight - 1,self.CoordRectangles[closestRectIndex][1],self.CoordRectangles[closestRectIndex][0])
                    coords = self.BresenhamLineSample(coords, 8)


                    current = 0
                    for i in range(len(coords)):
                        closestRectIndex = self.FindClosestRect(coords[i][0], coords[i][1])
                        if drivableIndexes[closestRectIndex] == 1:
                            current += 1
                    drivability.append(current)
                    closeness.append(self.EuclidianDistance(self.CoordRectangles[closestRectIndex][1],x_Middle,self.CoordRectangles[closestRectIndex][0],y_Middle))
                closeness = np.array(closeness)/float(np.max(closeness))
                closeness = 1.0 - closeness
                for i in range(len(drivability)):
                    goodnessScore.append(closeness[i]+float(drivability[i]))
                mostDrivableIndex = line*10 + minn+np.argmax(goodnessScore)

                percentage = self.GetPercentage(self.imageWidth//2,self.CoordRectangles[mostDrivableIndex][1],x_Middle)
                # Trick to see if the drivable x coordinate and extrapoled X coordinate are on the same side of the image
                if (self.CoordRectangles[mostDrivableIndex][1] - self.imageWidth//2) * (x_Middle - self.imageWidth//2) < 0:
                    percentage = percentage*-1
                return self.LimitAngles(maxAngle*percentage), drivableIndexes
        else:
            self.counter += 1
            return self.LimitAngles(maxAngle), []
        return self.LimitAngles(maxAngle), drivableIndexes