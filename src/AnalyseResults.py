import pickle
import os
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from CarDetector import CarDetector

class AnalyseResults:
    def __init__(self,dirChasing):
        self.dirChasing = dirChasing

    def loadPositionHistory(self, fileName):
        return pickle.load(open(fileName, "rb"))

    def EuclidianDistance(self,x1,x2,y1,y2,z1,z2):
        return math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))

    def LoadResults(self):
        file = self.dirChasing + '/results.txt'
        with open(file,'r') as f:
            lines = [line.rstrip() for line in f]
        return lines

    def FinishedPercentage(self, historyChased, historyChasing):
        lastPosChasing = historyChasing[-1]

        nOfPositionsDriven = 0
        tmp = 0
        for i in range(1,len(historyChasing)):
            m = len(historyChasing) - 1
            d = self.EuclidianDistance(lastPosChasing[0], historyChasing[m-i][0], lastPosChasing[1], historyChasing[m-i][1],
                                   lastPosChasing[2], historyChasing[m-i][2])
            if d > 2:
                break
            else:
                tmp +=1
        nOfPositionsDriven = len(historyChased) - tmp
        print(tmp)

        # find shortest position from the last path to the path
        shortestDist = 1000000000
        shortestIndex = 0
        for i in range(nOfPositionsDriven-1,-1,-1):
            dist = self.EuclidianDistance(lastPosChasing[0],historyChased[i][0],lastPosChasing[1],historyChased[i][1],lastPosChasing[2],historyChased[i][2])
            if dist < shortestDist:
                shortestDist = dist
                shortestIndex = i
            else:
                break
            # if dist > 2*shortestDist:
            #     break
        # shortestIndex = nOfPositionsDriven

        distToLastPoint = 0
        distfromLastPoint = 0

        for i in range(len(historyChased)):
            if i+1 < len(historyChased):
                if i+1 <= shortestIndex:
                    distToLastPoint += self.EuclidianDistance(historyChased[i+1][0],historyChased[i][0],historyChased[i+1][1],historyChased[i][1],historyChased[i+1][2],historyChased[i][2])
                else:
                    distfromLastPoint += self.EuclidianDistance(historyChased[i + 1][0], historyChased[i][0],
                                                              historyChased[i + 1][1], historyChased[i][1],
                                                              historyChased[i + 1][2], historyChased[i][2])
        percentage = distToLastPoint/float(distToLastPoint+distfromLastPoint)
        return percentage

    def FindMAE(self,drivename):
        results = self.LoadResults()
        for i in range(len(results)):
            tmp = results[i].split(',')
            if drivename in tmp[0]:
                return float(tmp[1]), float(tmp[2]), float(tmp[3])

def Analyse(dirChasing):
    dirChasing = dirChasing
    dirChased = '../drives'
    analyse = AnalyseResults(dirChasing)

    # chasedFiles = os.listdir(dirChased)
    # chasedFiles = ['ride11.p', 'ride12.p', 'ride13.p', 'ride14.p', 'ride15.p', 'ride16.p', 'ride17.p', 'ride18.p','ride19.p', 'ride20.p']
    # chasedFiles = ['ride1.p','ride2.p','ride3.p','ride4.p','ride5.p','ride6.p','ride7.p','ride8.p','ride9.p','ride10.p']
    chasedFiles = ['ride1.p', 'ride2.p', 'ride3.p', 'ride4.p', 'ride5.p', 'ride6.p', 'ride7.p', 'ride8.p', 'ride9.p',
                  'ride10.p',
                   'ride11.p', 'ride12.p', 'ride13.p', 'ride14.p', 'ride15.p', 'ride16.p', 'ride17.p', 'ride18.p',
                   'ride19.p', 'ride20.p']
    finished = 0
    percentages = []
    maes = [];
    rmses = [];
    crashes = []

    for file in chasedFiles:
        historyChased = analyse.loadPositionHistory(os.path.join(dirChased, file))
        historyChasing = analyse.loadPositionHistory(os.path.join(dirChasing, file))
        percentage = analyse.FinishedPercentage(historyChased, historyChasing)
        if percentage >= 0.94:
            finished += 1
            mae, rmse, crash = analyse.FindMAE(file)
            # print(mae, rmse, crash)
            maes.append(mae)
            rmses.append(rmse)
            crashes.append(crash)

        percentages.append(percentage)
        # print(file,':',percentage)
    print('Number of finished drives:', finished)
    print('Average percentage of finished drives:', np.mean(percentages))
    # print('Average number of crashes:',np.mean(crashes))
    # print('Average number MAE:', np.mean(maes))
    # print('Average number RMSE:',np.mean(rmses))

    return np.mean(percentages)

def AverageSpeed():
    dirChased = '../drives'
    chasedFiles = ['ride1.p', 'ride2.p', 'ride3.p', 'ride4.p', 'ride5.p', 'ride6.p', 'ride7.p', 'ride8.p', 'ride9.p',
                   'ride10.p',
                   'ride11.p', 'ride12.p', 'ride13.p', 'ride14.p', 'ride15.p', 'ride16.p', 'ride17.p', 'ride18.p',
                   'ride19.p', 'ride20.p']

    distances = []
    seconds = []
    analyse = AnalyseResults(dirChased)
    speeds = []
    for file in chasedFiles:
        historyChased = analyse.loadPositionHistory(os.path.join(dirChased, file))
        while(0.001 > analyse.EuclidianDistance(historyChased[0][0],historyChased[1][0],historyChased[0][1],
                                                        historyChased[1][1],0,0)):
            historyChased = historyChased[1:]

        while(0.001 > analyse.EuclidianDistance(historyChased[-1][0],historyChased[-2][0],historyChased[-1][1],
                                                        historyChased[-2][1],0,0)):
            historyChased = historyChased[:-1]

        totalDistMeters = 0
        totalNumberOfSeconds = 0
        for i in range(len(historyChased)-1):
            totalDistMeters += analyse.EuclidianDistance(historyChased[i][0],historyChased[i+1][0],historyChased[i][1],
                                                        historyChased[i+1][1],historyChased[i][2],historyChased[i+1][2])
        totalNumberOfSeconds = float(len(historyChased))/30.0
        distances.append(totalDistMeters)
        seconds.append(totalNumberOfSeconds)
        metersPerSecond = totalDistMeters/float(totalNumberOfSeconds)
        kmPerHour = metersPerSecond * 3.6
        speeds.append(kmPerHour)
    print(speeds)
    speeds = np.array(speeds,dtype=float)
    print('Average distance:', np.mean(distances))
    print('Average time seconds:', np.mean(seconds))

    print('Median distance:', np.max(distances))
    print('median time seconds:', np.max(seconds))
    print('AverageSpeed:',np.mean(speeds))
    print('AverageSpeed difficult:',np.mean(speeds[:10]))
    print('AverageSpeed easy:', np.mean(speeds[10:]))
    print(speeds[:10])
    X = [0 for i in range(20)]
    colors = ['#0B6623' for i in range(20)]
    for i in range(10):
        X[i] = 1
        colors[i] = '#0065BD'
    plt.rc('axes', labelsize=18)
    plt.scatter(X, speeds,color=colors,s=100,alpha=0.6)

    plt.ylabel('Avg. driving speed km/h')
    plt.yticks(fontsize=14)
    plt.xticks([-0.3,0,1,1.3],['','easy drives','difficult drives',''],fontsize=18)
    plt.savefig("speed5.pdf", bbox_inches='tight')
    plt.show()



def PlotTrajectory():
    img = plt.imread("carla_view_h.png")
    fig, ax = plt.subplots()
    ax.imshow(img,extent=[ -220, 220,-95, 250])
    dirChased = '../drives'
    dirChasing = '../Experiment1/chaseOutput_Advanced_DifficultDrives'
    chasedFiles = ['ride7.p']
    analyse = AnalyseResults(dirChased)
    speeds = []
    for file in chasedFiles:
        historyChased = analyse.loadPositionHistory(os.path.join(dirChased, file))
        historyChasing = analyse.loadPositionHistory(os.path.join(dirChasing, file))

    historyChasing = np.array(historyChasing)
    historyChased = np.array(historyChased)
    print(len(historyChasing),len(historyChased))
    plt.rc('axes', labelsize=26)

    plt.plot(historyChasing[:,1],historyChasing[:,0],color='#0065BD',linewidth=4,alpha=0.7) # #0065BD
    plt.plot(historyChased[:,1],historyChased[:,0],color='black',linewidth=3,alpha=0.7,linestyle='--',dashes=(2, 1))
    # plt.scatter(X, speeds, color=colors, s=100)

    plt.ylabel('X [m]')
    plt.xlabel('Y [m]')
    # plt.yticks(fontsize=14)
    # plt.xticks([-0.3, 0, 1, 1.3], ['', 'easy drives', 'difficult drives', ''], fontsize=18)
    plt.gca().set_aspect("equal")
    plt.savefig("trajectory_img_h.pdf", bbox_inches='tight')
    # plt.show(img)

def PlotPrubehy():
    dirChased = '../drives'
    dirChasing = '../Experiment1/chaseOutput_Advanced_DifficultDrives'
    chasedFiles = ['ride7.p']
    analyse = AnalyseResults(dirChased)
    distances = []


    for file in chasedFiles:
        historyChased = analyse.loadPositionHistory(os.path.join(dirChased, file))
        historyChasing = analyse.loadPositionHistory(os.path.join(dirChasing, file))

    historyChasing = np.array(historyChasing)
    historyChased = np.array(historyChased)

    for i in range(len(historyChased)-1):
        dist = analyse.EuclidianDistance(historyChased[i][0], historyChasing[i][0], historyChased[i][1],
                                         historyChasing[i][1], historyChased[i][2], historyChasing[i][2])
        distances.append(dist)

    plt.rc('axes', labelsize=18)
    plt.ylabel('Distance [m]')
    plt.xlabel('Time [s]')
    plt.yticks(fontsize=14)
    plt.xticks([0,600,1200,1800,2400,3000,3600],[0,600//30,1200//30,1800//30,2400//30,3000//30,3600//30],fontsize=18)
    plt.plot(distances, color='#0065BD')
    plt.savefig("distances.pdf", bbox_inches='tight')
    plt.show()

    speeds1 = []
    speeds2 = []
    for i in range(2,len(historyChased)-2):
        dist = analyse.EuclidianDistance(historyChased[i][0], historyChased[i+1][0], historyChased[i][1],
                                         historyChased[i+1][1], historyChased[i][2], historyChased[i+1][2])

        dist2 = analyse.EuclidianDistance(historyChasing[i][0], historyChasing[i+1][0], historyChasing[i][1],
                                         historyChasing[i+1][1], historyChasing[i][2], historyChasing[i+1][2])
        speeds1.append(dist*30*3.6)
        speeds2.append(dist2*30*3.6)

    plt.rc('axes', labelsize=18)
    plt.ylabel('Speed [km/h]')
    plt.xlabel('Time [s]')
    plt.yticks(fontsize=14)
    plt.xticks([0, 600, 1200, 1800, 2400, 3000, 3600],
               [0, 600 // 30, 1200 // 30, 1800 // 30, 2400 // 30, 3000 // 30, 3600 // 30], fontsize=18)
    plt.plot(speeds2, color='#0065BD')
    plt.plot(speeds1, color='black')
    patchesList = []
    patchesList.append(mpatches.Patch(color='#0065BD', label='Chasing car'))
    patchesList.append(mpatches.Patch(color='black', label='Chased car'))
    plt.legend(handles=patchesList)
    plt.savefig("speeds.pdf", bbox_inches='tight')
    plt.show()

    carDetector = CarDetector()
    angles = []
    for i in range(2,len(historyChased)-2):
        angle = carDetector.getAngle([historyChasing[i][0], historyChasing[i][1]], [historyChasing[i+1][0],
                                         historyChasing[i+1][1]], [historyChased[i][0], historyChasing[i][1]])
        angles.append(angle)

    plt.rc('axes', labelsize=18)
    plt.ylabel('Angle [degrees]')
    plt.xlabel('Time [s]')
    plt.yticks(fontsize=14)
    plt.xticks([0,600,1200,1800,2400,3000,3600],[0,600//30,1200//30,1800//30,2400//30,3000//30,3600//30],fontsize=18)
    plt.plot(angles, color='#0065BD')
    plt.savefig("angles.pdf", bbox_inches='tight')
    plt.show()


    carDetector = CarDetector()
    yawrate = []
    yawrate2 = []
    for i in range(2,len(historyChased)-3):
        angle = carDetector.getAngle([0,0], [historyChasing[i+1][0] - historyChasing[i][0], historyChasing[i+1][1] - historyChasing[i][1]],
                                     [historyChasing[i+2][0] - historyChasing[i+1][0], historyChasing[i+2][1] - historyChasing[i+1][1]])
        if abs(angle*30) > 200:
            angle = yawrate[-1]
        yawrate.append(angle*30)

        angle = carDetector.getAngle([0,0], [historyChased[i+1][0] - historyChased[i][0], historyChased[i+1][1] - historyChased[i][1]],
                                     [historyChased[i+2][0] - historyChased[i+1][0], historyChased[i+2][1] - historyChased[i+1][1]])
        if abs(angle*30) > 200:
            angle = yawrate2[-1]
        yawrate2.append(angle*30)

    plt.rc('axes', labelsize=18)
    plt.ylabel('Yaw rate [degrees/s]')
    plt.xlabel('Time [s]')
    plt.yticks(fontsize=14)
    plt.xticks([0,600,1200,1800,2400,3000,3600],[0,600//30,1200//30,1800//30,2400//30,3000//30,3600//30],fontsize=18)
    plt.plot(yawrate, color='#0065BD')
    plt.plot(yawrate2, color='black')
    plt.legend(handles=patchesList)
    plt.savefig("yaw_rate.pdf", bbox_inches='tight')
    plt.show()



def main():
    # PlotPrubehy()
    # exit()
    # PlotTrajectory()
    # exit()
    AverageSpeed()
    exit()

    finishedPercentagesWOSE = []
    finishedPercentagesFull = []
    finishedPercentagesWOS = []

    fromm = 0
    until = 101

    for i in range(fromm,until,5):
        finishedPercentagesWOSE.append(Analyse('../Experiment2_NoSegmEx/chaseOutput'+str(i)))
        finishedPercentagesFull.append(Analyse('../Experiment2_Full/chaseOutput'+str(i)))
        finishedPercentagesWOS.append(Analyse('../Experiment2_NoSegm/chaseOutput'+str(i)))


    plt.figure(figsize=(16, 9), dpi=1200, facecolor='w', edgecolor='k')
    plt.rc('axes', titlesize=32)
    plt.rc('axes', labelsize=32)
    plt.rcParams["legend.fontsize"] = 26
    plt.yticks(ticks=[0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8],fontsize=22)
    plt.xticks(ticks=[i for i in range(0,101,10)],fontsize=22)
    X = np.arange(fromm,until,5)

    plt.plot(X, finishedPercentagesWOSE, color='black',linewidth=3)
    plt.plot(X, finishedPercentagesFull, color='#0065BD',linewidth=3)
    plt.plot(X, finishedPercentagesWOS, color='purple',linewidth=3)
    plt.xlabel('Detection fails per 100 frames')
    plt.ylabel('Average drive completion')
    #plt.title('Drive performance based on detection recall')
    patchesList = []
    patchesList.append(mpatches.Patch(color='#0065BD', label='Full Algorithm'))
    patchesList.append(mpatches.Patch(color='purple', label='W/o Segmentation'))
    patchesList.append(mpatches.Patch(color='black', label='W/o Segmentation + Extrapolation'))
    plt.legend(handles=patchesList)
    plt.grid(True,dashes=(5,5))
    plt.savefig("recall_chart7.pdf", bbox_inches='tight',dpi=1200)
    plt.show()




if __name__ == '__main__':
    main()
