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
        file = 'res/results.txt'
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
    dirChased = 'drives'
    analyse = AnalyseResults(dirChasing)

    # chasedFiles = os.listdir(dirChased)
    # chasedFiles = ['ride11.p', 'ride12.p', 'ride13.p', 'ride14.p', 'ride15.p', 'ride16.p', 'ride17.p', 'ride18.p','ride19.p', 'ride20.p']
    chasedFiles = ['ride1.p'] #['ride1.p','ride2.p','ride3.p','ride4.p','ride5.p','ride6.p','ride7.p','ride8.p','ride9.p','ride10.p']
    # chasedFiles = ['ride1.p', 'ride2.p', 'ride3.p', 'ride4.p', 'ride5.p', 'ride6.p', 'ride7.p', 'ride8.p', 'ride9.p',
    #               'ride10.p',
    #                'ride11.p', 'ride12.p', 'ride13.p', 'ride14.p', 'ride15.p', 'ride16.p', 'ride17.p', 'ride18.p',
    #                'ride19.p', 'ride20.p']
    finished = 0
    percentages = []
    maes = [];
    rmses = [];
    crashes = []

    for file in chasedFiles:
        print('\nAnalysis of drive',file,)
        historyChased = analyse.loadPositionHistory(os.path.join(dirChased, file))
        historyChasing = analyse.loadPositionHistory(os.path.join(dirChasing, file))
        percentage = analyse.FinishedPercentage(historyChased, historyChasing)
        mae, rmse, crash = analyse.FindMAE(file)
        print('How long was the car able to chase the other car:',str(percentage),'[percentage]')
        print('MAE between desired and actual distance:',str(mae),'[meters]')
        print('RMSE between desired and actual distance:', str(rmse),'[meters]')
        print('Total number of crashes:', str(crash))
        if percentage >= 0.94:
            finished += 1
            # print(mae, rmse, crash)
            maes.append(mae)
            rmses.append(rmse)
            crashes.append(crash)

        percentages.append(percentage)
    print('\n\n---------------------------OVERALL STATITISTICS-------------------------------')
    print('Number of finished drives:', finished)
    print('Average percentage of finished drives:', np.mean(percentages))
    if len(crashes) > 0:
        print('Average number of crashes:',np.mean(crashes))
        print('Average number MAE:', np.mean(maes))
        print('Average number RMSE:',np.mean(rmses))

    return np.mean(percentages)


def PlotGraphs(dirChased,dirChasing,chasedFiles):
    #TODO move previous trajectories and graphs
    dirChased = dirChased
    dirChasing = dirChasing
    chasedFiles = chasedFiles
    analyse = AnalyseResults(dirChased)

    dirGraphs = 'graphs'
    if not os.path.exists(dirGraphs):
        os.mkdir(dirGraphs)


    for file in chasedFiles:
        distances = []
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
        plt.savefig(os.path.join(dirGraphs,'distances_'+file.split('.')[0]+'.pdf'), bbox_inches='tight')
        plt.clf()

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
        plt.savefig(os.path.join(dirGraphs,'speeds_'+file.split('.')[0]+'.pdf'), bbox_inches='tight')
        plt.clf()

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
        plt.savefig(os.path.join(dirGraphs,'angles_'+file.split('.')[0]+'.pdf'), bbox_inches='tight')
        plt.clf()

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
        plt.savefig(os.path.join(dirGraphs,'yaw_rate_'+file.split('.')[0]+'.pdf'), bbox_inches='tight')
        plt.clf()


def PlotTrajectory(dirChased,dirChasing,chasedFiles):
    chasedFiles = chasedFiles
    analyse = AnalyseResults(dirChased)
    # print(chasedFiles)


    for file in chasedFiles:
        historyChased = analyse.loadPositionHistory(os.path.join(dirChased, file))
        historyChasing = analyse.loadPositionHistory(os.path.join(dirChasing, file))

        historyChasing = np.array(historyChasing)
        historyChased = np.array(historyChased)
        # print(len(historyChasing),len(historyChased))
        plt.rc('axes', labelsize=26)

        plt.plot(historyChasing[:,1],historyChasing[:,0],color='#0065BD',linewidth=4,alpha=0.7) # #0065BD
        plt.plot(historyChased[:,1],historyChased[:,0],color='black',linewidth=3,alpha=0.7,linestyle='--',dashes=(2, 1))
        # plt.scatter(X, speeds, color=colors, s=100)
        # patchesList = []
        # patchesList.append(mpatches.Patch(color='#0065BD', label='Chasing car'))
        # patchesList.append(mpatches.Patch(color='black', label='Chased car'))
        # plt.legend(handles=patchesList)

        plt.ylabel('X [m]')
        plt.xlabel('Y [m]')
        plt.gca().set_aspect("equal")
        dirTrajectories = 'trajectories'
        if not os.path.exists(dirTrajectories):
            os.mkdir(dirTrajectories)

        plt.savefig(os.path.join(dirTrajectories,'trajectory_'+file.split('.')[0]+'.pdf'), bbox_inches='tight')
        plt.clf()




def main():
    chasedFiles = []
    dirChased = 'drives'
    dirChasing = 'chasingTrajectory'
    analyse = AnalyseResults(dirChased)

    lines = analyse.LoadResults()
    for line in lines:
        tmp = line.split(',')
        rideName = tmp[0].split('/')[1]
        chasedFiles.append(rideName)


    PlotTrajectory(dirChased,dirChasing,chasedFiles)
    PlotGraphs(dirChased,dirChasing,chasedFiles)
    Analyse('chasingTrajectory')


if __name__ == '__main__':
    main()
