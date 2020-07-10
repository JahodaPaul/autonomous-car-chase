#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
from CarDetector import CarDetector
from SemanticSegmentation import SemanticSegmentation
import math
import pickle
import pygame

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random

# try:
import pygame
# except ImportError:
#     raise RuntimeError('cannot import pygame, make sure pygame package is installed')

# try:
import numpy as np
# except ImportError:
#     raise RuntimeError('cannot import numpy, make sure numpy package is installed')

# try:
import queue
# except ImportError:
#     import Queue as queue


class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data

import os
def myPrint(angle,predicted_angle, possibleAngle,real_dist, predicted_distance, chaseMode=True):
    os.system('clear')
    print('Real angle:',angle)
    print('Predicted angle:',predicted_angle)
    print('Possible angle:',possibleAngle)
    print('Real distance:',real_dist)
    print('Predicted distance:',predicted_distance)

import imageio
from copy import deepcopy
def draw_image(surface, image, image2,location1, location2, blend=False, record=False,driveName='',smazat=[]):
    if record:
        driveName = driveName.split('/')[1]
        dirName = os.path.join('output',driveName)
        if not os.path.exists(dirName):
            os.mkdir(dirName)
        image.save_to_disk(dirName+'/%07d' % image.frame)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))


def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def should_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False




class CarPosition(object):
    def __init__(self,name):
        self.startRecording = True
        self.history = []
        self.fileName = name
        self.outputDir = 'chasingTrajectory'

    def SaveCarPosition(self,location):
        if self.startRecording:
            self.history.append([location.location.x, location.location.y, location.location.z, location.rotation.pitch, location.rotation.yaw, location.rotation.roll])

    def SaveHistoryToFile(self):
        if not os.path.exists(self.outputDir):
            os.mkdir(self.outputDir)
        if len(self.history) > 0:
            pickle.dump(self.history,  open(os.path.join(self.outputDir,self.fileName), "wb"))


class Evaluation():
    def __init__(self):
        self.sumMAE = 0
        self.sumRMSE = 0
        self.n_of_frames = 0
        self.n_of_collisions = 0
        self.history = []

    def AddError(self, distance, goalDistance):
        self.n_of_frames += 1
        self.sumMAE += abs(goalDistance-distance)
        self.sumRMSE += abs(goalDistance-distance)*abs(goalDistance-distance)

    def WriteIntoFileFinal(self, filename, driveName):
        if self.n_of_frames > 0:
            self.sumMAE = self.sumMAE / float(self.n_of_frames)
            self.sumRMSE = self.sumRMSE / float(self.n_of_frames)

        with open(filename,'a') as f:
            f.write(str(driveName)+', '+str(self.sumMAE)+', '+str(self.sumRMSE)+', '+str(self.n_of_collisions)+'\n')

    def LoadHistoryFromFile(self, fileName):
        self.history = pickle.load( open(fileName, "rb"))

    def CollisionHandler(self,event):
        self.n_of_collisions += 1

import copy
def main(optimalDistance, followDrivenPath, chaseMode, evaluateChasingCar, driveName='',record=False, followMode=False,
         resultsName='results',P=None,I=None,D=None,nOfFramesToSkip=0):
    counter = 1

    actor_list = []
    pygame.init()

    carDetector = CarDetector()
    position = CarPosition(driveName.split('/')[1])
    position.startRecording = True

    evaluation = Evaluation()
    semantic = SemanticSegmentation()

    display = pygame.display.set_mode(
        (800, 600),
        pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = get_font()
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()

    vehicleToFollowSpawned = False

    try:
        m = world.get_map()
        start_pose = random.choice(m.get_spawn_points())

        blueprint_library = world.get_blueprint_library()
        
        vehicle = world.spawn_actor(
            random.choice(blueprint_library.filter('jeep')),
            start_pose)
        actor_list.append(vehicle)
        vehicle.set_simulate_physics(True)
        if followDrivenPath:
            evaluation.LoadHistoryFromFile(driveName)
            first = evaluation.history[0]
            start_pose = carla.Transform(carla.Location(first[0], first[1], first[2]),
                                         carla.Rotation(first[3], first[4], first[5]))
            vehicle.set_transform(start_pose)

        collision_sensor = world.spawn_actor(blueprint_library.find('sensor.other.collision'),
                                             carla.Transform(), attach_to=vehicle)

        collision_sensor.listen(lambda event: evaluation.CollisionHandler(event))
        actor_list.append(collision_sensor)
        

        blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
        blueprint.set_attribute('image_size_x', '800')
        blueprint.set_attribute('image_size_y', '600')
        blueprint.set_attribute('fov', '90')

        camera_rgb = world.spawn_actor(
            blueprint_library.find('sensor.camera.rgb'),
            carla.Transform(carla.Location(x=1.5, z=1.4,y=0.3), carla.Rotation(pitch=0)), #5,3,0 # -0.3
            attach_to=vehicle)
        actor_list.append(camera_rgb)

        camera_rgb2 = world.spawn_actor(
            blueprint_library.find('sensor.camera.rgb'),
            carla.Transform(carla.Location(x=1.5, z=1.4,y=-0.3), carla.Rotation(pitch=0)),attach_to=vehicle)
        actor_list.append(camera_rgb2)

        camera_segmentation = world.spawn_actor(
            blueprint_library.find('sensor.camera.semantic_segmentation'),
            carla.Transform(carla.Location(x=1.5, z=1.4,y=0), carla.Rotation(pitch=0)), #5,3,0 # -0.3
            attach_to=vehicle)
        actor_list.append(camera_segmentation)


        # Create a synchronous mode context.
        with CarlaSyncMode(world,camera_rgb, camera_rgb2, camera_segmentation, fps=30) as sync_mode:

            while True:
                if should_quit():
                    return
                clock.tick(30)

                # Advance the simulation and wait for the data.
                snapshot, img_rgb, image_rgb2, image_segmentation = sync_mode.tick(timeout=2.0)

                line = []

                if not vehicleToFollowSpawned and followDrivenPath:
                    vehicleToFollowSpawned = True
                    location1 = vehicle.get_transform()
                    newX, newY = carDetector.CreatePointInFrontOFCar(location1.location.x, location1.location.y,
                                                                     location1.rotation.yaw)
                    diffX = newX - location1.location.x
                    diffY = newY - location1.location.y
                    newX = location1.location.x - (diffX*5)
                    newY = location1.location.y - (diffY*5)

                    start_pose.location.x = newX
                    start_pose.location.y = newY

                    vehicle.set_transform(start_pose)

                    start_pose2 = random.choice(m.get_spawn_points())

                    bp = blueprint_library.filter('model3')[0]
                    bp.set_attribute('color', '0,101,189')
                    vehicleToFollow = world.spawn_actor(
                        bp,
                        start_pose2)

                    start_pose2 = carla.Transform()
                    start_pose2.rotation = start_pose.rotation

                    start_pose2.location.x = start_pose.location.x
                    start_pose2.location.y = start_pose.location.y
                    start_pose2.location.z = start_pose.location.z

                    vehicleToFollow.set_transform(start_pose2)

                    actor_list.append(vehicleToFollow)
                    vehicleToFollow.set_simulate_physics(True)
                    vehicleToFollow.set_autopilot(False)

                if followDrivenPath:
                    if counter >= len(evaluation.history):
                        break
                    tmp = evaluation.history[counter]
                    currentPos = carla.Transform(carla.Location(tmp[0],tmp[1],tmp[2]),carla.Rotation(tmp[3],tmp[4],tmp[5]))
                    vehicleToFollow.set_transform(currentPos)
                    counter += 1

                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                location1 = vehicle.get_transform()
                location2 = vehicleToFollow.get_transform()

                position.SaveCarPosition(location1)

                newX, newY = carDetector.CreatePointInFrontOFCar(location1.location.x, location1.location.y,location1.rotation.yaw)
                angle = carDetector.getAngle([location1.location.x, location1.location.y], [newX, newY],
                                             [location2.location.x, location2.location.y])

                possibleAngle = 0
                carInTheImage = semantic.IsThereACarInThePicture(image_segmentation)
                bbox = carDetector.get3DboundingBox(vehicleToFollow, camera_rgb,carInTheImage)

                # Choose approriate steer and throttle here
                steer, throttle = 0, 1
                vehicle.apply_control(carla.VehicleControl(throttle=throttle,steer=steer))

                if evaluateChasingCar:
                    evaluation.AddError(location1.location.distance(location2.location),optimalDistance)

                velocity1 = vehicle.get_velocity()
                velocity2 = vehicleToFollow.get_velocity()


                draw_image(display, image_rgb2, image_segmentation,location1, location2,record=record,driveName=driveName,smazat=line)
                display.blit(font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)),(8, 10))
                display.blit(font.render('% 5d FPS (simulated)' % fps, True, (255, 255, 255)),(8, 28))

                if len(bbox) != 0:
                    points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
                    BB_COLOR = (248, 64, 24)
                    # draw lines
                    # base
                    pygame.draw.line(display, BB_COLOR, points[0], points[1])
                    pygame.draw.line(display, BB_COLOR, points[1], points[2])
                    pygame.draw.line(display, BB_COLOR, points[2], points[3])
                    pygame.draw.line(display, BB_COLOR, points[3], points[0])
                    # top
                    pygame.draw.line(display, BB_COLOR, points[4], points[5])
                    pygame.draw.line(display, BB_COLOR, points[5], points[6])
                    pygame.draw.line(display, BB_COLOR, points[6], points[7])
                    pygame.draw.line(display, BB_COLOR, points[7], points[4])
                    # base-top
                    pygame.draw.line(display, BB_COLOR, points[0], points[4])
                    pygame.draw.line(display, BB_COLOR, points[1], points[5])
                    pygame.draw.line(display, BB_COLOR, points[2], points[6])
                    pygame.draw.line(display, BB_COLOR, points[3], points[7])

                real_dist = location1.location.distance(location2.location)
                #if chaseMode or followMode:
                    #myPrint(angle,predicted_angle, possibleAngle,real_dist, predicted_distance,chaseMode)
                pygame.display.flip()
    except Exception as ex:
        print(ex)
    finally:
        print('Ending')
        if evaluateChasingCar:
            if not os.path.exists('res'):
                os.mkdir('res')
            evaluation.WriteIntoFileFinal(os.path.join('res',resultsName+'.txt'),driveName=driveName)
        position.SaveHistoryToFile()
        print('destroying actors.')
        for actor in actor_list:
            actor.destroy()

        pygame.quit()
        print('done.')

import os
if __name__ == '__main__':
    if os.path.exists(os.path.join('res','results.txt')):
        cnt = 0
        while True:
            cnt += 1
            if not os.path.exists(os.path.join('res','results'+str(cnt)+'.txt')):
                os.rename(os.path.join('res','results.txt'),os.path.join('res','results'+str(cnt)+'.txt'))

    try:
        optimalDistance = 8
        followDrivenPath = True
        evaluateChasingCar = True
        record = False
        chaseMode = True
        followMode = False

        drivesDir = 'drives'
        drivesFileNames = os.listdir(drivesDir)
        drivesFileNames.sort()

        drivesFileNames = ['ride1.p']

        # all drives
        # drivesFileNames = ['ride1.p','ride2.p','ride3.p','ride4.p','ride5.p','ride6.p','ride7.p','ride8.p','ride9.p','ride10.p',
        #                   'ride11.p', 'ride12.p', 'ride13.p', 'ride14.p', 'ride15.p', 'ride16.p', 'ride17.p', 'ride18.p','ride19.p', 'ride20.p']

        if evaluateChasingCar:
            for fileName in drivesFileNames:
                main(optimalDistance=optimalDistance,followDrivenPath=followDrivenPath,chaseMode=chaseMode, evaluateChasingCar=evaluateChasingCar,driveName=os.path.join(drivesDir,fileName),record=record,followMode=followMode)

    except Exception as ex:
        with open('problem.txt','a') as f:
            f.write(str(ex)+'\n')
