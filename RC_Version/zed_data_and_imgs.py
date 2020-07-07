import pyzed.sl as sl
import csv
import time
# import matplotlib
import logging
import sys
from threading import Thread, Lock
# import signal
import cv2
from scipy import stats
import numpy as np
import math
import copy

carChase = True
# if not carChase:
#     from birdEyeTransform import transform
#     sys.path.append('../MachineLearning/Segmentation/Codes')
#     import segmentator  # noqa: E402

from CarDetector import CarDetector
from DrivingControl import DrivingControl

carDetector = CarDetector()
drivingControl = DrivingControl(1.2)

# Create ZED image data lock
img_lock = Lock()
img_data = None
img_path = None

### Getting Rotatation and Translation Vector and save them to csv file ###

# ImageAcquisition
# Initialize logger
logger = logging.getLogger('zed_logger')
logger.setLevel(logging.DEBUG)

sh = logging.StreamHandler(sys.stdout)
sh.setLevel(logging.DEBUG)

fh = logging.FileHandler('zed.log')
fh.setLevel(logging.DEBUG)

file_formatter = logging.Formatter(
    '[%(asctime)s - -%(name)s - %(levelname)s] %(message)s')
fh.setFormatter(file_formatter)
prompt_formatter = logging.Formatter('[%(levelname)s] %(message)s')
sh.setFormatter(prompt_formatter)

logger.addHandler(fh)
logger.addHandler(sh)

# TEMP:START should be in other module
# Initialize
cut = 300

p01 = [570, 0]
p02 = [685, 0]
p03 = [181, 394]
p04 = [1072, 403]

IMAGE_W = 1280
IMAGE_H = 720 - cut
IMAGE_H_DEFORM = round(IMAGE_W*(3/2))

percentalBorder = 0.35
p11 = [IMAGE_W * percentalBorder, 0]
p12 = [IMAGE_W - (IMAGE_W * percentalBorder), 0]
p13 = [IMAGE_W * percentalBorder, IMAGE_H_DEFORM]
p14 = [IMAGE_W - (IMAGE_W * percentalBorder), IMAGE_H_DEFORM]
src = np.float32([p01, p02, p03, p04])
dst = np.float32([p11, p12, p13, p14])

M = cv2.getPerspectiveTransform(src, dst)

# TEMP:END


def init_zed(depth):

    if depth:
        depth_mode = sl.DEPTH_MODE.DEPTH_MODE_PERFORMANCE
    else:
        depth_mode = sl.DEPTH_MODE.DEPTH_MODE_NONE
    init = sl.InitParameters(camera_resolution=sl.RESOLUTION.RESOLUTION_HD720,
                             camera_fps=30,
                             camera_image_flip=True,
                             depth_mode=depth_mode,
                             coordinate_units=sl.UNIT.UNIT_METER,
                             coordinate_system=sl.COORDINATE_SYSTEM.COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP,
                             sdk_verbose=True)
    return init


def DataAcquisiton(zed, zed_pose, runtime_parameters):

    while True:

        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:

            #time1 = time.time()

            zed.get_position(zed_pose, sl.REFERENCE_FRAME.REFERENCE_FRAME_WORLD)  # Updating Pose

            # Translation
            translation = sl.Translation()
            tx = round(zed_pose.get_translation(translation).get()[0], 3)
            ty = round(zed_pose.get_translation(translation).get()[1], 3)
            tz = round(zed_pose.get_translation(translation).get()[2], 3)

            # Rotation
            rotation = zed_pose.get_rotation_vector()
            rx = round(rotation[0], 2)
            ry = round(rotation[1], 2)
            rz = round(rotation[2], 2)

            with open('/media/ctuxavier/ADATASE730H/zed_recording.csv', 'a') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow([ty, tx, tz, rx, ry, rz, time.time()])

def ConvertThrottleValue(val):
    val = 1520 + min((val*120),120)
    return int(val)

def ConvertSteerValue(steer,useSqrtVal=True):
    if useSqrtVal:
        if steer >= 0:
            steer = math.pow(abs(steer),1/2)
        else:
            steer = math.pow(abs(steer),1/2) * -1
    # steer = -1
    val = 1450 + (steer*300)
    return int(val)


def CarDetection(throttle_SP,steer_SP):

    global img_data, img_lock, img_path


    while True:
        with img_lock:
            image = img_data
            img_path_save = copy.deepcopy(img_path)

        if img_data is None:
            continue

        predicted_distance, predicted_angle = carDetector.Run(image, False)
        logger.info("Distance: " + str(predicted_distance) + " Angle: " + str(predicted_angle))
        logger.info("Analyzed image: "+str(img_path_save))

        steer, throttle = drivingControl.PredictSteerAndThrottle(predicted_distance, predicted_angle)
        throttle = ConvertThrottleValue(throttle)
        steer = ConvertSteerValue(steer)

        logger.info("Steer: " + str(steer) + " Throttle: " + str(throttle))

        with throttle_SP.get_lock():
            throttle_SP.value = throttle

        with steer_SP.get_lock():
            steer_SP.value = steer

    return False


def ImageAcquisition(zed, mat, runtime_parameters):
    # Init
    cntr = 0
    fps_timer = time.time()
    logger.info("Reading started...")
    global img_data, img_lock, img_path

    while True:
        # Obtain camera image and get timestamps
        t1 = time.time()
        err = zed.grab(runtime_parameters)

        # If grabbing image successfull, save to buffer
        if err == sl.ERROR_CODE.SUCCESS:

            zed.retrieve_image(mat, sl.VIEW.LEFT)#sl.VIEW.VIEW_SIDE_BY_SIDE) #CHANGED
            cntr += 1
            grab_timestamp = str(t1)
            grab_timestamp = grab_timestamp.replace('.', '')  # remove "." due to folder organization
            img_name = '/media/ctuxavier/ADATASE730H/images/img_{}.jpg'.format(grab_timestamp)

            img = sl.ERROR_CODE.ERROR_CODE_FAILURE
            countdown = 5
            while img != sl.ERROR_CODE.SUCCESS and countdown > 0:

                # Write image and image data to files
                write_timestamp = time.time()
                img = mat.write(img_name)
                with img_lock:
                    img_data = mat.get_data()
                    img_path = img_name[:-4]

                with open('/media/ctuxavier/ADATASE730H/img_data.txt', 'a') as data_file:   # external SSD
                    data_file.write("{}, {}\n".format(grab_timestamp,
                                                      write_timestamp))

                countdown -= 1
                if img == sl.ERROR_CODE.SUCCESS:
                    break

                logger.debug("Image not written!")
            time.sleep(0.01)

            # Count FPS
            if (time.time() - fps_timer) > 1:
                fps_timer = time.time()
                logger.info(f"Current FPS: {cntr}")
                cntr = 0


def get_front_surface(seg_img, d, r):
    """ Get surfes id in front of camera.

    Counts all pixel values in selected square of side a=2*r located with
    center in the middle of y-axis and d pixels from the bottom.

    Arguments:
        seg_img: segmented image
        d: distance (px) from bottom of image
        r: detection square half-side length
    """

    img_size = seg_img.shape

    if r > d or (d + r) > img_size[1]:
        raise ValueError("Invalid combination of 'r' and 'd' argument!")

    y1 = img_size[0] - d - r
    y2 = img_size[0] - d + r
    x1 = (img_size[1] // 2) - r
    x2 = (img_size[1] // 2) + r

    area = seg_img[y1:y2+1, x1:x2+1]

    mode, count = stats.mode(area, axis=None)

    area_size = (2*r+1)**2

    accuracy = count[0] / area_size

    res = mode[0]

    return (res, accuracy)


def set_abs_k(surface, abs_k):

    if not abs_k:
        print("Cannot access abs_k.")
        return False

    with abs_k:
        abs_k.value = surface

    return True


def image_segmentation(abs_k):

    global img_data, img_lock, img_path

    segm = segmentator.KerasSegmentator(
        height=960,
        width=640,
        num_classes=14,
        model_path='seg_gravel.h5',
        real_weight=30
        )

    while True:

        # img_lock.acquire(True, -1)
        # img_lock.release()

        with img_lock:
            image = img_data
            file_path = img_path

        if img_data is None:
            continue

        img = transform(image)

        # img = image[0:720, 0:1280, 0:3]

        # img = img[cut:(cut+IMAGE_H), 0:IMAGE_W]

        # imgTrans = PerspTransform(imgLeft, True)

        # img = cv2.warpPerspective(img, M,
        #                           (IMAGE_W, IMAGE_H_DEFORM))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # img = cv2.resize(img, (640, 960))
        img = cv2.resize(img, (384, 768))

        out = segm.segment_one(np.asarray(img))[0]

        # res, acc = get_front_surface(out, 200, 150)
        res, acc = get_front_surface(out, 100, 75)
        set_abs_k(res, abs_k)

        logger.info(f"Surface {res} with {acc*100} % accuracy.")

        # Save birdeye image
        cv2.imwrite(file_path + "_birdeye" + ".jpg", img)
        # Save mask image
        cv2.imwrite(file_path + "_annot" + ".png", out)
        print("Mask and birdeye saved.\n")

    return False


def main(abs_k=None, depth=True, throttle_SP=None, steer_SP=None):
    # Create a Camera object
    zed = sl.Camera()

    # Initilialize Camera Parameters
    init_params = init_zed(depth)

    # Open the camera
    zed_cam = zed.open(init_params)

    if zed_cam != sl.ERROR_CODE.SUCCESS:
        print("Error: Zed Camera could not be opened.")
        exit()

    print("")
    print("Zed Camera succesfully opened.")

    # # Enable positional tracking
    # if depth:
    #     transform = sl.Transform()
    #     tracking_parameters = sl.TrackingParameters(init_pos=transform)
    #     tracking = zed.enable_tracking(tracking_parameters)
    #
    #     if tracking != sl.ERROR_CODE.SUCCESS:
    #         exit()
    #
    #     print("")
    #     print("Tracking succesfully enabled.")

    runtime_parameters = sl.RuntimeParameters()

    zed_pose = sl.Pose()  # Getting Pose

    mat = sl.Mat()

    with open('/media/ctuxavier/ADATASE730H/zed_recording.csv', mode='w') as csv_file: # Open csv file
        print("Start Recording")
        print("Press Ctrl+C to stop")

        fieldnames = ['Tx', 'Ty', 'Tz', 'Pitch', 'Roll', 'Yaw', 'Timestamp']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

    with open('/media/ctuxavier/ADATASE730H/img_data.txt', 'w') as data_file:
        data_file.write("grab_timestamp, write_timestamp\n")

    threads = []
    # Init and start image acquisition thread
    ImageAcquisition_t = Thread(target=ImageAcquisition, args=(zed, mat, runtime_parameters))
    ImageAcquisition_t.daemon = True
    threads.append(ImageAcquisition_t)
    ImageAcquisition_t.start()

    # if depth:
    #     # Init and start data acquisition thread
    #     DataAcquisiton_t = Thread(target=DataAcquisiton, args=(zed, zed_pose, runtime_parameters))
    #     DataAcquisiton_t.daemon = True
    #     threads.append(DataAcquisiton_t)
    #     DataAcquisiton_t.start()

    # start car detection thread by Pavel Jahoda - for autonomous car chasing
    if carChase:
        car_detection_t = Thread(target=CarDetection,args=(throttle_SP,steer_SP))
        car_detection_t.daemon = True
        threads.append(car_detection_t)
        car_detection_t.start()

    # Init and start image segmentation thread
    # image_segmentation_t = Thread(target=image_segmentation, args=(abs_k, ))
    # image_segmentation_t.daemon = True
    # threads.append(image_segmentation)
    # image_segmentation_t.start()

    try:
        for thread in threads:
            thread.join()
    except (KeyboardInterrupt, SystemExit):
        print("Zed data and images threads closed!")


if __name__ == "__main__":
    main()
