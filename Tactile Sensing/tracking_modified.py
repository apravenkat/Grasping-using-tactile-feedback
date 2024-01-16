import copy
import matplotlib as ml
import matplotlib.pyplot as plt
import find_marker
from serialRead import *
from tempfile import TemporaryFile
import pandas as pd
from scipy.stats import entropy
import numpy as np
import rospy
from std_msgs.msg import Int32
from robotiq_hande_ros_driver.srv import gripper_service
from matplotlib.animation import FuncAnimation
import cv2
import time
import marker_detection
import sys
import matplotlib.animation as animation
import joblib
import setting
import matplotlib.pyplot as plt
import pickle as pk
# from utils.live_ximea import GelSight
from camera_calibration import warp_perspective
import os
p = 0


def find_cameras():
    # checks the first 10 indexes.
    index = 0
    arr = []
    i = 10
    while i >= 0:
        cap = cv2.VideoCapture(index)
        if cap.read()[0]:
            command = 'v4l2-ctl -d ' + str(index) + ' --info'
            is_arducam = os.popen(command).read()
            if is_arducam.find('Arducam') != -1 or is_arducam.find('Mini') != -1:
                arr.append(index)
            cap.release()
        index += 1
        i -= 1

    return arr


def resize_crop_mini(img, imgw, imgh):
    # resize, crop and resize back
    # size suggested by janos to maintain aspect ratio
    img = cv2.resize(img, (895, 672))
    border_size_x, border_size_y = int(img.shape[0] * (1 / 7)), int(
        np.floor(img.shape[1] * (1 / 7)))  # remove 1/7th of border from each size
    img = img[border_size_x:img.shape[0] - border_size_x,
              border_size_y:img.shape[1] - border_size_y]
    img = img[:, :-1]  # remove last column to get a popular image resolution
    img = cv2.resize(img, (imgw, imgh))  # final resize for 3d
    return img


def trim(img):
    img[img < 0] = 0
    img[img > 255] = 255


def compute_tracker_gel_stats(thresh):
    numcircles = 9 * 7
    mmpp = .0625
    true_radius_mm = .5
    true_radius_pixels = true_radius_mm / mmpp
    circles = np.where(thresh)[0].shape[0]
    circlearea = circles / numcircles
    radius = np.sqrt(circlearea / np.pi)
    radius_in_mm = radius * mmpp
    percent_coverage = circlearea / (np.pi * (true_radius_pixels) ** 2)
    return radius_in_mm, percent_coverage*100.


'''rospy.init_node("force_subscriber", anonymous=True)
gripper_srv = rospy.ServiceProxy('gripper_service', gripper_service)
response = gripper_srv(
    position=0, speed=255, force=255)
p = 0

rospy.Subscriber('force_data', Float32,
                 Detection.callback)
rospy.spin()

print(Detection.posi)
p = Detection.posi
'''
imgw = 320
imgh = 240

USE_LIVE_R1 = False
calibrate = False
border_size = 25

outdir = './TEST/'
SAVE_VIDEO_FLAG = False
SAVE_ONE_IMG_FLAG = False

if SAVE_ONE_IMG_FLAG:
    sn = input('Please enter the serial number of the gel \n')
    # sn = str(5)
    viddir = outdir + 'vids/'
    imgdir = outdir + 'imgs/'
    resultsfile = outdir + 'marker_qc_results.txt'
    vidfile = viddir + sn + '.avi'
    imgonlyfile = imgdir + sn + '.png'
    maskfile = imgdir + 'mask_' + sn + '.png'
    # check to see if the directory exists, if not create it
    if not os.path.exists(outdir):
        os.mkdir(outdir)
    if not os.path.exists(viddir):
        os.mkdir(viddir)
    if not os.path.exists(imgdir):
        os.mkdir(imgdir)


# if len(sys.argv) > 1:
#     if sys.argv[1] == 'calibrate':
#         calibrate = True

time.sleep(2)
if USE_LIVE_R1:
    gs = GelSight(0)
    WHILE_COND = 1
else:
    cameras = find_cameras()
    cap = cv2.VideoCapture(cameras[0])
    # cap = cv2.VideoCapture('http://pi:robits@raspiatgelsightinc.local:8080/?action=stream')
    # cap = cv2.VideoCapture('/home/radhen/Downloads/GS-Mini_Test_X-Axis.avi')
    WHILE_COND = cap.isOpened()

# set the format into MJPG in the FourCC format
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

# Resize scale for faster image processing
setting.init()
RESCALE = setting.RESCALE

if SAVE_VIDEO_FLAG:
    # Below VideoWriter object will create a frame of above defined The output is stored in 'filename.avi' file.
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # fourcc = cv2.VideoWriter_fourcc('p', 'n', 'g', ' ')
    # fourcc = cv2.VideoWriter_fourcc(*'jp2','')
    out = cv2.VideoWriter(vidfile, fourcc, 25, (imgw, imgh), isColor=True)

frame0 = None
counter = 0
while 1:
    if counter < 50:
        ret, frame = cap.read()
        print('flush black imgs')

        if counter == 48:
            ret, frame = cap.read()
            ##########################
            frame = resize_crop_mini(frame, imgw, imgh)
            # find marker masks
            mask = marker_detection.find_marker(frame)
            # find marker centers
            mc = marker_detection.marker_center(mask, frame)
            break

        counter += 1

counter = 0


mccopy = mc
mc_sorted1 = mc[mc[:, 0].argsort()]
mc1 = mc_sorted1[:setting.N_]
mc1 = mc1[mc1[:, 1].argsort()]

mc_sorted2 = mc[mc[:, 1].argsort()]
mc2 = mc_sorted2[:setting.M_]
mc2 = mc2[mc2[:, 0].argsort()]


"""
N_, M_: the row and column of the marker array
x0_, y0_: the coordinate of upper-left marker
dx_, dy_: the horizontal and vertical interval between adjacent markers
"""
N_ = setting.N_
M_ = setting.M_
fps_ = setting.fps_
x0_ = np.round(mc1[0][0])
y0_ = np.round(mc1[0][1])
dx_ = mc2[1, 0] - mc2[0, 0]
dy_ = mc1[1, 1] - mc1[0, 1]

print('x0:', x0_, '\n', 'y0:', y0_, '\n', 'dx:', dx_, '\n', 'dy:', dy_)

radius, coverage = compute_tracker_gel_stats(mask)

if SAVE_ONE_IMG_FLAG:
    fresults = open(resultsfile, "a")
    fresults.write(
        f"{sn} {float(f'{dx_:.2f}')} {float(f'{dy_:.2f}')} {float(f'{radius*2:.2f}')} {float(f'{coverage:.2f}')}\n")


# cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
# cv2.namedWindow('mask', cv2.WINDOW_NORMAL)
# cv2.resizeWindow('frame', 320*3, 240*3)
# cv2.resizeWindow('mask', 320*3, 240*3)
# Create Mathing Class

m = find_marker.Matching(N_, M_, fps_, x0_, y0_, dx_, dy_)
# rospy.init_node("gripper_test_node")
# gripper_srv = rospy.ServiceProxy('gripper_service', gripper_service)
slip_array = []
time_count = 0
time_array = []
count = 0
entropy_array = []
vx_array = []
prev_err = 0
old_dx = 0
old_dy = 0
old_ent = 0
vy_array = []
Kp = 100
Kd = 0.5
delta_entropy_array = []
des_prob = 1
my_model = joblib.load('classifier_svm.sav')
s = 0
f = 0
try:
    tm = time.time()
    while (WHILE_COND):
        if USE_LIVE_R1:
            gs.cam.get_image(gs.img)
            frame = gs.img.get_image_data_numpy()
        else:
            ret, frame = cap.read()
            if not (ret):
                break

        ##########################
        # resize (or unwarp)
        # frame = cv2.resize(frame, (imgw,imgh))
        # frame = frame[10:-10,5:-10] # for R1.5
        # frame = frame[border_size:imgh - border_size, border_size:imgw - border_size] # for mini
        frame = resize_crop_mini(frame, imgw, imgh)
        raw_img = copy.deepcopy(frame)

        # frame = frame[55:,:]
        # frame = cv2.resize(frame, (imgw, imgh))

        ''' EXTRINSIC calibration ...
        ... the order of points [x_i,y_i] | i=[1,2,3,4], are same
        as they appear in plt.imshow() image window. Put them in
        clockwise order starting from the topleft corner'''
        # frame = frame[30:400, 70:400]
        # frame = warp_perspective(frame, [[35, 15], [320, 15], [290, 360], [65, 360]], output_sz=frame.shape[:2])   # params for small dots
        # frame = warp_perspective(frame, [[180, 130], [880, 130], [800, 900], [260, 900]], output_sz=(640,480)) # org. img size (1080x1080)

        # intrinsic calibration
        # path = '/home/radhen/Documents/tactile_sdk_local/utils/camera_intrinsic_params/'
        # with open(path + "mtx.txt", "rb") as fp:  # Unpickling
        #     mtx = pk.load(fp)
        # with open(path + "dist.txt", "rb") as fp:  # Unpickling
        #     dist = pk.load(fp)
        # img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # h, w = img_gray.shape[:2]
        # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        # # undistort
        # frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        # # frame = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        # frame = marker_detection.init(frame)
        # frame = marker_detection.init_HSR(frame)
        ############################################################

        # find marker masks
        mask = marker_detection.find_marker(frame)

        # find marker centers
        mc = marker_detection.marker_center(mask, frame)

        if calibrate == False:

            # matching init
            m.init(mc)
            print("hi")
            # matching
            m.run()
            # print(time.time() - tm)

            # matching result
            """
            output: (Ox, Oy, Cx, Cy, Occupied) = flow
                Ox, Oy: N*M matrix, the x and y coordinate of each marker at frame 0
                Cx, Cy: N*M matrix, the x and y coordinate of each marker at current frame
                Occupied: N*M matrix, the index of the marker at each position, -1 means inferred.
                    e.g. Occupied[i][j] = k, meaning the marker mc[k] lies in row i, column j.
            """
            flow = m.get_flow()

            if frame0 is None:
                frame0 = frame.copy()
                frame0 = cv2.GaussianBlur(frame0, (int(63), int(63)), 0)

            # diff = (frame * 1.0 - frame0) * 4 + 127
            # trim(diff)
            # print("Flow ", flow)
            # # draw flow

            ent, vx, vy, delta_entropy, dx, dy = marker_detection.draw_flow(
                frame, flow, old_ent, old_dx, old_dy)

            features = np.array([vx, vy])
            # features = np.array([vx, vy, delta_entropy, ent])
            features = features.reshape(1, 2)
            print(features)
            slip_flag = my_model.predict(features)
            # probs = my_model.predict_proba(features).round(2)
            # noslip_prob = probs[0, 0]
            # err = des_prob - noslip_prob
            # act = np.rint(np.abs(np.rint(Kp*err + Kd*(err - prev_err))))
            # act = int(act+195)
            # print("Actuation ", act)
            status = ""
            if slip_flag == 0:
                status = "No Slip Detected"

            else:
                '''speed = int(np.abs(delta_entropy/200)*255)
                delta_pos = int(np.abs(delta_entropy/200)*10)
                p = p+delta_pos
                response = gripper_srv(
                    position=p, speed=255, force=100)'''
                status = "Slip Detected"
            # probabilites = "No Slip " + \
            #     str(probs[0, 0])+" Trans Slip " + \
            #     str(probs[0, 1])+" Rot Slip "+str(probs[0, 2])
            time_count = time.time() - tm
            entropy_array.append(np.abs(ent))
            time_array.append(time_count)
            delta_entropy_array.append(np.abs(delta_entropy))
            vx_array.append(vx)
            slip_array.append(slip_flag)
            vy_array.append(vy)
            # plot_params(time_array, entropy_array,
            #             delta_entropy_array, vx_array, vy_array)
            old_ent = ent
            old_dx = dx
            old_dy = dy
            # prev_err = err
        # mask_img = mask.astype(frame[0].dtype)
        mask_img = np.asarray(mask)
        # mask_img = cv2.merge((mask_img, mask_img, mask_img))

        bigframe = cv2.resize(frame, (frame.shape[1]*3, frame.shape[0]*3))
        bigframe = cv2.putText(bigframe, status, (50, 50),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        parameters = "vx " + str(int(vx)) + " vy " + str(int(vy)) + \
            " ent " + str(int(ent)) + " delta_ent " + str(int(delta_entropy))
        bigframe = cv2.putText(bigframe, parameters, (50, 200),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        # bigframe=cv2.putText(bigframe, probabilites, (50, 100),
        # cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow('frame', bigframe)
        # cv2.moveWindow('frame', 800, 200)
        bigmask = cv2.resize(
            mask_img*255, (mask_img.shape[1]*3, mask_img.shape[0]*3))
        cv2.imshow('mask', bigmask)

        '''if SAVE_ONE_IMG_FLAG:
            cv2.imwrite(imgonlyfilejoblib.dump(
                clf_svm, filename2), raw_img)
            cv2.imwrite(maskfile, mask*255)
            SAVE_ONE_IMG_FLAG = False

        if calibrate:
            # Display the mask
            cv2.imshow('mask', mask_img*255)
        if SAVE_VIDEO_FLAG:
            out.write(frame)
        # print(frame.shape)'''
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # return time_array, entropy_array
        # time.sleep(0.001)

except KeyboardInterrupt:
    x = 0.5
    plt.show()
    window_size = 5
    moving_averages_entropy = []
    moving_averages_entropy.append(entropy_array[0])
    moving_averages_delta_entropy = []
    moving_averages_delta_entropy.append(delta_entropy_array[0])
    moving_averages_dx = []
    moving_averages_dx.append(vx_array[0])
    moving_averages_dy = []
    moving_averages_dy.append(vy_array[0])
    count_array = []
    count_array.append(0)
    i = 20
    slip = 600
    # logging
    '''while i < len(entropy_array) - (100+window_size + 1):

        entropy_wa = round(
            np.sum(entropy_array[i:i+window_size]) / window_size, 2)
        delta_entropy_wa = round(
            np.sum(delta_entropy_array[i:i+window_size]) / window_size, 2)
        dx_wa = round(np.sum(dx_array[i:i+window_size]) / window_size, 2)
        dy_wa = round(np.sum(dy_array[i:i+window_size]) / window_size, 2)

        # Store the cumulative average
        # of current window in moving average list
        moving_averages_entropy.append(entropy_wa)
        moving_averages_delta_entropy.append(delta_entropy_wa)
        moving_averages_dx.append(dx_wa)
        moving_averages_dy.append(dy_wa)
        # Shift window to right by one position
        count_array.append(i)
        i += 1
        '''
    # logData = np.array(
    #     [time_array, entropy_array, delta_entropy_array, vx_array, vy_array])
    logData = np.array(
        [entropy_array, delta_entropy_array, vx_array, vy_array])
    logData = logData.T
    '''np.savetxt("LogData_rotational/box_RotationalSlip.csv",
            logData, delimiter=",")'''
    fig1, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5)
    ax1.plot(time_array, entropy_array)
    ax1.set_title('Entropy')
    ax1.set_xlabel('Counts')
    ax1.set_ylabel('Entropy')

    ax2.plot(time_array, delta_entropy_array)
    ax2.set_title('Rate of Change of Entropy')
    ax2.set_xlabel('Counts')
    ax2.set_ylabel('Rate of Change of Entropy')

    ax3.plot(time_array, vx_array)
    ax3.set_title('Vx')
    ax3.set_xlabel('Counts')
    ax3.set_ylabel('Vx')

    ax4.plot(time_array, vy_array)
    ax4.set_title('Vy')
    ax4.set_xlabel('Counts')
    ax4.set_ylabel('Vy')

    ax5.plot(time_array, slip_array)
    ax5.set_title('Slip')
    ax5.set_xlabel('Counts')
    ax5.set_ylabel('Slip')
    plt.show()

    print('Interrupted!')


# release the capture and other stuff
# ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=10)
# plt.show()

if USE_LIVE_R1:
    gs.end_process()
else:
    cap.release()
    cv2.destroyAllWindows()
if SAVE_VIDEO_FLAG:
    out.release()


'''def main():

    main_script()


if __name__ == "__main__":
    main()'''
