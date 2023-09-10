# From Python
# It requires OpenCV installed for Python
import sys
import cv2
import os
import zmq
from sys import platform
import argparse

context = zmq.Context()

# Turn on Server
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

def display(datums):
    datum = datums[0]
    cv2.imshow("OpenPose 1.7.0 - Python API", datum.cvOutputData)
    key = cv2.waitKey(1)
    return (key == 27)


def printKeypoints(datums):
    datum = datums[0]
    print("Body keypoints: \n" + str(datum.poseKeypoints))

def verifyCheckStart(datums):
    datum = datums[0]
    if datum.poseKeypoints is None:
        return
    for person in datum.poseKeypoints:
        for keypoint in person:
            keypointX = keypoint[0]
            if keypointX != 0 and keypointX <= 0.5:
                #print("Arm check not started")
                return False
    return True

def checkArmExtendedLeft(datums, left_id):
    datum = datums[0]
    if datum.poseKeypoints is None:
        return
    person = datum.poseKeypoints[left_id]
    for keypoint in person:
        keypointX = keypoint[0]
        if keypointX != 0 and keypointX >= 0.40:
            return True
    return False

def checkArmRetractedLeft(datums, left_id):
    datum = datums[0]
    if datum.poseKeypoints is None:
        return
    person = datum.poseKeypoints[left_id]
    for keypoint in person:
        keypointX = keypoint[0]
        if keypointX != 0 and keypointX >= 0.40:
            return False
    return True

def checkArmExtendedRight(datums, right_id):
    datum = datums[0]
    if datum.poseKeypoints is None:
        return
    person = datum.poseKeypoints[right_id]
    for keypoint in person:
        keypointX = keypoint[0]
        if keypointX != 0 and keypointX <= 0.60:
            return True
    return False

def checkArmRetractedRight(datums, right_id):
    datum = datums[0]
    if datum.poseKeypoints is None:
        return
    person = datum.poseKeypoints[right_id]
    for keypoint in person:
        keypointX = keypoint[0]
        if keypointX != 0 and keypointX <= 0.60:
            return False
    return True

def defineIdPos(datums):
    datum = datums[0]
    if datum.poseKeypoints is None:
        return
    if len(datum.poseKeypoints) == 1:
        return (0,-1)
    if len(datum.poseKeypoints) > 2:
        return (0,-2)
    p0 = datum.poseKeypoints[0]
    p1 = datum.poseKeypoints[1]
    for keypointP0 in p0:
        if keypointP0[0] != 0:
            p0X = keypointP0[0]
            break
    for keypointP1 in p1:
        if keypointP1[0] != 0:
            p1X = keypointP1[0]
            break
    if p0X < p1X:
        return (0, 1)
    else:
        return (1, 0)


try:
    # Import Openpose (Windows/Ubuntu/OSX)
    dir_path = os.path.dirname(os.path.realpath(__file__))
    try:
        # Windows Import
        if platform == "win32":
            # Change these variables to point to the correct folder (Release/x64 etc.)
            sys.path.append(dir_path + '/../../python/openpose/Release');
            os.environ['PATH']  = os.environ['PATH'] + ';' + dir_path + '/../../x64/Release;' +  dir_path + '/../../bin;'
            import pyopenpose as op
        else:
            # Change these variables to point to the correct folder (Release/x64 etc.)
            sys.path.append('../../python');
            # If you run `make install` (default path is `/usr/local/python` for Ubuntu), you can also access the OpenPose/python module from there. This will install OpenPose and the python library at your desired installation path. Ensure that this is in your python path in order to use it.
            # sys.path.append('/usr/local/python')
            from openpose import pyopenpose as op
    except ImportError as e:
        print('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
        raise e


    # Flags
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-display", action="store_true", help="Disable display.")
    args = parser.parse_known_args()

    # Custom Params (refer to include/openpose/flags.hpp for more parameters)
    params = dict()
    params["model_folder"] = "../../../models/"
    params["net_resolution"] = "192x144"
    params["keypoint_scale"] = "3"
    params["camera"] = 2

    # Add others in path?
    for i in range(0, len(args[1])):
        curr_item = args[1][i]
        if i != len(args[1])-1: next_item = args[1][i+1]
        else: next_item = "1"
        if "--" in curr_item and "--" in next_item:
            key = curr_item.replace('-','')
            if key not in params:  params[key] = "1"
        elif "--" in curr_item and "--" not in next_item:
            key = curr_item.replace('-','')
            if key not in params: params[key] = next_item

    # Construct it from system arguments
    # op.init_argv(args[1])
    # oppython = op.OpenposePython()

    # Starting OpenPose
    opWrapper = op.WrapperPython(op.ThreadManagerMode.AsynchronousOut)
    opWrapper.configure(params)
    opWrapper.start()

    while True:
         
        turn_order_aux = socket.recv()
        print("Message received:", turn_order_aux)
        socket.send_string("Order Received")
        turn_order = turn_order_aux.decode()
        print(turn_order)
        turn_n = 0        

        # Main loop
        userWantsToExit = False
        checkStatus = 0
        buffer = []
        buffer_len = 0
        buffer_it = 0
        config_finished = False

        while not userWantsToExit:
            # Pop frame
            datumProcessed = op.VectorDatum()
            if opWrapper.waitAndPop(datumProcessed):
                if not args[0].no_display:
                    # Display image
                    userWantsToExit = display(datumProcessed)
                #printKeypoints(datumProcessed)

                idPos = defineIdPos(datumProcessed)
                
                if idPos is None:
                    print("0 people detected")
                    continue

                if idPos[1] == -1:
                    print("Only 1 person detected")
                    continue
                
                if idPos[1] == -2:
                    print("More than 2 people detected")
                    continue
                
                left_id = idPos[0]
                right_id = idPos[1]

                if turn_n >= len(turn_order)-1:
                    config_finished = True

                elif turn_order[turn_n] == 'b':
                    if checkStatus == 0:
                        if checkArmExtendedLeft(datumProcessed, left_id) == True:
                            action_flag = True
                            buffer.append("Left: Arm Extended")
                            buffer_len += 1
                            checkStatus = 1
                            print("Left: Arm Extended")
                    elif checkStatus == 1:
                        if checkArmRetractedLeft(datumProcessed, left_id) == True:
                            action_flag = True
                            buffer.append("Left: Arm Retracted")
                            buffer_len += 1
                            checkStatus = 0
                            turn_n += 1
                            print("Left: Arm Retracted")

                elif turn_order[turn_n] == 'p':
                    if checkStatus == 0:
                        if checkArmExtendedRight(datumProcessed, right_id) == True:
                            action_flag = True
                            buffer.append("Right: Arm Extended")
                            buffer_len += 1
                            checkStatus = 1
                            print("Right: Arm Extended")
                    elif checkStatus == 1:
                        if checkArmRetractedRight(datumProcessed, right_id) == True:
                            action_flag = True
                            buffer.append("Right: Arm Extended")
                            buffer_len += 1
                            checkStatus = 0
                            turn_n += 1
                            print("Right: Arm Retracted")
                
                elif turn_order[turn_n] == 'y':
                    turn_n += 1
                
                if buffer_it < buffer_len or config_finished == True:
                    try:
                        #check for a message, this will not block
                        message = socket.recv(flags=zmq.NOBLOCK)

                        #a message has been received
                        print("Message received:", message)
                        
                        if message.decode() == "Configuration finished":
                            print("Configuration finished. Waiting for next configuration...")
                            socket.send_string("Received")
                            break
                        
                        socket.send_string(buffer[buffer_it])
                        buffer_it += 1
                        action_flag = False

                    except zmq.Again as e:
                        pass
            else:
                break
        
except Exception as e:
    print(e)
    sys.exit(-1)