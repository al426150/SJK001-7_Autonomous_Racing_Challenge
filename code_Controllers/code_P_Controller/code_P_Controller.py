# The first step to do this program is to import all
# the needed libraries. 

import WebGUI
import HAL
import cv2

# Initialize variables outside the main loop
i=0
while True:
    # Convert image to HSV and create a binary mask for red color detection
    img = HAL.getImage()
    WebGUI.showImage(img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv, (0,125,125), (30, 255, 255))

    # Calculate image moments to determine the centroid of the line
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # M = cv2.moments(contours[0])
    # Calculate geometric moments of the binary mask to locate the center of mass
    M = cv2.moments(red_mask)
    if M["m00"] != 0:
        cX = M["m10"] / M["m00"]
        cY = M["m01"] / M["m00"]
    else:
        # Fallback to zero if the red line is completely out of frame
        cX, cY = 0,0

    if cX > 0:
        # Compute horizontal tracking error relative to target setpoint (center line)
        err = 320 - cX 
        # Send velocity commands to the vehicle
        HAL.setV(5.7)
        # Proportional control law for steering
        HAL.setW(0.015 * err)

    # Visual Feedback & Telemetry Logging
    # Display the processed binary mask in the viewer for threshold validation
    # WebGUI.showImage(red_mask)
    print('%d cX: %.2f cY: %.2f' %(i,cX,cY))
    i = i+1