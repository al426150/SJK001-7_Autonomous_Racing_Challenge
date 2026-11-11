# The first step to do this program is to import all
# the needed libraries. 
import WebGUI
import HAL
import cv2
# --- PD Controller Gains ---
kp = 0.011      # Proportional gain: determines steering responsiveness relative to error.
kd = 0.035     # Derivative gain: provides damping to suppress oscillations.
ki = 0.00001	     # Initial integral gain.

# Initialize variables outside the main loop
i=0
prev_err = 0
integral = 0
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
        # Current error relative to the horizontal center of the frame (640 / 2 = 320)
        err = 320 - cX 
        # Error accumulation and limitation of integral's value
        integral += err
        integral = max(-1000, min(1000, integral))
        # Discrete derivative: rate of change of the error
        d_err = err - prev_err
        # PD control law for angular velocity
        setV = max(5.5, 11 - 0.015 * abs(err))
        # speed_factor is used to adjust the intensity of the turn based on the speed.
        speed_factor = 5.0 / setV
        setW = (kp * err + ki * integral + kd * d_err) * speed_factor
        # setW = (kp * err) + (kd * d_err)
        # Actuator commands for static version
        # HAL.setV(6.0)
        # HAL.setW(setW)
        # Dynamic version
        # setV = max(5.5, 7.8 - 0.015 * abs(err))
        
        HAL.setV(setV)       
        HAL.setW(setW)

        # Store current error for the next cycle
        prev_err = err
    else:
        # Failsafe fallback if the line is lost
        HAL.setV(4.0)

    # WebGUI.showImage(red_mask)
    print('%d cX: %.2f cY: %.2f' %(i,cX,cY))
    i = i+1