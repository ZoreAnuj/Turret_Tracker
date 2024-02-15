import cv2
import mediapipe as mp
import RPi.GPIO as GPIO
from time import sleep

# Setup GPIO
pan_pin = 11
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pan_pin, GPIO.OUT)
pwm = GPIO.PWM(pan_pin, 50)
pwm.start(0)

tilt_pin = 13
GPIO.setmode(GPIO.BOARD)
GPIO.setup(tilt_pin, GPIO.OUT)
pwm1 = GPIO.PWM(tilt_pin, 50)
pwm1.start(0)
new_ver = 0
previous_dif = 0
mag = 0

def setAngle(angle, pwm):
    duty = angle / 18 + 2
    pwm.ChangeDutyCycle(duty)
    sleep(0.5)  # Adjust sleep time as needed
    
def gradualSetAngle(target_angle, delay, step=1):
    global previous_pan  # Assume this holds the current angle of the servo
    steps = range(previous_pan, target_angle, step if target_angle > previous_pan else -step)
    for angle in steps:
        duty = angle / 18 + 2
        pwm.ChangeDutyCycle(duty)
        sleep(delay)
    pwm.ChangeDutyCycle(target_angle / 18 + 2)
    previous_pan = target_angle

# Initialize mediapipe pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

cap = cv2.VideoCapture(0)
selected_landmark_idx = 11 
previous_pan = 120
kp=0.02
new = 120
previous_tilt = 120
setAngle(previous_tilt,pwm1)
setAngle(previous_pan,pwm)


try:
    for frame in range(1000):  # Limit the number of iterations for testing
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue
        if success and frame%2==0 :
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = pose.process(image_rgb)
            if results.pose_landmarks:
                landmarks = results.pose_landmarks.landmark
                landmark_x = int(landmarks[selected_landmark_idx].x * image.shape[1])
                landmark_y = int(landmarks[selected_landmark_idx].y * image.shape[0])
                #cv2.circle(image, (landmark_x, landmark_y), 7, (0, 0, 255), -1)
                #cv2.circle(image, (320,240), 7, (0, 255, 0), -1)
                #cv2.imshow('Output', image)
                #cv2.waitKey(3)
                
                diff = abs(landmark_x-320)
                
                diff_vertical = abs(landmark_y-240)
	  
				#mag = max(int(diff/45),1)
                
                if frame == 0: previous_dif = diff
				
                mag= abs(diff*kp)
                
                #mag_ver = abs(diff_vertical*kp)
                
                #if diff > 120: 
                    
                    
                if landmark_x < 325 and landmark_x>315: 
                    
                    new = previous_pan
                    continue
            
                if landmark_x > 330: new = max(0, previous_pan - mag) 
                
                if landmark_x < 315:  new = min(180, previous_pan + mag)
                
                
                
                #if landmark_y >260: new_ver = max(0, previous_tilt - mag) 
                
                #elif landmark_y < 220:  new_ver = min(180, previous_tilt + mag)
                
                new = int(new)
                #new_ver = int(new_ver)
              
                #print(f'Current angle = {(new,new_ver)} {(landmark_x, landmark_y)} step_size = {(mag,mag_ver)} diff = {(diff,diff_vertical)} delay = {(min(0.025,abs(0.025-(2*diff/(480*8)))), min(0.025,abs(0.025-(2*diff_vertical/(320*8)))))}')
                #setAngle(new)
                
                print(f'Current angle = {(new,new_ver)} {(landmark_x, landmark_y)} step_size = {(mag)} diff = {(diff)} delay = {min(0.025,abs(0.025-(2*diff_vertical/(320*8))))}')
                #gradualSetAngle((new), step=1, delay=min(0.04,abs(0.04-(2*diff/(480*8)))))
                gradualSetAngle((new), step=1, delay=(0.02))
                #gradualSetAngle((new_ver), step=1, delay=0.4)
                
                previous_pan = int(new)
                previous_tilt = int(new_ver)
                #setAngle(90,pwm1)
                
                cv2.waitKey(500)
				
            else: setAngle(120,pwm)
			

finally:
    # Cleanup
    setAngle(120,pwm)  # Reset to default position
    pwm.stop()
    pwm1.stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()
