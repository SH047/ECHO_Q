#!/usr/bin/env python3
from flask import Flask, render_template, Response
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

app = Flask(__name__)

# Global variable to store the latest camera frame
current_frame = None

def camera_callback(msg):
    global current_frame
    # Convert ROS CompressedImage to OpenCV format
    np_arr = np.fromstring(msg.data, np.uint8)
    current_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def generate_frames():
    global current_frame
    while True:
        if current_frame is not None:
            # Encode the frame as JPEG
            ret, buffer = cv2.imencode('.jpg', current_frame)
            frame = buffer.tobytes()
            
            # Yield the frame in MJPEG format (standard for video streaming)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            # If no camera, yield nothing or a placeholder
            pass

@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Initialize ROS Node
    rospy.init_node('echo_q_web_interface', anonymous=True)
    
    # Subscribe to the Raspberry Pi Camera topic
    # (Make sure your pi_cam node publishes to /camera/image/compressed)
    rospy.Subscriber("/camera/image/compressed", CompressedImage, camera_callback)
    
    rospy.loginfo("Starting Web Interface on port 5000...")
    
    # Run Flask App
    # host='0.0.0.0' allows access from other devices (like your phone)
    app.run(host='0.0.0.0', port=5000, debug=False)
