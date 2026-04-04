#!/usr/bin/env python3
"""ECHO-Q Flask web dashboard — live camera + telemetry."""
import json, threading
import rospy, cv2, numpy as np
from flask import Flask, render_template, Response, jsonify, request
from sensor_msgs.msg    import CompressedImage
from std_msgs.msg       import String
from geometry_msgs.msg  import Twist

app = Flask(__name__,
    template_folder='../templates',
    static_folder='../static')

_lock = threading.Lock(); _frame = None; _state = {}; _cmd_pub = None

def _cb_cam(msg):
    global _frame
    arr = np.frombuffer(msg.data, np.uint8)
    with _lock: _frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

def _cb_state(msg):
    with _lock:
        try: _state.update(json.loads(msg.data))
        except: pass

def _gen():
    ph = _placeholder()
    while True:
        with _lock: f = _frame
        if f is not None:
            _, buf = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 80])
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n'
        else:
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + ph + b'\r\n'

def _placeholder():
    img = np.zeros((360,640,3), dtype=np.uint8)
    cv2.putText(img, 'ECHO-Q | No Camera Feed', (140,180),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,120), 2)
    _, buf = cv2.imencode('.jpg', img); return buf.tobytes()

@app.route('/') 
def index(): return render_template('index.html')

@app.route('/video_feed')
def video_feed(): return Response(_gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/state')
def api_state():
    with _lock: return jsonify(dict(_state))

@app.route('/api/cmd', methods=['POST'])
def api_cmd():
    if not _cmd_pub: return jsonify({'error':'not ready'}), 503
    d = request.get_json(force=True, silent=True) or {}
    msg = Twist(); msg.linear.x = float(d.get('vx',0)); msg.angular.z = float(d.get('omega',0))
    _cmd_pub.publish(msg); return jsonify({'status':'ok'})

if __name__ == '__main__':
    global _cmd_pub
    rospy.init_node('echo_q_web', anonymous=True, disable_signals=True)
    rospy.Subscriber('/camera/image/compressed', CompressedImage, _cb_cam)
    rospy.Subscriber('/echo_q/state', String, _cb_state)
    _cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    threading.Thread(target=rospy.spin, daemon=True).start()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
