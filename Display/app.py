import os
from flask import Flask, jsonify, render_template
import requests

app = Flask(__name__)

PI_DASHBOARD_URL = "http://127.0.0.1:8001/api/pose"

session = requests.Session()
session.trust_env = False

_last_good_data = {
    "global": {"heading": 0.0, "lat": 36.389, "lon": 120.445, "unknown": True},
    "local_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    "path": [],
    "status": {
        "gnss": {"lat": "Unknown", "lon": "Unknown", "online": "Unknown", "satellites": "Unknown"},
        "imu": {"frequency": "Unknown", "online": "Unknown"},
        "lidar": {"frequency": "Unknown", "online": "Unknown"},
        "mode": "Unknown",
        "network": {"signal": "Unknown", "type": "Unknown"},
        "power": {"battery": "Unknown"},
        "vision": {"fps": "Unknown", "online": "Unknown"}
    }
}

@app.route("/")
def index():
    amap_api_key = os.environ.get('AMAP_API_KEY', '').strip()
    if not amap_api_key:
        return (
            'AMAP_API_KEY is not configured. Export it in the server environment '
            'before starting the dashboard; see RELEASE_SETUP.md.',
            503,
        )
    return render_template('index.html', amap_api_key=amap_api_key)

@app.route("/api/pose")
def pose():
    global _last_good_data
    try:
        res = session.get(PI_DASHBOARD_URL, timeout=5)
        data = res.json()
        _last_good_data = data
        return jsonify(data)
    except Exception as e:
        print("pose fetch warning:", e)
        return jsonify(_last_good_data)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001, debug=True)
