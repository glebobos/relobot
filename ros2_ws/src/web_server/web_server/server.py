"""
High-level glue between ROS, domain controllers and Flask UI.
"""

import logging
import rclpy  
import os
import threading
import time
from typing import Dict, Any

from flask import Flask, Response, jsonify, render_template, request, send_from_directory
from flask.logging import default_handler

from .config import ControllerConfig
from .knife_controller import KnifeController
from .motor_controller import MotorController
from .pid_controller import PIDController
from .ros_node import RobotROSNode

logger = logging.getLogger(__name__)


class RobotWebServer:

    def __init__(self) -> None:
        # ---------------- configuration ---------------- #
        self._cfg_path = os.path.join(
            os.path.dirname(__file__), "controller_config.json")
        self.cfg = ControllerConfig.load_from_file(self._cfg_path)

        # ---------------- ROS layer --------------------- #
        self.node = RobotROSNode()

        # ---------------- domain logic ------------------ #
        self.knife = KnifeController(self.cfg, self.node.publish_rpm)
        self.motors = MotorController(self.cfg, self.node.publish_twist)
        self.pid = PIDController(self.node.toggle_pid)

        # ---------------- Flask app --------------------- #
        self.app = Flask(__name__, template_folder="./templates")
        if os.environ.get("ENABLE_FLASK_LOGS", "").lower() not in ("true", "1"):
            self.app.logger.removeHandler(default_handler)
            logging.getLogger("werkzeug").setLevel(logging.ERROR)

        self._register_routes()

    # --------------------------------------------------------------------- #
    #                           Flask routes                                #
    # --------------------------------------------------------------------- #
    def _register_routes(self) -> None:

        @self.app.route("/")
        def index():
            return render_template("index.html")

        @self.app.route("/robot-control.js")
        def js():
            return send_from_directory("templates", "robot-control.js",
                                       mimetype="application/javascript")

        @self.app.route("/styles.css")
        def css():
            return send_from_directory("templates", "styles.css",
                                       mimetype="text/css")

        # ------------- video ------------------------------------------------
        @self.app.route("/video_feed/depth")
        def vid_depth():
            return Response(self._stream(self.node.depth_frame),
                            mimetype="multipart/x-mixed-replace; boundary=frame")

        @self.app.route("/video_feed/confidence")
        def vid_conf():
            return Response(self._stream(self.node.confidence_frame),
                            mimetype="multipart/x-mixed-replace; boundary=frame")

        # ------------- voltage ----------------------------------------------
        @self.app.route("/api/voltage")
        def api_voltage():
            vin = self.node.latest_voltage()
            if vin is None:
                return jsonify(success=False, message="No data yet"), 503
            return jsonify(success=True, vin=vin)

        @self.app.route("/stream/voltage")
        def sse_voltage():
            def gen():
                last = None
                while True:
                    vin = self.node.latest_voltage()
                    if vin is not None and vin != last:
                        yield f"data: {vin}\n\n"
                        last = vin
                    time.sleep(0.2)
            return Response(gen(), mimetype="text/event-stream")

        # ------------- gamepad ----------------------------------------------
        @self.app.route("/gamepad", methods=["POST"])
        def gamepad():
            try:
                data: Dict[str, Any] = request.get_json(force=True, silent=True) or {}
                axes, buttons = data.get("axes", []), data.get("buttons", [])

                if len(axes) <= max(self.cfg.turn_axis, self.cfg.drive_axis):
                    return jsonify(success=False, message="Missing axis data")
                if len(buttons) <= max(self.cfg.knife_button, self.cfg.pid_toggle_button):
                    return jsonify(success=False, message="Missing button data")

                self.motors.process_joystick(
                    axes[self.cfg.turn_axis], axes[self.cfg.drive_axis])

                self.knife.process_button_value(buttons[self.cfg.knife_button])

                if buttons[self.cfg.pid_toggle_button] == 1:
                    self.pid.toggle()

                return jsonify(success=True)
            except Exception as exc:                                      # pragma: no cover
                logger.exception("Gamepad route failed: %s", exc)
                return jsonify(success=False, error=str(exc))

        # ------------- direct XY --------------------------------------------
        @self.app.route("/set_motors", methods=["POST"])
        def set_motors():
            try:
                x, y = float(request.json["x"]), float(request.json["y"])
                self.motors.process_xy_input(x, y)
                return jsonify(success=True)
            except Exception as exc:
                logger.exception("set_motors failed")
                return jsonify(success=False, error=str(exc))

        # ------------- config -----------------------------------------------
        @self.app.route("/api/config", methods=["GET"])
        def get_cfg():
            return jsonify(success=True, config=self.cfg.to_dict())

        @self.app.route("/api/config", methods=["POST"])
        def set_cfg():
            new = request.get_json(force=True, silent=True) or {}
            merged = {**self.cfg.to_dict(), **new}
            updated = ControllerConfig.from_dict(merged)
            ok, msg = updated.validate()
            if not ok:
                return jsonify(success=False, message=msg)
            self.cfg = updated
            self.motors.cfg = updated
            self.knife.cfg = updated
            self.cfg.save_to_file(self._cfg_path)
            return jsonify(success=True, config=self.cfg.to_dict())

        # ------------- e-stop -----------------------------------------------
        @self.app.route("/emergency_stop", methods=["POST"])
        def estop():
            self.motors.stop()
            return jsonify(success=True, message="Motors halted")

    # --------------------------------------------------------------------- #
    #                          helpers & lifecycle                          #
    # --------------------------------------------------------------------- #
    @staticmethod
    def _stream(frame_supplier):
        def gen():
            while True:
                frame = frame_supplier()
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" +
                       frame + b"\r\n")
                time.sleep(1 / 30)
        return gen()

    def start(self) -> None:
        # start knife RPM loop
        self.knife.start()

        # start Flask in its own thread so rclpy.spin blocks main
        threading.Thread(target=lambda: self.app.run(
            host="0.0.0.0", port=80, threaded=True, debug=False),
            name="FlaskThread", daemon=True).start()

        logger.info("WebServer up – spinning ROS ...")
        rclpy.spin(self.node)      # blocks until Ctrl-C / kill signal

    def shutdown(self) -> None:
        logger.info("Shutting down server")
        self.knife.stop()
        self.motors.stop()
        self.node.destroy_node()
