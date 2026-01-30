export let ros = null;

export function connectROS() {
    if (ros) return ros;

    // Wait for ROSLIB to be loaded from CDN
    if (!window.ROSLIB) {
        console.error("ROSLIB not loaded!");
        return null;
    }

    ros = new window.ROSLIB.Ros({
        url: "ws://" + window.location.hostname + ":9090",
    });

    ros.on("connection", function () {
        console.log("Connected to websocket server.");
    });

    ros.on("error", function (error) {
        console.log("Error connecting to websocket server: ", error);
    });

    ros.on("close", function () {
        console.log("Connection to websocket server closed.");
    });

    return ros;
}

export function getROS() {
    return ros || connectROS();
}
