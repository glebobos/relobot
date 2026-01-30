export function initCamera() {
    const img = document.getElementById('camera-stream');
    const container = document.getElementById('video-container');
    const status = document.getElementById('camera-status');

    if (!img || !container) return;

    // Stream URL
    // Use web_video_server logic from original code
    const videoServerUrl = `http://${window.location.hostname}:8080/stream?topic=/camera/image_raw&type=mjpeg`;

    img.onload = () => {
        if(status) status.classList.add('hidden');
        console.log("[Camera] Stream connected");
    };

    img.onerror = () => {
        if(status) status.classList.remove('hidden');
        console.warn("[Camera] Stream error, retrying...");
        setTimeout(() => {
            img.src = videoServerUrl + '&t=' + Date.now();
        }, 2000);
    };

    img.src = videoServerUrl;

    // Touch Controls (Virtual Joystick)
    let isActive = false;
    let startX = 0, startY = 0;
    const CONTROL_RADIUS = 100; // Pixels

    function updateMotors(x, y) {
        fetch('/set_motors', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ x, y }),
        }).catch(err => console.error("Motor Control Error:", err));
    }

    let lastRan = 0;
    function throttleUpdateMotors(x, y) {
        const now = Date.now();
        if (now - lastRan >= 100) {
            updateMotors(x, y);
            lastRan = now;
        }
    }

    function handleStart(e) {
        // Prevent default only if interacting with video container
        if (e.target !== container && !container.contains(e.target)) return;

        isActive = true;
        const touch = e.touches ? e.touches[0] : e;
        startX = touch.clientX;
        startY = touch.clientY;
        // Prevent scrolling while driving
        if(e.cancelable) e.preventDefault();
    }

    function handleMove(e) {
        if (!isActive) return;
        const touch = e.touches ? e.touches[0] : e;
        const dx = touch.clientX - startX;
        const dy = touch.clientY - startY;

        // Normalize: x is turning (left/right), y is throttle (up/down)
        // Original code: normalizedX = dx, normalizedY = -dy
        let nx = Math.max(-1, Math.min(1, dx / CONTROL_RADIUS));
        let ny = Math.max(-1, Math.min(1, -dy / CONTROL_RADIUS)); // Invert Y because drag up is negative Y

        throttleUpdateMotors(nx, ny);
        if(e.cancelable) e.preventDefault();
    }

    function handleEnd() {
        if (!isActive) return;
        isActive = false;
        updateMotors(0, 0);
    }

    container.addEventListener('mousedown', handleStart);
    container.addEventListener('touchstart', handleStart, { passive: false });

    document.addEventListener('mousemove', handleMove);
    document.addEventListener('touchmove', handleMove, { passive: false });

    document.addEventListener('mouseup', handleEnd);
    document.addEventListener('touchend', handleEnd);
}
