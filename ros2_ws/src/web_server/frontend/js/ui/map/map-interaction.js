export class MapInteractionHandler {
    constructor(mapView) {
        this.mapView = mapView;
        this.overlayCanvas = mapView.overlayCanvas;

        // Interaction state
        this.zoneDrawing = false;
        this.zoneDragStart = null;
        this.navAnchor = null;

        // Bind event handlers
        this.startHandler = (e) => this.handleStart(e);
        this.moveHandler = (e) => this.handleMove(e);
        this.endHandler = (e) => this.handleEnd(e);
    }

    init() {
        if (!this.overlayCanvas) return;

        // Mouse listeners
        this.overlayCanvas.addEventListener('mousedown', this.startHandler);
        this.overlayCanvas.addEventListener('mousemove', this.moveHandler);
        this.overlayCanvas.addEventListener('mouseup', this.endHandler);

        // Touch listeners (non-passive to allow e.preventDefault() for scrolling suppression)
        this.overlayCanvas.addEventListener('touchstart', this.startHandler, { passive: false });
        this.overlayCanvas.addEventListener('touchmove', this.moveHandler, { passive: false });
        this.overlayCanvas.addEventListener('touchend', this.endHandler, { passive: false });
    }

    reset() {
        this.zoneDrawing = false;
        this.zoneDragStart = null;
        this.navAnchor = null;
    }

    handleStart(e) {
        const t = e.touches ? e.touches[0] : e;
        if (this.mapView.navPointMode) {
            const world = this.mapView.screenToWorld(t.clientX, t.clientY);
            if (!world) return;
            this.navAnchor = { screenX: t.clientX, screenY: t.clientY, worldX: world.x, worldY: world.y };
            this.mapView.renderNavTarget(world.x, world.y);
            e.preventDefault();
            return;
        }
        if (!this.mapView.zoneDrawMode) return;
        this.zoneDrawing = true;
        this.zoneDragStart = { x: t.clientX, y: t.clientY };
        e.preventDefault();
    }

    handleMove(e) {
        const t = e.touches ? e.touches[0] : e;
        if (this.mapView.navPointMode && this.navAnchor) {
            const endWorld = this.mapView.screenToWorld(t.clientX, t.clientY);
            if (endWorld) {
                const yaw = Math.atan2(endWorld.y - this.navAnchor.worldY, endWorld.x - this.navAnchor.worldX);
                this.mapView.renderNavTarget(this.navAnchor.worldX, this.navAnchor.worldY, yaw);
            }
            e.preventDefault();
            return;
        }
        if (!this.mapView.zoneDrawMode || !this.zoneDrawing || !this.zoneDragStart) return;
        const r = this.mapView.mapContainer.getBoundingClientRect();
        this.mapView.drawOverlayRect(
            this.zoneDragStart.x - r.left, this.zoneDragStart.y - r.top,
            t.clientX - r.left, t.clientY - r.top
        );
        e.preventDefault();
    }

    handleEnd(e) {
        const t = e.changedTouches ? e.changedTouches[0] : e;
        if (this.mapView.navPointMode && this.navAnchor) {
            const endWorld = this.mapView.screenToWorld(t.clientX, t.clientY);
            let yaw = 0;
            if (endWorld) {
                const ddx = endWorld.x - this.navAnchor.worldX;
                const ddy = endWorld.y - this.navAnchor.worldY;
                const sdx = t.clientX - this.navAnchor.screenX;
                const sdy = t.clientY - this.navAnchor.screenY;
                if (sdx * sdx + sdy * sdy > 100) {
                    yaw = Math.atan2(ddy, ddx);
                }
            }
            this.mapView.commitNavPoint(this.navAnchor.worldX, this.navAnchor.worldY, yaw);
            e.preventDefault();
            return;
        }
        if (!this.mapView.zoneDrawMode || !this.zoneDrawing || !this.zoneDragStart) return;
        this.zoneDrawing = false;
        const dx = Math.abs(t.clientX - this.zoneDragStart.x);
        const dy = Math.abs(t.clientY - this.zoneDragStart.y);
        if (dx > 10 && dy > 10) {
            this.mapView.commitZone(this.zoneDragStart.x, this.zoneDragStart.y, t.clientX, t.clientY);
        } else {
            this.mapView.clearOverlay();
        }
        this.zoneDragStart = null;
        e.preventDefault();
    }

    destroy() {
        if (!this.overlayCanvas) return;
        this.overlayCanvas.removeEventListener('mousedown', this.startHandler);
        this.overlayCanvas.removeEventListener('mousemove', this.moveHandler);
        this.overlayCanvas.removeEventListener('mouseup', this.endHandler);
        this.overlayCanvas.removeEventListener('touchstart', this.startHandler);
        this.overlayCanvas.removeEventListener('touchmove', this.moveHandler);
        this.overlayCanvas.removeEventListener('touchend', this.endHandler);
    }
}
