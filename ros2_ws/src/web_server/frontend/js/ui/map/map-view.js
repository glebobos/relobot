import { MapInteractionHandler } from './map-interaction.js';
import { MapOverlayRenderer } from './map-overlay-renderer.js';
import { MapRosAdapter } from './map-ros-adapter.js';
import { MapScene } from './map-scene.js';
import { OccupancyGridRenderer } from './occupancy-grid-renderer.js';
import { RobotVisual } from './robot-visual.js';

const ARRIVAL_DISTANCE_METERS = 0.20;
const ARRIVAL_PIN_DELAY_MS = 5000;

/**
 * Public map façade used by the control panel and interaction handler.
 * Rendering, ROS transport, robot animation, and resource ownership live in
 * focused collaborators so this class only coordinates user-facing behavior.
 */
export class MapView {
    constructor(containerId) {
        this.containerId = containerId;
        this.mapContainer = document.getElementById(containerId);
        if (!this.mapContainer) throw new Error(`Map container #${containerId} was not found.`);

        this.overlayCanvas = document.getElementById('zone-draw-canvas');
        this.overlayContext = this.overlayCanvas?.getContext('2d') || null;
        this.onPublishCommand = null;
        this.onNavigateToPoint = null;
        this.onZoneDrawModeChange = null;
        this.onNavPointModeChange = null;
        this.zoneDrawMode = false;
        this.navPointMode = false;
        this.activeZoneCorners = null;
        this.activeNavTarget = null;
        this.arrivalTimer = null;
        this.destroyed = false;

        this.scene = new MapScene(this.mapContainer, () => this.resizeOverlay());
        this.overlays = new MapOverlayRenderer(this.scene.scene);
        this.robot = new RobotVisual(this.scene.scene);
        this.occupancyGrid = new OccupancyGridRenderer(
            this.scene.scene,
            () => this.robot.position,
        );
        this.interactionHandler = new MapInteractionHandler(this);
        this.interactionHandler.init();
        this.rosAdapter = new MapRosAdapter({
            previewPath: message => this.overlays.renderPreviewPath(message),
            mapPolygon: message => this.overlays.renderMapPolygon(message),
            obstacles: message => this.overlays.renderObstacles(message),
            robotPose: message => this.handleRobotPose(message),
            map: message => this.occupancyGrid.handleMessage(message),
        });

        this.navigationSettingsHandler = () => {
            const coverageEnabled = localStorage.getItem('navigation_coverage_boundary_enabled') === 'true';
            this.rosAdapter.subscribeCoveragePolygon(coverageEnabled);
            if (!coverageEnabled) {
                this.clearMapPolygon();
            }
        };
        window.addEventListener('navigationSettingsChanged', this.navigationSettingsHandler);

        this.resizeOverlay();
        this.scene.start((dt, now) => {
            this.robot.animate(dt, now);
            this.occupancyGrid.animate(now);
            this.overlays.animate(now);
        });
    }

    handleRobotPose(message) {
        const position = message?.pose?.position;
        const quaternion = message?.pose?.orientation;
        if (![position?.x, position?.y, quaternion?.x, quaternion?.y, quaternion?.z, quaternion?.w]
            .every(Number.isFinite)) {
            console.warn('[MapView] Ignored malformed robot pose.');
            return;
        }
        this.robot.pushPose(position, quaternion);

        if (!this.activeNavTarget) return;
        const distance = Math.hypot(
            position.x - this.activeNavTarget.x,
            position.y - this.activeNavTarget.y,
        );
        if (distance < ARRIVAL_DISTANCE_METERS && !this.arrivalTimer) {
            this.arrivalTimer = setTimeout(() => this.clearNavTarget(), ARRIVAL_PIN_DELAY_MS);
        } else if (distance >= ARRIVAL_DISTANCE_METERS && this.arrivalTimer) {
            clearTimeout(this.arrivalTimer);
            this.arrivalTimer = null;
        }
    }

    resizeOverlay() {
        if (!this.overlayCanvas) return;
        const rectangle = this.mapContainer.getBoundingClientRect();
        const pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
        this.overlayCanvas.width = Math.max(1, Math.round(rectangle.width * pixelRatio));
        this.overlayCanvas.height = Math.max(1, Math.round(rectangle.height * pixelRatio));
        this.overlayCanvas.style.width = `${rectangle.width}px`;
        this.overlayCanvas.style.height = `${rectangle.height}px`;
        this.overlayContext?.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
    }

    clearOverlay() {
        if (!this.overlayContext || !this.overlayCanvas) return;
        const pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
        this.overlayContext.clearRect(
            0,
            0,
            this.overlayCanvas.width / pixelRatio,
            this.overlayCanvas.height / pixelRatio,
        );
    }

    drawOverlayRect(x1, y1, x2, y2) {
        if (!this.overlayContext) return;
        this.clearOverlay();
        this.overlayContext.strokeStyle = '#00e676';
        this.overlayContext.lineWidth = 2;
        this.overlayContext.setLineDash([6, 3]);
        this.overlayContext.strokeRect(
            Math.min(x1, x2),
            Math.min(y1, y2),
            Math.abs(x2 - x1),
            Math.abs(y2 - y1),
        );
    }

    screenToWorld(screenX, screenY) {
        return this.scene.screenToWorld(screenX, screenY);
    }

    setZoneDrawMode(enabled) {
        if (enabled && this.navPointMode) {
            this.navPointMode = false;
            this.onNavPointModeChange?.(false);
        }
        this.zoneDrawMode = Boolean(enabled);
        this.updateInteractionSurface();
        if (this.zoneDrawMode) {
            this.resizeOverlay();
            this.scene.snapTopDown();
        } else {
            this.clearOverlay();
            this.interactionHandler.reset();
            if (!this.navPointMode) this.scene.restoreCamera();
        }
        this.onZoneDrawModeChange?.(this.zoneDrawMode);
    }

    setNavPointMode(enabled) {
        if (enabled && this.zoneDrawMode) {
            this.zoneDrawMode = false;
            this.onZoneDrawModeChange?.(false);
        }
        this.navPointMode = Boolean(enabled);
        this.updateInteractionSurface();
        this.interactionHandler.reset();
        if (this.navPointMode) {
            this.resizeOverlay();
            this.clearNavTarget();
            this.scene.snapTopDown();
        } else {
            this.clearOverlay();
            if (!this.zoneDrawMode) this.scene.restoreCamera();
        }
        this.onNavPointModeChange?.(this.navPointMode);
    }

    updateInteractionSurface() {
        const active = this.zoneDrawMode || this.navPointMode;
        if (this.overlayCanvas) this.overlayCanvas.style.pointerEvents = active ? 'auto' : 'none';
        this.mapContainer.style.cursor = active ? 'crosshair' : '';
    }

    renderNavTarget(worldX, worldY, yaw) {
        this.overlays.renderTarget(worldX, worldY, yaw);
    }

    clearNavTarget() {
        this.overlays.clearTarget();
        this.activeNavTarget = null;
        if (this.arrivalTimer) clearTimeout(this.arrivalTimer);
        this.arrivalTimer = null;
    }

    commitNavPoint(worldX, worldY, yaw) {
        if (![worldX, worldY, yaw].every(Number.isFinite)) return;
        this.clearNavTarget();
        this.activeNavTarget = { x: worldX, y: worldY };
        this.renderNavTarget(worldX, worldY, yaw);
        this.onNavigateToPoint?.(worldX, worldY, yaw);

        this.navPointMode = false;
        this.interactionHandler.reset();
        this.clearOverlay();
        this.updateInteractionSurface();
        if (!this.zoneDrawMode) this.scene.restoreCamera();
        this.onNavPointModeChange?.(false);
    }

    commitZone(x1Screen, y1Screen, x2Screen, y2Screen) {
        const first = this.screenToWorld(
            Math.min(x1Screen, x2Screen),
            Math.min(y1Screen, y2Screen),
        );
        const second = this.screenToWorld(
            Math.max(x1Screen, x2Screen),
            Math.max(y1Screen, y2Screen),
        );
        if (!first || !second) return;
        const corners = [
            { x: first.x, y: first.y },
            { x: second.x, y: first.y },
            { x: second.x, y: second.y },
            { x: first.x, y: second.y },
        ];
        this.activeZoneCorners = corners;
        this.clearOverlay();
        this.overlays.renderZone(corners);
        this.onPublishCommand?.(`set_zone:${JSON.stringify(corners)}`);
        this.setZoneDrawMode(false);
    }

    startZoneDraw() {
        this.setZoneDrawMode(!this.zoneDrawMode);
        return this.zoneDrawMode;
    }

    clearZone() {
        this.overlays.renderZone(null);
        this.clearOverlay();
        this.activeZoneCorners = null;
        this.setZoneDrawMode(false);
        this.onPublishCommand?.('clear_zone');
    }

    clearMapPolygon() {
        this.overlays.clearCoverage();
    }

    startNavPoint() {
        this.setNavPointMode(!this.navPointMode);
        return this.navPointMode;
    }

    cancelNavPoint() {
        this.setNavPointMode(false);
    }

    isZoneDrawMode() {
        return this.zoneDrawMode;
    }

    hasActiveZone() {
        return this.activeZoneCorners !== null;
    }

    isNavPointMode() {
        return this.navPointMode;
    }

    destroy() {
        if (this.destroyed) return;
        this.destroyed = true;
        if (this.arrivalTimer) clearTimeout(this.arrivalTimer);
        this.arrivalTimer = null;
        this.interactionHandler.destroy();
        this.rosAdapter.destroy();
        this.occupancyGrid.destroy();
        this.robot.destroy();
        this.overlays.destroy();
        this.scene.destroy();
        if (this.navigationSettingsHandler) {
            window.removeEventListener('navigationSettingsChanged', this.navigationSettingsHandler);
        }
        this.onPublishCommand = null;
        this.onNavigateToPoint = null;
        this.onZoneDrawModeChange = null;
        this.onNavPointModeChange = null;
    }
}
