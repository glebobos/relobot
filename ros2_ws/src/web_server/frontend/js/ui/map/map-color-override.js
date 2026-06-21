/**
 * Install a custom colour-mapping override for ROS3D.OccupancyGrid.
 * Must be called AFTER ros3d.js is available on window.ROS3D.
 *
 * Colour strategy for a lawn-robot occupancy map:
 *   - Unknown (-1) : fully transparent  → dark/grass background shows through
 *   - Free   (0)   : fully transparent  → grass texture shows through
 *   - Occupied (>0): dark charcoal      → rendered as solid wall
 */
export function installMapColorOverride() {
    if (!window.ROS3D || !window.ROS3D.OccupancyGrid) return;

    ROS3D.OccupancyGrid.prototype.getColor = function(index, row, col, value) {
        if (value === 0) {
            // Free / explored space — fully transparent (grass shows through)
            return [0, 0, 0, 0];
        }
        if (value < 0) {
            // Unknown / undiscovered — dark semi-transparent fog
            return [10, 12, 18, 200];
        }
        // Occupied (walls, obstacles) — solid dark charcoal
        const darkness = Math.max(0, 255 - Math.round((value / 100) * 220));
        return [darkness, darkness + 4, darkness + 6, 255];
    };
}
