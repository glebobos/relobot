export class DynamicLine {
    constructor(color, maxPoints = 5000, thickness = 1) {
        this.maxPoints = maxPoints;
        this.geometry = new THREE.BufferGeometry();
        this.positions = new Float32Array(maxPoints * 3);
        this.positionAttr = new THREE.BufferAttribute(this.positions, 3);
        this.positionAttr.setDynamic(true);
        this.geometry.addAttribute('position', this.positionAttr);
        this.geometry.setDrawRange(0, 0);

        this.material = new THREE.LineBasicMaterial({
            color: color,
            linewidth: thickness
        });

        this.line = new THREE.Line(this.geometry, this.material);
    }

    updatePoints(points) {
        const count = Math.min(points.length, this.maxPoints);
        const array = this.positionAttr.array;
        for (let i = 0; i < count; i++) {
            const p = points[i];
            array[i * 3] = p.x;
            array[i * 3 + 1] = p.y;
            array[i * 3 + 2] = p.z !== undefined ? p.z : 0.0;
        }
        this.positionAttr.needsUpdate = true;
        this.geometry.setDrawRange(0, count);
        this.line.visible = count > 0;
    }

    clear() {
        this.geometry.setDrawRange(0, 0);
        this.line.visible = false;
    }

    dispose() {
        this.geometry.dispose();
        this.material.dispose();
    }
}
