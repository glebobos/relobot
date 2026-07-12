export class TfTransformer {
    constructor() {
        this.transforms = {}; // map of parent_frame -> child_frame -> THREE.Matrix4
        this._cache = {};     // cache of resolved transforms: "parent>child" -> THREE.Matrix4
        this._dirty = false;  // flag: true when transforms have been updated since last cache build
    }

    /**
     * Update the cached transforms from a TFMessage.
     * @param {Object} tfMessage - Message of type tf2_msgs/msg/TFMessage
     */
    update(tfMessage) {
        if (!tfMessage || !tfMessage.transforms) return;
        
        for (const t of tfMessage.transforms) {
            const parent = this.cleanFrameId(t.header.frame_id);
            const child = this.cleanFrameId(t.child_frame_id);
            
            if (!parent || !child) continue;

            if (!this.transforms[parent]) {
                this.transforms[parent] = {};
            }

            const trans = t.transform.translation;
            const rot = t.transform.rotation;
            
            // Build transformation matrix using Three.js (which is globally available)
            const matrix = new THREE.Matrix4().compose(
                new THREE.Vector3(trans.x, trans.y, trans.z),
                new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w),
                new THREE.Vector3(1, 1, 1)
            );
            
            this.transforms[parent][child] = matrix;
        }

        // Invalidate the resolved-transform cache after any update
        this._dirty = true;
    }

    /**
     * Strip leading/trailing slashes and trim frame IDs.
     */
    cleanFrameId(id) {
        if (!id) return '';
        return id.trim().replace(/^\/+/, '').replace(/\/+$/, '');
    }

    /**
     * Compute the composite transformation matrix from parent to child frame.
     * Uses a cache to avoid BFS on every call when the tree hasn't changed.
     * @param {string} parentFrame - The source/parent frame ID (e.g. 'map')
     * @param {string} childFrame - The destination/child frame ID (e.g. 'base_link')
     * @returns {THREE.Matrix4|null} The transformation matrix or null if no path exists.
     */
    getTransform(parentFrame, childFrame) {
        const parent = this.cleanFrameId(parentFrame);
        const child = this.cleanFrameId(childFrame);

        if (parent === child) {
            return new THREE.Matrix4();
        }

        const cacheKey = parent + '>' + child;

        // Return cached result if the tree hasn't changed since last resolve
        if (!this._dirty && this._cache[cacheKey]) {
            return this._cache[cacheKey];
        }

        // BFS to find a path from parent to child in the TF tree
        const queue = [{ frame: parent, matrix: new THREE.Matrix4() }];
        const visited = new Set([parent]);

        while (queue.length > 0) {
            const { frame, matrix } = queue.shift();
            if (frame === child) {
                this._cache[cacheKey] = matrix;
                this._dirty = false;
                return matrix;
            }

            const children = this.transforms[frame];
            if (children) {
                for (const nextFrame in children) {
                    if (!visited.has(nextFrame)) {
                        visited.add(nextFrame);
                        const nextMatrix = matrix.clone().multiply(children[nextFrame]);
                        queue.push({ frame: nextFrame, matrix: nextMatrix });
                    }
                }
            }
        }

        // No path found — cache null to avoid repeated BFS failures
        this._cache[cacheKey] = null;
        this._dirty = false;
        return null;
    }
}
