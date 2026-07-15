/**
 * Dispose all GPU-backed resources owned by a Three.js object tree.
 * Shared resources may be excluded by passing them in `preserve`.
 */
export function disposeObjectTree(root, preserve = new Set()) {
    if (!root) return;
    root.traverse((object) => {
        if (object.geometry && !preserve.has(object.geometry)) {
            object.geometry.dispose();
        }
        const materials = Array.isArray(object.material)
            ? object.material
            : object.material ? [object.material] : [];
        materials.forEach((material) => disposeMaterial(material, preserve));
    });
}

export function disposeMaterial(material, preserve = new Set()) {
    if (!material || preserve.has(material)) return;
    Object.values(material).forEach((value) => {
        if (value?.isTexture && !preserve.has(value)) value.dispose();
    });
    material.dispose();
}

export function detachAndDispose(parent, object, preserve = new Set()) {
    if (!object) return;
    if (parent) parent.remove(object);
    disposeObjectTree(object, preserve);
}
