
/**
 * Utility to parse URDF XML strings and build a Three.js 3D model.
 * Complies with ROS coordinates (X=forward, Y=left, Z=up) and handles
 * joint hierarchies without requiring tf2_web_republisher.
 */

export function parseUrdf(urdfString) {
    const parser = new DOMParser();
    const xmlDoc = parser.parseFromString(urdfString, 'text/xml');
    
    const links = {};
    const joints = [];
    
    // Parse links
    const linkNodes = xmlDoc.getElementsByTagName('link');
    for (let i = 0; i < linkNodes.length; i++) {
        const linkNode = linkNodes[i];
        const name = linkNode.getAttribute('name');
        
        const visuals = [];
        const visualNodes = linkNode.getElementsByTagName('visual');
        for (let j = 0; j < visualNodes.length; j++) {
            const visualNode = visualNodes[j];
            
            // Origin
            const originNode = visualNode.getElementsByTagName('origin')[0];
            const xyzStr = originNode ? originNode.getAttribute('xyz') : '0 0 0';
            const rpyStr = originNode ? originNode.getAttribute('rpy') : '0 0 0';
            
            const xyz = xyzStr.split(/\s+/).filter(Boolean).map(Number);
            const rpy = rpyStr.split(/\s+/).filter(Boolean).map(Number);
            
            // Geometry
            const geomNode = visualNode.getElementsByTagName('geometry')[0];
            let geometryType = null;
            let geometryParams = {};
            
            if (geomNode) {
                const boxNode = geomNode.getElementsByTagName('box')[0];
                const cylNode = geomNode.getElementsByTagName('cylinder')[0];
                const sphereNode = geomNode.getElementsByTagName('sphere')[0];
                
                if (boxNode) {
                    geometryType = 'box';
                    const sizeAttr = boxNode.getAttribute('size') || '0 0 0';
                    geometryParams = {
                        size: sizeAttr.split(/\s+/).filter(Boolean).map(Number)
                    };
                } else if (cylNode) {
                    geometryType = 'cylinder';
                    geometryParams = {
                        radius: Number(cylNode.getAttribute('radius') || 0),
                        length: Number(cylNode.getAttribute('length') || 0)
                    };
                } else if (sphereNode) {
                    geometryType = 'sphere';
                    geometryParams = {
                        radius: Number(sphereNode.getAttribute('radius') || 0)
                    };
                }
            }
            
            // Material
            const matNode = visualNode.getElementsByTagName('material')[0];
            const matName = matNode ? matNode.getAttribute('name') : null;
            let matColor = null;
            if (matNode) {
                const colorNode = matNode.getElementsByTagName('color')[0];
                if (colorNode) {
                    const rgbaAttr = colorNode.getAttribute('rgba') || '0.5 0.5 0.5 1';
                    matColor = rgbaAttr.split(/\s+/).filter(Boolean).map(Number);
                }
            }
            
            visuals.push({
                origin: { 
                    xyz: xyz.length === 3 ? xyz : [0, 0, 0], 
                    rpy: rpy.length === 3 ? rpy : [0, 0, 0] 
                },
                geometry: { type: geometryType, params: geometryParams },
                material: { name: matName, color: matColor }
            });
        }
        
        links[name] = { name, visuals };
    }
    
    // Parse joints
    const jointNodes = xmlDoc.getElementsByTagName('joint');
    for (let i = 0; i < jointNodes.length; i++) {
        const jointNode = jointNodes[i];
        const name = jointNode.getAttribute('name');
        const type = jointNode.getAttribute('type');
        
        const parentNode = jointNode.getElementsByTagName('parent')[0];
        const childNode = jointNode.getElementsByTagName('child')[0];
        const originNode = jointNode.getElementsByTagName('origin')[0];
        
        const parent = parentNode ? parentNode.getAttribute('link') : null;
        const child = childNode ? childNode.getAttribute('link') : null;
        
        const xyzStr = originNode ? originNode.getAttribute('xyz') : '0 0 0';
        const rpyStr = originNode ? originNode.getAttribute('rpy') : '0 0 0';
        
        const xyz = xyzStr.split(/\s+/).filter(Boolean).map(Number);
        const rpy = rpyStr.split(/\s+/).filter(Boolean).map(Number);
        
        joints.push({
            name,
            type,
            parent,
            child,
            origin: { 
                xyz: xyz.length === 3 ? xyz : [0, 0, 0], 
                rpy: rpy.length === 3 ? rpy : [0, 0, 0] 
            }
        });
    }
    
    return { links, joints };
}

export function buildThreeGroup(parsedUrdf) {
    const { links, joints } = parsedUrdf;
    const materialCache = {};
    
    function getMaterial(matInfo) {
        if (!matInfo || (!matInfo.name && !matInfo.color)) {
            return new THREE.MeshBasicMaterial({
                color: 0x94a3b8 // Slate grey
            });
        }
        
        const cacheKey = matInfo.name || (matInfo.color ? matInfo.color.join(',') : 'default');
        if (materialCache[cacheKey]) {
            return materialCache[cacheKey];
        }
        
        let colorHex = 0x94a3b8;
        let opacity = 1.0;
        let transparent = false;
        
        if (matInfo.color) {
            const [r, g, b, a] = matInfo.color;
            colorHex = (Math.round(r * 255) << 16) | (Math.round(g * 255) << 8) | Math.round(b * 255);
            opacity = a;
            transparent = a < 1.0;
        } else if (matInfo.name) {
            const name = matInfo.name.toLowerCase();
            if (name.includes('blue')) {
                colorHex = 0x1e3a8a; // Premium Dark Blue
            } else if (name.includes('black')) {
                colorHex = 0x111827; // Premium Charcoal Black
            } else if (name.includes('red')) {
                colorHex = 0xd97706; // Amber Red-Orange
            } else if (name.includes('orange')) {
                colorHex = 0xea580c; // Premium Safety Orange
            } else if (name.includes('white')) {
                colorHex = 0xf1f5f9; // Slate White
            } else if (name.includes('green')) {
                colorHex = 0x059669; // Emerald Green
            }
        }
        
        // Define high-fidelity materials for a premium look
        let mat;
        const nameLower = (matInfo.name || '').toLowerCase();
        if (nameLower.includes('blue')) {
            mat = new THREE.MeshBasicMaterial({
                color: colorHex,
                opacity: opacity,
                transparent: transparent
            });
        } else if (nameLower.includes('black')) {
            mat = new THREE.MeshBasicMaterial({
                color: colorHex,
                opacity: opacity,
                transparent: transparent
            });
        } else if (nameLower.includes('red') || nameLower.includes('orange')) {
            mat = new THREE.MeshBasicMaterial({
                color: colorHex,
                opacity: opacity,
                transparent: transparent
            });
        } else {
            mat = new THREE.MeshBasicMaterial({
                color: colorHex,
                opacity: opacity,
                transparent: transparent
            });
        }
        
        materialCache[cacheKey] = mat;
        return mat;
    }
    
    // Create ThreeJS Groups for all links
    const linkGroups = {};
    for (const linkName in links) {
        const link = links[linkName];
        const group = new THREE.Group();
        group.name = `link_${linkName}`;
        
        link.visuals.forEach((visual, idx) => {
            let geometry;
            const { type, params } = visual.geometry;
            

            if (type === 'box') {
                geometry = new THREE.BoxGeometry(params.size[0], params.size[1], params.size[2]);
            } else if (type === 'cylinder') {
                // Three.js cylinders are oriented along Y, URDF is along Z.
                // We'll rotate the mesh to align with URDF Z-axis.
                geometry = new THREE.CylinderGeometry(params.radius, params.radius, params.length, 32);
            } else if (type === 'sphere') {
                geometry = new THREE.SphereGeometry(params.radius, 32, 32);
            }
            
            if (geometry) {
                if (typeof geometry.computeFlatVertexNormals === 'function') {
                    geometry.computeFlatVertexNormals();
                }
                const material = getMaterial(visual.material);
                const mesh = new THREE.Mesh(geometry, material);
                
                if (type === 'cylinder') {
                    mesh.rotation.x = Math.PI / 2;
                }
                
                const visualContainer = new THREE.Group();
                visualContainer.name = `visual_${idx}`;
                
                const { xyz, rpy } = visual.origin;
                visualContainer.position.set(xyz[0], xyz[1], xyz[2]);
                
                const euler = new THREE.Euler(rpy[0], rpy[1], rpy[2], 'ZYX');
                visualContainer.rotation.copy(euler);
                
                visualContainer.add(mesh);
                group.add(visualContainer);
            }
        });
        
        linkGroups[linkName] = group;
    }
    
    // Map connections
    const children = new Set();
    const parentToJoints = {};
    
    joints.forEach(joint => {
        children.add(joint.child);
        if (!parentToJoints[joint.parent]) {
            parentToJoints[joint.parent] = [];
        }
        parentToJoints[joint.parent].push(joint);
    });
    
    // Find the root link
    let rootLinkName = null;
    for (const linkName in links) {
        if (!children.has(linkName)) {
            rootLinkName = linkName;
            break;
        }
    }
    if (!rootLinkName && Object.keys(links).length > 0) {
        rootLinkName = Object.keys(links)[0];
    }
    if (!rootLinkName) return new THREE.Group();
    
    // Recursively assemble joints
    function assembleTree(linkName) {
        const group = linkGroups[linkName];
        const jointsFromParent = parentToJoints[linkName] || [];
        
        jointsFromParent.forEach(joint => {
            const childGroup = assembleTree(joint.child);
            const jointContainer = new THREE.Group();
            jointContainer.name = `joint_${joint.name}`;
            
            const { xyz, rpy } = joint.origin;
            jointContainer.position.set(xyz[0], xyz[1], xyz[2]);
            
            const euler = new THREE.Euler(rpy[0], rpy[1], rpy[2], 'ZYX');
            jointContainer.rotation.copy(euler);
            
            jointContainer.add(childGroup);
            group.add(jointContainer);
        });
        
        return group;
    }
    
    const rootGroup = assembleTree(rootLinkName);
    
    // Wrap model and compute ground height offset
    const wrapper = new THREE.Group();
    wrapper.name = 'robot_model_wrapper';
    
    const offsetContainer = new THREE.Group();
    offsetContainer.name = 'robot_offset_container';
    
    // Dynamically calculate wheel distance from origin to set the contact point at z = 0
    let groundOffset = 0.0;
    joints.forEach(joint => {
        if (joint.child && joint.child.toLowerCase().includes('wheel')) {
            const wheelLink = links[joint.child];
            if (wheelLink && wheelLink.visuals) {
                wheelLink.visuals.forEach(visual => {
                    if (visual.geometry.type === 'cylinder') {
                        const zOffset = Math.abs(joint.origin.xyz[2]) + visual.geometry.params.radius;
                        groundOffset = Math.max(groundOffset, zOffset);
                    }
                });
            }
        }
    });
    
    if (groundOffset === 0.0) {
        groundOffset = 0.1862; // Standard ReloBot diffbot fallback offset
    }
    
    // Shift up so that bottom of the wheels sits exactly on the ground plane (z = 0)
    offsetContainer.position.z = groundOffset;
    offsetContainer.add(rootGroup);
    wrapper.add(offsetContainer);
    
    return wrapper;
}

export function buildRobotModelFromUrdf(urdfString) {
    try {
        const parsed = parseUrdf(urdfString);
        return buildThreeGroup(parsed);
    } catch (e) {
        console.error('[RobotModel] Failed to construct robot model from URDF:', e);
        return null;
    }
}
