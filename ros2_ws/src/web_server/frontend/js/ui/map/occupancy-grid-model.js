export function isValidOccupancyGrid(message) {
    const info = message?.info;
    return Boolean(
        info
        && Number.isInteger(info.width)
        && Number.isInteger(info.height)
        && info.width > 0
        && info.height > 0
        && info.resolution > 0
        && info.origin?.position
        && info.origin?.orientation
        && message.data?.length === info.width * info.height,
    );
}

export function occupancyDataEqual(left, right) {
    if (!left || !right || left.length !== right.length) return false;
    for (let index = 0; index < left.length; index++) {
        if (left[index] !== right[index]) return false;
    }
    return true;
}

export function occupancyOriginEqual(left, right) {
    if (!left || !right) return false;
    return left.position.x === right.position.x
        && left.position.y === right.position.y
        && left.position.z === right.position.z
        && left.orientation.x === right.orientation.x
        && left.orientation.y === right.orientation.y
        && left.orientation.z === right.orientation.z
        && left.orientation.w === right.orientation.w;
}

export function cloneOccupancyOrigin(origin) {
    return {
        position: { ...origin.position },
        orientation: { ...origin.orientation },
    };
}

export function classifyOccupancyCells(data, width, height) {
    const occupied = [];
    const unexploredBoundary = [];
    for (let row = 0; row < height; row++) {
        for (let column = 0; column < width; column++) {
            const value = data[row * width + column];
            if (value === 100) {
                occupied.push({ row, column });
            } else if (value === -1 && bordersFreeCell(data, width, height, row, column)) {
                unexploredBoundary.push({ row, column });
            }
        }
    }
    return { occupied, unexploredBoundary };
}

function bordersFreeCell(data, width, height, row, column) {
    return (row > 0 && data[(row - 1) * width + column] === 0)
        || (row < height - 1 && data[(row + 1) * width + column] === 0)
        || (column > 0 && data[row * width + column - 1] === 0)
        || (column < width - 1 && data[row * width + column + 1] === 0);
}
