import assert from 'node:assert/strict';
import test from 'node:test';
import {
    classifyOccupancyCells,
    isValidOccupancyGrid,
    occupancyDataEqual,
} from '../js/ui/map/occupancy-grid-model.js';

function message(data, width = 3, height = 3) {
    return {
        info: {
            width,
            height,
            resolution: 0.05,
            origin: {
                position: { x: 0, y: 0, z: 0 },
                orientation: { x: 0, y: 0, z: 0, w: 1 },
            },
        },
        data,
    };
}

test('validates occupancy-grid dimensions and metadata', () => {
    assert.equal(isValidOccupancyGrid(message(new Int8Array(9))), true);
    assert.equal(isValidOccupancyGrid(message(new Int8Array(8))), false);
    assert.equal(isValidOccupancyGrid({}), false);
});

test('classifies occupied cells and only unknown cells bordering free space', () => {
    const data = new Int8Array([
        -1, -1, -1,
        -1, 0, 100,
        -1, -1, -1,
    ]);
    const result = classifyOccupancyCells(data, 3, 3);
    assert.deepEqual(result.occupied, [{ row: 1, column: 2 }]);
    assert.deepEqual(result.unexploredBoundary, [
        { row: 0, column: 1 },
        { row: 1, column: 0 },
        { row: 2, column: 1 },
    ]);
});

test('compares typed map payloads without coercion', () => {
    assert.equal(occupancyDataEqual(new Int8Array([0, -1]), [0, -1]), true);
    assert.equal(occupancyDataEqual(new Int8Array([0, -1]), [0, 0]), false);
});
