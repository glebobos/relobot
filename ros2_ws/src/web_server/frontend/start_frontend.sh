#!/bin/sh
set -e

# Define source and build directories
SRC_DIR="/app"
BUILD_DIR="/app/dist"

echo "Starting Frontend Container..."
echo "DEV mode: $DEV"

if [ "$DEV" = "true" ]; then
    echo "Development mode enabled. Building frontend..."
    cd "$SRC_DIR"

    if [ ! -f "package.json" ]; then
        echo "Error: package.json not found in $SRC_DIR"
        exit 1
    fi

    echo "Running npm install..."
    npm install

    echo "Running npm run build..."
    npm run build
else
    echo "Production mode. Assuming artifacts are present in $BUILD_DIR..."
    if [ ! -d "$BUILD_DIR" ]; then
        echo "Warning: $BUILD_DIR does not exist. Nginx might fail to serve content."
    fi
fi

echo "Starting Nginx..."
nginx -g 'daemon off;'
