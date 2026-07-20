#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

usage() {
  cat <<'EOF'
Usage: ./run_firmware.sh <command> [target] [options]

Commands:
  build [target|all]  Build firmware UF2 binary in Docker.
  flash <target>      Discover port, reset board into BOOTSEL mode, and flash via picotool in Docker.
  scan                Scan and list all connected serial devices (/dev/ttyACM*) on host.
  info                Run picotool info -a inside Docker to inspect connected Pico boards.
  monitor <target|port> Open USB serial monitor via picocom in Docker.
  all                 Build all firmware targets.

Targets:
  wheels              pico_wheels_microros.uf2 (XIAO RP2040)
  wheels_calibration  pico_wheels_calibration.uf2 (XIAO RP2040)
  knives              pico_knives_microros.uf2 (XIAO RP2040)
  imu                 pico_imu_microros.uf2 (RPi Pico RP2040)
  ina226              pico_ina226_microros.uf2 (RPi Pico 2 RP2350)

Options:
  --port <device>     Explicitly specify serial port (e.g. /dev/ttyACM0)
  --baud <baudrate>   Baud rate for serial monitor (default: 115200)
  -y, --yes           Non-interactive mode (auto-confirm selected port)
  -h, --help          Show this help message

Examples:
  ./run_firmware.sh build wheels
  ./run_firmware.sh flash wheels
  ./run_firmware.sh flash wheels_calibration
  ./run_firmware.sh scan
  ./run_firmware.sh monitor /dev/ttyACM0
EOF
}

# Target metadata helpers
get_pkg_dir() {
  case "$1" in
    wheels|wheels_calibration) echo "pico_ware_wheels_microros" ;;
    knives)                     echo "pico_ware_knives_microros" ;;
    imu)                        echo "pico_ware_imu_microros" ;;
    ina226)                     echo "pico_ware_ina226_microros" ;;
    *)                          echo "" ;;
  esac
}

get_uf2_name() {
  case "$1" in
    wheels)             echo "pico_wheels_microros.uf2" ;;
    wheels_calibration) echo "pico_wheels_calibration.uf2" ;;
    knives)             echo "pico_knives_microros.uf2" ;;
    imu)                echo "pico_imu_microros.uf2" ;;
    ina226)             echo "pico_ina226_microros.uf2" ;;
    *)                  echo "" ;;
  esac
}

get_image_name() {
  local pkg_dir
  pkg_dir=$(get_pkg_dir "$1")
  if [[ -n "$pkg_dir" ]]; then
    echo "relobot_${pkg_dir}:latest"
  else
    echo "relobot_pico_ware_wheels_microros:latest"
  fi
}

get_expected_product() {
  case "$1" in
    wheels)             echo "pico_wheels_microros" ;;
    wheels_calibration) echo "pico_wheels_calibration" ;;
    knives)             echo "pico_knives_microros" ;;
    imu)                echo "pico_imu_microros" ;;
    ina226)             echo "pico_ina226_microros" ;;
    *)                  echo "" ;;
  esac
}

# Helper: Ensure Docker image exists for target (or build it once)
ensure_docker_image() {
  local target="${1:-wheels}"
  local pkg_dir
  pkg_dir=$(get_pkg_dir "$target")
  if [[ -z "$pkg_dir" ]]; then pkg_dir="pico_ware_wheels_microros"; fi

  local image_name
  image_name=$(get_image_name "$target")

  if ! docker image inspect "$image_name" >/dev/null 2>&1; then
    echo "==> Building Docker image ($image_name)..." >&2
    docker build -t "$image_name" "$pkg_dir"
  fi
  echo "$image_name"
}

# Helper: Extract USB sysfs properties for a given ttyACM sysfs path
read_tty_info() {
  local dev_sys="$1"
  local port="/dev/$(basename "$dev_sys")"
  local prod
  prod=$(cat "$dev_sys/device/../product" 2>/dev/null || echo "Unknown Product")
  local mfr
  mfr=$(cat "$dev_sys/device/../manufacturer" 2>/dev/null || echo "Unknown Mfr")
  local vid
  vid=$(cat "$dev_sys/device/../idVendor" 2>/dev/null || echo "????")
  local pid
  pid=$(cat "$dev_sys/device/../idProduct" 2>/dev/null || echo "????")
  local bus
  bus=$(readlink -f "$dev_sys/device/.." 2>/dev/null | xargs basename 2>/dev/null || echo "Unknown")

  echo "$port|$prod|$vid:$pid|$bus|$mfr"
}

# Helper: Find active tty port by USB product name
find_port_by_product() {
  local expected_prod="$1"
  for dev_sys in /sys/class/tty/ttyACM*; do
    if [[ -e "$dev_sys" ]]; then
      local info
      info=$(read_tty_info "$dev_sys")
      IFS='|' read -r p prod _ _ _ <<< "$info"
      if [[ "$prod" == "$expected_prod" ]]; then
        echo "$p"
        return 0
      fi
    fi
  done
  echo ""
}

build_target() {
  local target="$1"
  if [[ "$target" == "all" ]]; then
    for t in wheels knives imu ina226; do
      build_target "$t"
    done
    return 0
  fi

  local pkg_dir
  pkg_dir=$(get_pkg_dir "$target")
  if [[ -z "$pkg_dir" ]]; then
    echo "Error: Unknown build target '$target'." >&2
    exit 1
  fi

  local image_name
  image_name=$(ensure_docker_image "$target")

  echo "==> Compiling firmware for '$target'..."
  docker run --rm \
    --user "$(id -u):$(id -g)" \
    -v "$SCRIPT_DIR/$pkg_dir/src:/project/src" \
    -w /project \
    "$image_name" \
    /usr/local/bin/build.sh

  local uf2_name
  uf2_name=$(get_uf2_name "$target")
  if [[ -f "$pkg_dir/src/$uf2_name" ]]; then
    echo "✅ Success: Created $pkg_dir/src/$uf2_name"
  else
    echo "❌ Error: $uf2_name was not generated!" >&2
    exit 1
  fi
}

scan_ports() {
  echo "================================================================"
  echo " Scanning Connected Serial Devices (/dev/ttyACM*)"
  echo "================================================================"
  local found=0
  for dev_sys in /sys/class/tty/ttyACM*; do
    if [[ -e "$dev_sys" ]]; then
      found=1
      local info
      info=$(read_tty_info "$dev_sys")
      IFS='|' read -r port prod vidpid bus _ <<< "$info"
      printf "Port: %-15s | Product: %-25s | VID:PID: %s | Bus: %s\n" \
        "$port" "$prod" "$vidpid" "$bus"
    fi
  done

  if [[ $found -eq 0 ]]; then
    echo "No /dev/ttyACM* serial devices detected."
  fi
  echo "================================================================"
}

flash_target() {
  local target="$1"
  local pkg_dir
  pkg_dir=$(get_pkg_dir "$target")
  if [[ -z "$pkg_dir" ]]; then
    echo "Error: Unknown flash target '$target'." >&2
    exit 1
  fi

  local uf2_name
  uf2_name=$(get_uf2_name "$target")
  local uf2_path="$pkg_dir/src/$uf2_name"

  if [[ ! -f "$uf2_path" ]]; then
    echo "Firmware binary '$uf2_path' not found. Building first..."
    build_target "$target"
  fi

  local image_name
  image_name=$(ensure_docker_image "$target")

  local selected_port=""
  local expected_prod
  expected_prod=$(get_expected_product "$target")

  if [[ -n "$PORT" ]]; then
    selected_port="$PORT"
    echo "Using user-specified serial port: $selected_port"
  else
    echo "==> Scanning connected serial ports to locate '$target'..."
    local ports=()
    local matched_index=-1
    local idx=1

    for dev_sys in /sys/class/tty/ttyACM*; do
      if [[ -e "$dev_sys" ]]; then
        local info
        info=$(read_tty_info "$dev_sys")
        IFS='|' read -r p prod vidpid _ _ <<< "$info"

        ports+=("$p")

        local match_flag=""
        if [[ "$prod" == "$expected_prod" ]]; then
          match_flag=" (MATCHED PRODUCT)"
          matched_index=$((idx - 1))
        fi

        printf "  [%d] %-15s -> Product: %-25s VID:PID: %s%s\n" \
          "$idx" "$p" "$prod" "$vidpid" "$match_flag"
        idx=$((idx + 1))
      fi
    done

    echo "  [M] Manual BOOTSEL mode (Board already in BOOTSEL / skip 1200 baud reset)"

    if [[ ${#ports[@]} -eq 0 ]]; then
      echo "No active /dev/ttyACM* ports detected."
      echo "Assuming target board is already in BOOTSEL mode..."
      selected_port="MANUAL"
    elif [[ "$YES_FLAG" == "1" ]]; then
      if [[ $matched_index -ge 0 ]]; then
        selected_port="${ports[$matched_index]}"
      else
        selected_port="${ports[0]}"
      fi
      echo "Auto-selected $selected_port (non-interactive mode)"
    else
      local default_choice="M"
      if [[ $matched_index -ge 0 ]]; then
        default_choice=$((matched_index + 1))
      elif [[ ${#ports[@]} -gt 0 ]]; then
        default_choice=1
      fi

      read -r -p "Select port to reset into BOOTSEL mode [1-${#ports[@]} or M] (Default: $default_choice): " choice
      choice="${choice:-$default_choice}"

      if [[ "$choice" =~ ^[0-9]+$ ]] && [[ "$choice" -ge 1 ]] && [[ "$choice" -le ${#ports[@]} ]]; then
        selected_port="${ports[$((choice - 1))]}"
      else
        selected_port="MANUAL"
      fi
    fi
  fi

  if [[ "$selected_port" != "MANUAL" && -n "$selected_port" ]]; then
    echo "==> Sending 1200-baud reset signal to $selected_port..."
    stty -F "$selected_port" 1200 2>/dev/null || true
    echo "Waiting 3 seconds for board to re-enumerate in BOOTSEL mode..."
    sleep 3
  else
    echo "Proceeding directly to USB picotool flashing..."
  fi

  echo "==> Flashing '$uf2_name' via picotool in Docker..."
  docker run --rm --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    -v "$SCRIPT_DIR/$pkg_dir/src:/project" \
    -w /project \
    "$image_name" \
    picotool load -f -x "$uf2_name"

  echo "✅ Flashing complete! Microcontroller rebooted."
}

picotool_info() {
  local image_name
  image_name=$(ensure_docker_image "wheels")

  echo "==> Running picotool info -a in Docker..."
  docker run --rm --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    "$image_name" \
    picotool info -a
}

monitor_target() {
  local target="$1"
  local port=""

  if [[ "$target" =~ ^/dev/tty ]]; then
    port="$target"
  else
    local expected_prod
    expected_prod=$(get_expected_product "$target")
    if [[ -n "$expected_prod" ]]; then
      port=$(find_port_by_product "$expected_prod")
    fi
  fi

  if [[ -z "$port" ]]; then
    if [[ -n "$PORT" ]]; then
      port="$PORT"
    else
      echo "Error: Serial port not found for '$target'. Pass --port /dev/ttyACM<X>." >&2
      exit 1
    fi
  fi

  local image_name
  image_name=$(ensure_docker_image "$target")

  echo "==> Connecting serial monitor on $port ($BAUD baud) via picocom in Docker..."
  docker run --rm -it \
    --device="$port" \
    "$image_name" \
    picocom --baud "$BAUD" "$port"
}

# Main option parsing
PORT=""
BAUD="115200"
YES_FLAG=0

[[ $# -gt 0 ]] || { usage; exit 1; }
COMMAND="$1"; shift

TARGET=""
if [[ $# -gt 0 && ! "$1" =~ ^- ]]; then
  TARGET="$1"
  shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --baud) BAUD="$2"; shift 2 ;;
    -y|--yes) YES_FLAG=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

case "$COMMAND" in
  build)
    build_target "${TARGET:-all}"
    ;;
  flash)
    if [[ -z "$TARGET" ]]; then
      echo "Error: flash requires a target name (e.g. wheels, wheels_calibration, knives, imu, ina226)." >&2
      exit 1
    fi
    flash_target "$TARGET"
    ;;
  scan)
    scan_ports
    ;;
  info)
    picotool_info
    ;;
  monitor)
    if [[ -z "$TARGET" && -z "$PORT" ]]; then
      echo "Error: monitor requires a target name or --port /dev/ttyACM<X>." >&2
      exit 1
    fi
    monitor_target "${TARGET:-$PORT}"
    ;;
  all)
    build_target "all"
    ;;
  *)
    usage
    exit 1
    ;;
esac
