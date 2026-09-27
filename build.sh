#!/bin/bash
set -e

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Load common utilities
source "${SCRIPT_DIR}/scripts/common.sh"

# ========================
# Build Functions
# ========================

setup_inference_runtime() {
    print_header "[Setting up Inference Runtime]"

    DOWNLOAD_SCRIPT="${SCRIPT_DIR}/scripts/download_inference_runtime.sh"

    if [ -f "$DOWNLOAD_SCRIPT" ]; then
        print_info "Checking inference libraries..."
        bash "$DOWNLOAD_SCRIPT" onnx || {
            print_error "Failed to setup inference libraries"
            exit 1
        }
        print_success "Inference runtime setup completed!"
    else
        print_warning "Download script not found: $DOWNLOAD_SCRIPT"
    fi
}

setup_mujoco() {
    print_header "[Setting up MuJoCo]"

    DOWNLOAD_MUJOCO_SCRIPT="${SCRIPT_DIR}/scripts/download_mujoco.sh"

    if [ -f "$DOWNLOAD_MUJOCO_SCRIPT" ]; then
        print_info "Checking MuJoCo library..."
        bash "$DOWNLOAD_MUJOCO_SCRIPT" || {
            print_error "Failed to setup MuJoCo"
            exit 1
        }
        print_success "MuJoCo setup completed!"
    else
        print_warning "MuJoCo download script not found: $DOWNLOAD_MUJOCO_SCRIPT"
    fi
}

setup_robot_descriptions() {
    print_header "[Setting up Robot Descriptions]"

    DOWNLOAD_ROBOT_DESC_SCRIPT="${SCRIPT_DIR}/scripts/download_robot_descriptions.sh"

    if [ -f "$DOWNLOAD_ROBOT_DESC_SCRIPT" ]; then
        print_info "Checking robot description files..."
        bash "$DOWNLOAD_ROBOT_DESC_SCRIPT" || {
            print_error "Failed to setup robot descriptions"
            exit 1
        }
        print_success "Robot descriptions setup completed!"
    else
        print_warning "Robot descriptions download script not found: $DOWNLOAD_ROBOT_DESC_SCRIPT"
    fi
}

run_cmake_build() {
    print_header "[Running CMake Build]"
    print_separator

    cmake src/rl_sar/ -B cmake_build
    cmake --build cmake_build -j$(nproc 2>/dev/null || echo 4)

    print_success "CMake build completed!"
}

run_mujoco_build() {
    print_header "[Running MuJoCo Build]"
    print_info "Building with MuJoCo simulator support..."
    print_separator

    cmake src/rl_sar/ -B cmake_build -DUSE_MUJOCO=ON
    cmake --build cmake_build -j$(nproc 2>/dev/null || echo 4)

    print_success "MuJoCo build completed!"
}

# ========================
# Clean Functions
# ========================

clean_workspace() {
    print_header "[Cleaning Workspace]"

    print_info "The following will be cleaned:"
    echo "  - directory cmake_build/"

    if ! ask_confirmation "Are you sure you want to clean build artifacts?"; then
        print_warning "Clean operation cancelled."
        exit 0
    fi

    print_info "Cleaning build artifacts..."
    rm -rf cmake_build/

    print_success "Clean completed!"
}

# ========================
# Main Script
# ========================

show_usage() {
    print_header "[Build System Usage]"
    print_header
    echo -e "Usage: $0 [OPTIONS]"
    echo ""
    echo -e "${COLOR_INFO}Options:${COLOR_RESET}"
    echo -e "  -c, --clean      Clean build artifacts"
    echo -e "  -m, --cmake      Build using CMake (default)"
    echo -e "  -mj,--mujoco     Build with MuJoCo simulator support"
    echo -e "  -h, --help       Show this help message"
    echo ""
    echo -e "${COLOR_INFO}Examples:${COLOR_RESET}"
    echo -e "  $0                    # Build (same as -m)"
    echo -e "  $0 -c                 # Clean build artifacts"
    echo -e "  $0 -mj                # Build with MuJoCo simulator support"
}

main() {
    local clean_mode=false
    local cmake_mode=false
    local mujoco_mode=false

    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -c|--clean) clean_mode=true; shift ;;
            -m|--cmake) cmake_mode=true; shift ;;
            -mj|--mujoco) cmake_mode=true; mujoco_mode=true; shift ;;
            -h|--help) show_usage; exit 0 ;;
            *) print_error "Unknown option: $1"; show_usage; exit 1 ;;
        esac
    done

    # Handle clean mode
    if [ "$clean_mode" = true ]; then
        clean_workspace
        exit 0
    fi

    # Handle MuJoCo build mode
    if [ "$mujoco_mode" = true ]; then
        setup_inference_runtime
        setup_robot_descriptions
        setup_mujoco
        run_mujoco_build
        exit 0
    fi

    # CMake build (default, for hardware deployment)
    setup_inference_runtime
    run_cmake_build
}

main "$@"
