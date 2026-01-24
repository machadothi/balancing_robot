#!/bin/bash
# =============================================================================
# Balancing Robot - Project Setup Script
# =============================================================================
# This script initializes the project by:
#   1. Checking for required tools
#   2. Cloning/updating git submodules (libopencm3, FreeRTOS-Kernel)
#   3. Building libopencm3
#   4. Building the project
#
# Usage:
#   ./scripts/setup.sh          # Full setup and build
#   ./scripts/setup.sh --clean  # Clean and rebuild
#   ./scripts/setup.sh --help   # Show help
#
# Author: Thiago Cunha
# =============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Project root directory (where this script lives)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Submodule directories
LIB_DIR="${PROJECT_ROOT}/lib"
LIBOPENCM3_DIR="${LIB_DIR}/libopencm3"
FREERTOS_DIR="${LIB_DIR}/FreeRTOS-Kernel"
BUILD_DIR="${PROJECT_ROOT}/build"

# =============================================================================
# Helper Functions
# =============================================================================

print_header() {
    echo -e "\n${BLUE}==============================================================================${NC}"
    echo -e "${BLUE} $1${NC}"
    echo -e "${BLUE}==============================================================================${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

check_command() {
    if command -v "$1" &> /dev/null; then
        print_success "$1 found: $(command -v $1)"
        return 0
    else
        print_error "$1 not found!"
        return 1
    fi
}

show_help() {
    echo "Balancing Robot - Project Setup Script"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --clean     Clean build directory and rebuild"
    echo "  --rebuild   Rebuild libopencm3 and project"
    echo "  --deps-only Initialize submodules and build dependencies only"
    echo "  --help      Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0              # Full setup and build"
    echo "  $0 --clean      # Clean everything and rebuild"
    echo "  $0 --deps-only  # Just setup dependencies"
}

# =============================================================================
# Tool Check
# =============================================================================

check_tools() {
    print_header "Checking Required Tools"
    
    local missing=0
    
    check_command "arm-none-eabi-gcc" || missing=1
    check_command "arm-none-eabi-objcopy" || missing=1
    check_command "cmake" || missing=1
    check_command "make" || missing=1
    check_command "git" || missing=1
    
    # Optional tools
    if check_command "st-flash"; then
        echo "  (flashing will be available)"
    else
        print_warning "st-flash not found - flashing will not be available"
    fi
    
    if [ $missing -eq 1 ]; then
        echo ""
        print_error "Missing required tools! Please install them first."
        echo ""
        echo "On Ubuntu/Debian:"
        echo "  sudo apt install gcc-arm-none-eabi cmake make git"
        echo ""
        echo "On Arch Linux:"
        echo "  sudo pacman -S arm-none-eabi-gcc arm-none-eabi-newlib cmake make git"
        echo ""
        exit 1
    fi
    
    print_success "All required tools found!"
}

# =============================================================================
# Submodule Setup
# =============================================================================

setup_submodules() {
    print_header "Setting Up Git Submodules"
    
    cd "$PROJECT_ROOT"
    
    # Create lib directory if it doesn't exist
    mkdir -p "$LIB_DIR"
    
    # Check if we're in a git repository
    if [ ! -d ".git" ]; then
        print_warning "Not a git repository. Cloning dependencies manually..."
        
        # Clone libopencm3 (shallow clone for speed)
        if [ ! -d "$LIBOPENCM3_DIR" ]; then
            echo "Cloning libopencm3..."
            git clone --depth 1 https://github.com/libopencm3/libopencm3.git "$LIBOPENCM3_DIR"
        else
            print_success "libopencm3 already exists"
        fi
        
        # Clone FreeRTOS-Kernel (shallow clone for speed, LTS version)
        if [ ! -d "$FREERTOS_DIR" ]; then
            echo "Cloning FreeRTOS-Kernel (LTS)..."
            git clone --depth 1 --branch V10.4.3-LTS-Patch-2 https://github.com/FreeRTOS/FreeRTOS-Kernel.git "$FREERTOS_DIR"
        else
            print_success "FreeRTOS-Kernel already exists"
        fi
    else
        # Initialize submodules if .gitmodules exists
        if [ -f ".gitmodules" ]; then
            echo "Initializing git submodules..."
            git submodule update --init --recursive --depth 1
        else
            # Add submodules if they don't exist
            if [ ! -d "$LIBOPENCM3_DIR/.git" ]; then
                echo "Adding libopencm3 as submodule..."
                git submodule add --depth 1 https://github.com/libopencm3/libopencm3.git lib/libopencm3 || true
            fi
            
            if [ ! -d "$FREERTOS_DIR/.git" ]; then
                echo "Adding FreeRTOS-Kernel as submodule..."
                git submodule add --depth 1 https://github.com/FreeRTOS/FreeRTOS-Kernel.git lib/FreeRTOS-Kernel || true
            fi
            
            git submodule update --init --recursive --depth 1
        fi
    fi
    
    print_success "Submodules ready!"
}

# =============================================================================
# Build libopencm3
# =============================================================================

build_libopencm3() {
    print_header "Building libopencm3"
    
    if [ ! -d "$LIBOPENCM3_DIR" ]; then
        print_error "libopencm3 directory not found!"
        exit 1
    fi
    
    cd "$LIBOPENCM3_DIR"
    
    # Check if already built
    if [ -f "lib/libopencm3_stm32f1.a" ] && [ "$1" != "--rebuild" ]; then
        print_success "libopencm3 already built (use --rebuild to force)"
        return 0
    fi
    
    echo "Building libopencm3 for STM32F1..."
    make TARGETS=stm32/f1 -j$(nproc)
    
    if [ -f "lib/libopencm3_stm32f1.a" ]; then
        print_success "libopencm3 built successfully!"
    else
        print_error "libopencm3 build failed!"
        exit 1
    fi
}

# =============================================================================
# Build Project
# =============================================================================

build_project() {
    print_header "Building Balancing Robot"
    
    cd "$PROJECT_ROOT"
    
    # Create build directory
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    
    # Configure with CMake
    echo "Configuring with CMake..."
    cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi.cmake ..
    
    # Build
    echo ""
    echo "Building..."
    make -j$(nproc)
    
    # Check if build succeeded
    if [ -f "balancing-robot.elf" ]; then
        echo ""
        print_success "Build successful!"
        echo ""
        echo "Output files:"
        echo "  - build/balancing-robot.elf"
        echo "  - build/balancing-robot.bin"
        echo "  - build/balancing-robot.hex"
        echo ""
        echo "To flash:"
        echo "  cd build && make flash"
        echo "  or: st-flash write build/balancing-robot.bin 0x8000000"
    else
        print_error "Build failed!"
        exit 1
    fi
}

# =============================================================================
# Clean
# =============================================================================

clean_build() {
    print_header "Cleaning Build"
    
    if [ -d "$BUILD_DIR" ]; then
        rm -rf "$BUILD_DIR"
        print_success "Build directory removed"
    fi
}

clean_all() {
    clean_build
    
    if [ -d "$LIBOPENCM3_DIR" ]; then
        cd "$LIBOPENCM3_DIR"
        make clean 2>/dev/null || true
        print_success "libopencm3 cleaned"
    fi
}

# =============================================================================
# Main
# =============================================================================

main() {
    print_header "Balancing Robot - Project Setup"
    echo "Project root: $PROJECT_ROOT"
    
    # Parse arguments
    case "${1:-}" in
        --help|-h)
            show_help
            exit 0
            ;;
        --clean)
            clean_all
            check_tools
            setup_submodules
            build_libopencm3 --rebuild
            build_project
            ;;
        --rebuild)
            clean_build
            check_tools
            build_libopencm3 --rebuild
            build_project
            ;;
        --deps-only)
            check_tools
            setup_submodules
            build_libopencm3
            print_success "Dependencies ready! Run './scripts/setup.sh' to build the project."
            ;;
        "")
            check_tools
            setup_submodules
            build_libopencm3
            build_project
            ;;
        *)
            print_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
    
    echo ""
    print_success "Done!"
}

main "$@"
