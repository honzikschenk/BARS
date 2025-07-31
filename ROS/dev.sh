#!/bin/bash
# Development helper script for BARS ROS2 workspace

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE} $1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

# Function to check if Docker is available
check_docker() {
    if ! command -v docker &> /dev/null; then
        print_error "Docker is not installed or not in PATH"
        exit 1
    fi
    
    if ! command -v docker-compose &> /dev/null; then
        print_error "Docker Compose is not installed or not in PATH"
        exit 1
    fi
}

# Function to detect architecture
detect_arch() {
    local arch=$(uname -m)
    case $arch in
        x86_64) echo "amd64" ;;
        aarch64|arm64) echo "arm64" ;;
        armv7l) echo "arm/v7" ;;
        *) echo "unknown" ;;
    esac
}

# Function to build containers
build_containers() {
    print_header "Building BARS Containers"
    
    local arch=$(detect_arch)
    print_status "Detected architecture: $arch"
    
    if [ "$arch" = "unknown" ]; then
        print_warning "Unknown architecture, attempting to build anyway"
    fi
    
    print_status "Building development container..."
    docker-compose build bars_dev
    
    print_status "Building production container..."
    docker-compose build bars_prod
}

# Function to start development environment
start_dev() {
    print_header "Starting Development Environment"
    
    print_status "Starting development container..."
    docker-compose up -d bars_dev
    
    print_status "Entering development container..."
    docker-compose exec bars_dev bash
}

# Function to build ROS workspace
build_workspace() {
    print_header "Building ROS2 Workspace"
    
    print_status "Building workspace in container..."
    docker-compose exec bars_dev bash -c "
        cd /workspace && \
        rosdep install --from-paths src --ignore-src -r -y && \
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    "
}

# Function to run tests
run_tests() {
    print_header "Running Tests"
    
    print_status "Running ROS2 tests..."
    docker-compose exec bars_dev bash -c "
        cd /workspace && \
        colcon test && \
        colcon test-result --verbose
    "
}

# Function to clean workspace
clean_workspace() {
    print_header "Cleaning Workspace"
    
    print_status "Cleaning build artifacts..."
    docker-compose exec bars_dev bash -c "
        cd /workspace && \
        rm -rf build install log
    "
}

# Function to stop all containers
stop_containers() {
    print_header "Stopping Containers"
    
    print_status "Stopping all BARS containers..."
    docker-compose down
}

# Function to show help
show_help() {
    cat << EOF
BARS Development Helper Script

Usage: $0 [COMMAND]

Commands:
    build       Build Docker containers
    dev         Start development environment
    workspace   Build ROS2 workspace
    test        Run tests
    clean       Clean workspace
    stop        Stop all containers
    help        Show this help message

Examples:
    $0 build      # Build containers for development
    $0 dev        # Start development environment
    $0 workspace  # Build the ROS workspace
    $0 test       # Run all tests
    $0 clean      # Clean build artifacts
    $0 stop       # Stop all running containers

For more information, see README.md
EOF
}

# Main script logic
main() {
    check_docker
    
    case "${1:-help}" in
        build)
            build_containers
            ;;
        dev)
            start_dev
            ;;
        workspace)
            build_workspace
            ;;
        test)
            run_tests
            ;;
        clean)
            clean_workspace
            ;;
        stop)
            stop_containers
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "Unknown command: $1"
            show_help
            exit 1
            ;;
    esac
}

# Run main function with all arguments
main "$@"