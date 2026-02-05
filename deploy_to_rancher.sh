#!/bin/bash
# deploy_to_rancher.sh - Deploy LCD application to Rancher cluster

set -e  # Exit on any error

echo "=== Deploying LCD Application to Rancher ==="

# Function to check prerequisites
check_prerequisites() {
    echo "Checking prerequisites..."
    
    if ! command -v kubectl &> /dev/null; then
        echo "Error: kubectl is not installed or not in PATH"
        exit 1
    fi
    
    if ! command -v docker &> /dev/null; then
        echo "Warning: docker is not installed or not in PATH (building will be skipped)"
    fi
    
    echo "Prerequisites check passed"
}

# Function to build Docker image
build_image() {
    if command -v docker &> /dev/null; then
        echo "Building Docker image..."
        ./build_and_push_docker.sh
    else
        echo "Skipping Docker build (docker not available)"
    fi
}

# Function to connect to cluster
connect_to_cluster() {
    echo "Connecting to Gorizond cluster..."
    
    # Attempt to connect to the cluster
    # This assumes you have the connection details available
    # You might need to adjust this part based on your actual connection method
    
    # Check if we're already connected to a cluster
    if kubectl cluster-info &> /dev/null; then
        echo "Already connected to cluster:"
        kubectl cluster-info | head -n 1
    else
        echo "Error: Cannot connect to Kubernetes cluster"
        echo "Please ensure you have proper kubeconfig setup"
        exit 1
    fi
}

# Function to deploy to cluster
deploy_to_cluster() {
    echo "Deploying LCD application to cluster..."
    
    # Apply the LCD deployment
    kubectl apply -f k8s/lcd_deployment.yaml
    
    if [ $? -ne 0 ]; then
        echo "Error: Failed to apply LCD deployment"
        exit 1
    fi
    
    echo "LCD deployment applied successfully"
    
    # Wait for deployment to be ready
    echo "Waiting for pods to be ready..."
    kubectl rollout status daemonset/spot-lcd-cpp -n spot-system --timeout=300s
    
    if [ $? -eq 0 ]; then
        echo "Deployment successful!"
    else
        echo "Warning: Deployment may not be completely ready, but timeout reached"
    fi
    
    # Show current pods
    echo "Current LCD pods:"
    kubectl get pods -n spot-system -l app.kubernetes.io/name=spot-lcd-cpp
}

# Function to verify deployment
verify_deployment() {
    echo "Verifying deployment..."
    
    # Check if daemonset exists
    if kubectl get daemonset spot-lcd-cpp -n spot-system &> /dev/null; then
        echo "✓ DaemonSet exists"
    else
        echo "✗ DaemonSet does not exist"
    fi
    
    # Check pods status
    PODS_RUNNING=$(kubectl get pods -n spot-system -l app.kubernetes.io/name=spot-lcd-cpp --field-selector=status.phase=Running --no-headers | wc -l)
    TOTAL_PODS=$(kubectl get pods -n spot-system -l app.kubernetes.io/name=spot-lcd-cpp --no-headers | wc -l)
    
    echo "✓ $PODS_RUNNING out of $TOTAL_PODS pods are running"
    
    # Show logs from one of the pods (if any are running)
    if [ $PODS_RUNNING -gt 0 ]; then
        POD_NAME=$(kubectl get pods -n spot-system -l app.kubernetes.io/name=spot-lcd-cpp --field-selector=status.phase=Running -o jsonpath='{.items[0].metadata.name}' 2>/dev/null)
        if [ ! -z "$POD_NAME" ]; then
            echo "Sample logs from $POD_NAME:"
            kubectl logs -n spot-system $POD_NAME | tail -n 10
        fi
    fi
}

# Main execution
main() {
    check_prerequisites
    
    if [ "$1" != "--skip-build" ]; then
        build_image
    else
        echo "Skipping build (--skip-build flag)"
    fi
    
    connect_to_cluster
    deploy_to_cluster
    verify_deployment
    
    echo ""
    echo "=== Deployment completed ==="
    echo "LCD application has been deployed to Rancher cluster"
    echo ""
    echo "To view pods: kubectl get pods -n spot-system -l app.kubernetes.io/name=spot-lcd-cpp"
    echo "To view logs: kubectl logs -n spot-system -l app.kubernetes.io/name=spot-lcd-cpp"
    echo "To delete deployment: kubectl delete -f k8s/lcd_deployment.yaml"
}

# Call main function with all arguments
main "$@"