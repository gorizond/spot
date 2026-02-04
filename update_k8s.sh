#!/bin/bash

echo "Connecting to Gorizond cluster..."

# Connect to the cluster via mcporter
mcporter rancher_clusters_kubeconfig --serverId gorizond --clusterId c-m-6mjj8vhv

if [ $? -ne 0 ]; then
    echo "Error connecting to cluster"
    exit 1
fi

echo "Connected to cluster, restarting daemonset..."

# Restart the daemonset to apply new image
kubectl rollout restart daemonset/spot-lcd-cpp -n spot-system

if [ $? -eq 0 ]; then
    echo "DaemonSet restarted successfully"
    echo "Checking pod status..."
    kubectl get pods -n spot-system -l app.kubernetes.io/name=spot-lcd-cpp
else
    echo "Error restarting daemonset"
    exit 1
fi
