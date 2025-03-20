#!/bin/bash

echo "Starting Federated Learning Server..."
python federated/server.py &


sleep 3

echo "Starting Client 1 (Region Factor: 0.8)..."
python federated/client.py --region_factor=0.8 &

sleep 1

echo "Starting Client 2 (Region Factor: 1.0)..."
python federated/client.py --region_factor=1.0 &

sleep 1

echo "Starting Client 3 (Region Factor: 1.2)..."
python federated/client.py --region_factor=1.2 &

echo "All Clients and Server Started."


wait
