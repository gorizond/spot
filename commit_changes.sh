#!/bin/bash

cd /root/Yandex.Disk/Projects/gorizond/spot

echo "Adding changes to git..."
git add .

echo "Committing changes..."
git commit -m "Fix GPIO initialization in LCD C++ app to work in container"

echo "Pushing changes to remote repository..."
git push origin main

echo "Changes committed and pushed successfully!"
