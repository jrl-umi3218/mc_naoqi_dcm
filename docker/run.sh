#!/bin/bash

# Retrieve two parameters from the command line: robotName and version
# Ensure that there is enough parameters, display help otherwise
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 robotName version"
  exit 1
fi
robotName="${1}"
version="${2}"

image_name=docker-naoqi-dcm-${robotName}-${version}:latest

docker run -it ${image_name}
