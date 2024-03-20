#!/bin/bash

# Retrieve two parameters from the command line: robotName and version
# Ensure that there is enough parameters, display help otherwise
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 robotName version"
  exit 1
fi
robotName="${1}"
version="${2}"

image_name=docker-naoqi-dcm-${robotName}-${version}
docker_file=docker/Dockerfile_${version}
sed -i -e "s/@ROBOT_NAME@/${robotName}/g" ${docker_file}
echo "Generating docker image ${image_name} from docker file ${docker_file}"
cat ${docker_file}
echo ""
docker build -t ${image_name} -f ${docker_file} .

# check for failed build
if [ $? -ne 0 ]; then
  echo "Docker build failed"
  exit 1
fi

echo "Copying generated library to /tmp"
id=$(docker create ${image_name})
docker cp $id:/libmc_naoqi_dcm.so /tmp/libmc_naoqi_dcm_${robotName}_${version}.so

# check if file exist
if [ -f /tmp/libmc_naoqi_dcm_${robotName}_${version}.so ]; then
  echo "Successfully generated /tmp/libmc_naoqi_dcm_${robotName}_${version}.so"
  echo "You may now copy this file on your ${robotName} robot"
else
  echo "Failed to generate mc_naoqi_dcm library"
fi
