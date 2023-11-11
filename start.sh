#!/bin/bash
export VK_ICD_FILENAMES="/usr/share/vulkan/icd.d/nvidia_icd.json"  && ./simulator/CarlaUE4.sh -quality-level=Low -ResX=800 -ResY=600
