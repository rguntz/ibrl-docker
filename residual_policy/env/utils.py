# Copyright 2025 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
    
import collections
import os

from matplotlib.image import AxesImage
import matplotlib.pyplot as plt
import numpy as np
import time




def plot_observation_images(observation: dict, cam_list: list[str]) -> list[AxesImage]:
    """
    Plot observation images from multiple camera viewpoints.

    :param observation: The observation data containing images.
    :param cam_list: List of camera names used for capturing images.
    :return: A list of AxesImage objects for dynamic updates.
    """
    images = observation.get("images", {})

    # Define the layout based on the provided camera list
    num_cameras = len(cam_list)

    if num_cameras == 4:
        cols = 2
        rows = 2
    else:
        cols = min(3, num_cameras)  # Maximum of 3 columns
        rows = (num_cameras + cols - 1) // cols  # Compute rows dynamically
    _, axs = plt.subplots(rows, cols, figsize=(10, 10))
    axs = axs.flatten() if isinstance(axs, (list, np.ndarray)) else [axs]

    plt_imgs: list[AxesImage] = []
    titles = {
        "cam_high": "Camera High",
        "cam_low": "Camera Low",
        "cam_teleop": "Teleoperator POV",
        "cam_left_wrist": "Left Wrist Camera",
        "cam_right_wrist": "Right Wrist Camera",
    }

    for i, cam in enumerate(cam_list):
        if cam in images:
            plt_imgs.append(axs[i].imshow(images[cam]))
            axs[i].set_title(titles.get(cam, cam))

    #for ax in axs.flat:
        #ax.axis("off")

    plt.ion()
    return plt_imgs

def plot_observation_images_from_obs(observation: dict, cam_list: list[str]) -> list[AxesImage]:
    """
    Plot observation images from multiple camera viewpoints.

    :param observation: The observation data containing images.
    :param cam_list: List of camera keys (e.g., 'video', 'video_2', etc.).
    :return: A list of AxesImage objects for dynamic updates.
    """
    # Extract images directly from the observation dict using new keys
    images = {cam: observation.get(cam) for cam in cam_list}

    # Define the layout based on the number of cameras
    num_cameras = len(cam_list)
    if num_cameras == 4:
        cols = 2
        rows = 2
    else:
        cols = min(3, num_cameras)
        rows = (num_cameras + cols - 1) // cols

    _, axs = plt.subplots(rows, cols, figsize=(10, 10))
    axs = axs.flatten() if isinstance(axs, (list, np.ndarray)) else [axs]

    plt_imgs: list[AxesImage] = []
    # Updated titles for new keys
    titles = {
        "video": "Camera High",
        "video_2": "Camera Low",
        "wrist_video": "Left Wrist Camera",
        "wrist_video_2": "Right Wrist Camera",
    }

    for i, cam in enumerate(cam_list):
        if images.get(cam) is not None:
            img_to_plot = np.transpose(images[cam], (1, 2, 0))
            plt_imgs.append(axs[i].imshow(img_to_plot))
            axs[i].set_title(titles.get(cam, cam))
        else:
            axs[i].axis("off")  # hide empty axes

    plt.ion()
    return plt_imgs




def set_observation_images(
    observation: dict,
    plt_imgs: list[AxesImage],
    cam_list: list[str],
) -> list[AxesImage]:
    """
    Update displayed observation images dynamically.

    :param observation: The observation data containing updated images.
    :param plt_imgs: A list of AxesImage objects for dynamic updates.
    :param cam_list: List of camera names.
    :return: Updated list of AxesImage objects for real-time visualization.
    """
    images = observation.get("images", {})

    # Update image data dynamically
    for i, cam in enumerate(cam_list):
        if cam in images and i < len(plt_imgs):
            plt_imgs[i].set_data(images[cam])

    plt.pause(0.02)
    return plt_imgs


def set_observation_images_from_obs(
    observation: dict,
    plt_imgs: list[AxesImage],
    cam_list: list[str],
) -> list[AxesImage]:
    """
    Update displayed observation images dynamically.

    :param observation: The observation data containing updated images.
    :param plt_imgs: A list of AxesImage objects for dynamic updates.
    :param cam_list: List of camera names.
    :return: Updated list of AxesImage objects for real-time visualization.
    """
    images = {cam: observation.get(cam) for cam in cam_list}

    # Update image data dynamically
    for i, cam in enumerate(cam_list):
        if cam in images and i < len(plt_imgs):
            # Convert CHW -> HWC for matplotlib
            img_to_plot = np.transpose(images[cam], (1, 2, 0))
            plt_imgs[i].set_data(img_to_plot)

    plt.pause(0.02)
    return plt_imgs