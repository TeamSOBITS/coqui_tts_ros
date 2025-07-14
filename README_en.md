<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]


# Coqui TTS for ROS

<details>
<summary>Table of Contents</summary>
<ol>
<li>
<a href="#introduction">Introduction</a>
</li>
<li>
<a href="#getting-started">Getting Started</a>
<ul>
<li><a href="#prerequisites">Prerequisites</a></li>
<li><a href="#installation">Installation</a></li>
</ul>
</li>
<li><a href="#launch-and-usage">Launch and Usage</a></li>
<li><a href="#milestones">Milestones</a></li>
<li><a href="#references">References</a></li>
</ol>
</details>

## Introduction

This repository enables the connection of [coqui-ai/TTS](https://github.com/coqui-ai/TTS) with ROS2 to provide real-time, advanced speech synthesis.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Getting Started

This section provides instructions on how to set up this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Prerequisites

First, ensure you have the following environment set up before proceeding to the installation step.

| System  | Version |
| --- | --- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS | Humble Hawksbill |
| Python | 3.10 |
| Docker Engine | 26.0.0 (Tested) |
| CUDA | >=11.8 (for GPU usage) |

> [!NOTE]
> This repository requires [Docker](https://docs.docker.com/engine/install/ubuntu/) to be installed.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Installation

1.  Navigate to your ROS2 `src` folder.
    ```sh
    cd ~/colcon_ws/src/
    ```
2.  Clone this repository.
    ```sh
    git clone -b feature/humble-devel https://github.com/TeamSOBITS/coqui_tts_ros
    ```
3.  Navigate into the repository.
    ```sh
    cd coqui_tts_ros/
    ```
4.  Install the dependent packages.
    ```sh
    bash install.sh
    ```
5.  Compile the package.
    ```sh
    cd ~/colcon_ws/
    ```
    ```sh
    colcon build --symlink-install
    ```
    ```sh
    source ~/colcon_ws/install/setup.sh
    ```
6.  Create an `alias` for easily launching the TTS server.
      - **For CPU only**:
    <!-- end list -->
    ```sh
    echo "alias tts_launch='docker run --rm -it -p 5002:5002 -v ~/{PATH_ROS_WS_LOCAL}/src/coqui_tts_ros/models/:/root/.local/share/tts/ --entrypoint \"tts-server\" ghcr.io/coqui-ai/tts-cpu'" >> ~/.bash_alias
    ```
      - **For GPU**:
    <!-- end list -->
    ```sh
    echo "alias tts_launch='docker run --rm -it -p 5002:5002 --gpus all -v ~/{PATH_ROS_WS_LOCAL}/src/coqui_tts_ros/models/:/root/.local/share/tts/ --entrypoint \"tts-server\" ghcr.io/coqui-ai/tts'" >> ~/.bash_alias
    ```

> [\!IMPORTANT]
> `{PATH_ROS_WS_LOCAL}` is the path to your ROS workspace on your **local machine**.

> [!IMPORTANT]
> If you are already inside a Docker container, you must run step 6 on your local machine.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Launch and Usage


1.  Launch the TTS server on your **local machine**.

      - **For CPU only**:

    <!-- end list -->

    ```sh
    tts_launch --model_name tts_models/en/vctk/vits
    ```

      - **For GPU**:

    <!-- end list -->

    ```sh
    tts_launch --model_name tts_models/en/vctk/vits --use_cuda true
    ```

2.  Configure the parameters for the TTS launch file, [tts.launch.py](launch/tts.launch.py).

    ```python
    DeclareLaunchArgument(
            'url',
            default_value='http://localhost:5002',
            description='Set Coqui TTS server url'
        ),
        DeclareLaunchArgument(
            'addStopChar',
            default_value='true',
            description='Add period at the end of a sentence'
        ),
        # DeclareLaunchArgument(
        #     'filename',
        #     default_value='output.wav',
        #     description='Set result sound filename'
        # ),
        DeclareLaunchArgument(
            'style_wav',
            default_value='',
            description='Set input style_wav if sample voice is given'
        ),
        DeclareLaunchArgument(
            'speaker_id',
            default_value='p225',
            description='Set Speaker ID if multi-speaker model is being used'
        ),
        DeclareLaunchArgument(
            'language_id',
            default_value='',
            description='Set Language if multi-language model is being used'
        ),
        DeclareLaunchArgument(
            'sound_audio',
            default_value='true',
            description='Set sound_audio to true if you want to play the sound'
        ),
    ```

3.  Run the [tts.launch.py](launch/tts.launch.py) launch file.

    ```sh
    ros2 launch coqui_tts_ros tts.launch.py
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Milestones

  - [ ] Enable setting `--model_name` as a parameter.
  - [ ] Implement the `style_wav` functionality.

For current bugs or new feature requests, please check the [Issue page](https://github.com/TeamSOBITS/coqui_tts_ros/issues).

<p align="right">(<a href="#readme-top">to top</a>)</p>

## References

  * [coqui-ai/TTS](https://github.com/coqui-ai/TTS)
  * [coqui-ai/TTS Docker images](https://docs.coqui.ai/en/latest/docker_images.html)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/coqui_tts_ros.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/coqui_tts_ros/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/coqui_tts_ros.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/coqui_tts_ros/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/coqui_tts_ros.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/coqui_tts_ros/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/coqui_tts_ros.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/coqui_tts_ros/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/coqui_tts_ros.svg?style=for-the-badge
[license-url]: LICENSE