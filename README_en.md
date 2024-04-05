[<a name="readme-top"></a>

[JP](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
<!-- [![MIT License][license-shield]][license-url] -->

# Coqui TTS ROS

<!-- TABLE OF CONTENTS -->
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
    <li><a href="#milestone">Milestone</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>


<!-- INTRODUCTION -->
## Introduction

This repository is a fork of the [coqui-ai/TTS](https://github.com/coqui-ai/TTS) which provides a library for advanced Text-to-Speech generation.
The latest `TTSv2` provides of 16 languages and better performance overall.

> [!NOTE]
> This repository is intended to be used inside a Docker container.
Changes in the local enviroment may occur.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- GETTING STARTED -->
## Getting Started

This section describes how to set up this repository.

### Prerequisites

First, please set up the following environment before proceeding to the next installation stage.

| System  | Version |
| --- | --- |
| Ubuntu | 20.04 (Focal Fossa) - Local Env. |
| Python | >= 3.9, < 3.12 |
| Docker Engine | Tested on 26.0.0 |
| CUDA | >=11.8 (If GPU is used) |

> [!NOTE]
> [Docker](https://docs.docker.com/engine/install/ubuntu/) is required to use this TTS library.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Installation

1. Clone this repository.
    ```sh
    $ git clone https://github.com/TeamSOBITS/coqui_tts_ros
    ```
2. Navigate into the repository.
    ```sh
    $ cd coqui_tts/
    ```
3. Install the dependent packages.
    ```sh
    $ bash install.sh
    ```
4. Compile the package.
    ```sh
    $ roscd
    # Or just use "cd ~/catkin_ws/" and change directory.
    $ catkin_make
    ```

5. Create a simple alias to launch the TTS server.
    - If using **CPU**:
    ```sh
    $ echo "alias tts_launch='docker run --rm -it -p 5002:5002 -v ~/{PATH_ROS_WS_LOCAL}/src/coqui_tts_ros/models/:/root/.local/share/tts/ --entrypoint \"tts-server\" ghcr.io/coqui-ai/tts-cpu'" >> ~/.bash_alias
    ```
    - If using **GPU**:
    ```sh
    $ echo "alias tts_launch='docker run --rm -it -p 5002:5002 --gpus all -v ~/{PATH_ROS_WS_LOCAL}/src/coqui_tts_ros/models/:/root/.local/share/tts/ --entrypoint \"tts-server\" ghcr.io/coqui-ai/tts'" >> ~/.bash_alias
    ```
> [!IMPORTANT]
> `{PATH_ROS_WS_LOCAL}` needs to be updated to your ROS PATH in the **local environment**.

> [!IMPORTANT]
> You need to run the command 5. in the **local environment**.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- LAUNCH AND USAGE -->
## Launch and Usage

1. Launch TTS server from the **local environment**.
    - If using **CPU**:
    ```sh
    $ tts_launch --model_name tts_models/en/vctk/vits
    ```
    - If using **GPU**:
    ```sh
    $ tts_launch --model_name tts_models/en/vctk/vits --use_cuda true
    ```
> [!NOTE]
> Remember that `--model_name` value can be updated.
Please, check the available models in [model_list.yaml](models/model_list.yaml).

> [!IMPORTANT]
> If you are using another container at the same time, remember to launch the server in your **local environment**.

2. Set the parameters inside [tts.launch](launch/tts.lach.launch) and select the functions to be used.
    ```xml
    <!-- Set Coqui TTS server url -->
    <arg name="url"         default="http://localhost:5002"/>
    <!-- Add period at the end of a sentence (true) -->
    <arg name="addStopChar" default="true"/>
    <!-- Set result sound filename -->
    <arg name="filename"    default="output.wav"/>
    <!-- Set input style_wav if sample voice is given -->
    <arg name="style_wav"   default=""/>
    <!-- Set Speaker ID if multi-speaker model is being used -->
    <arg name="speaker_id"  default="p225"/>
    <!-- Set Language if multi-language model is being used -->
    <arg name="language_id" default=""/>
    ```

3. Execute the launch file [tts.launch](launch/tts.launch).
    ```sh
    $ roslaunch coqui_tts_ros tts.launch
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MILESTONE -->
## Milestone

- [ ] Choose `--model_name` value through parameter.
- [ ] Make available the funtion of `style_wav`.

See the [open issues](issues-url) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTRIBUTING -->
<!-- ## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->



<!-- LICENSE -->
<!-- ## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [coqui-ai/TTS](https://github.com/coqui-ai/TTS)
* [coqui-ai/TTS Docker images](https://docs.coqui.ai/en/latest/docker_images.html)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/coqui_tts.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/coqui_tts/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/coqui_tts.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/coqui_tts/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/coqui_tts.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/coqui_tts/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/coqui_tts.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/coqui_tts/issues
<!-- [license-shield]: https://img.shields.io/github/license/TeamSOBITS/coqui_tts.svg?style=for-the-badge -->
[license-url]: LICENSE.txt