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
> If you need to install `Ubuntu` or `ROS`, please check our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Installation

1. Clone this repository.
   ```sh
   $ git clone https://github.com/TeamSOBITS/coqui_tts
   ```
2. Navigate into the repository.
   ```sh
   $ cd coqui_tts/
   ```
4. Build image.
   ```sh
   $ bash build.sh
   ```
5. Create container.
   ```sh
   $ bash run.sh
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- LAUNCH AND USAGE -->
## Launch and Usage

<!-- It would be useful to add demo examples and screenshots about the nominal state of the program -->
Use this space to show how to launch and use the project.

1. Get a free API Key at [https://example.com](https://example.com)
2. Clone the repo
   ```sh
   git clone https://github.com/TeamSOBITS/coqui_tts.git
   ```
3. Install NPM packages
   ```sh
   npm install
   ```
4. Enter your API in `config.js`
   ```js
   const API_KEY = 'ENTER YOUR API';
   ```


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MILESTONE -->
## Milestone

- [ ] Feature 1
- [ ] Feature 2
- [ ] Feature 3
    - [ ] Nested Feature

See the [open issues](issues-url) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CHANGE-LOG -->
## Change-Log

- 2.0: Explanatory Title
  - In-detail 1
  - In-detail 2
  - In-detail 3
- 1.1: Explanatory Title
  - In-detail 1
  - In-detail 2
  - In-detail 3
- 1.0: Explanatory Title
  - In-detail 1
  - In-detail 2
  - In-detail 3


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

* []()
* []()
* []()

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