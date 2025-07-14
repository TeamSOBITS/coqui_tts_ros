<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# Coqui TTS for ROS

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#セットアップ">セットアップ</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li><a href="#実行・操作方法">実行・操作方法</a></li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <!-- <li><a href="#変更履歴">変更履歴</a></li> -->
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>


<!-- 概要 -->
## 概要

本リポジトリは[coqui-ai/TTS](https://github.com/coqui-ai/TTS)とROS2の接続を可能にし，リアルタイムの高度な音声合成を提供する．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- セットアップ -->
## セットアップ

ここで，本レポジトリのセットアップ方法について説明する．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System  | Version |
| --- | --- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS | Humble Hawksbill |
| Python | 3.10 |
| Docker Engine | 26.0.0 (動作確認済) |
| CUDA | >=11.8 (GPU使用の場合) |

> [!NOTE]
> 本レポジトリを使用するには， [Docker](https://docs.docker.com/engine/install/ubuntu/) が必要である．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### インストール方法

1. ROS2の`src`フォルダに移動します．
   ```sh
   cd ~/colcon_ws/src/
   ```
2. 本レポジトリをcloneします．
   ```sh
   git clone -b feature/humble-devel https://github.com/TeamSOBITS/coqui_tts_ros
   ```
3. レポジトリの中へ移動します．
   ```sh
   cd coqui_tts_ros/
   ```
4. 依存パッケージをインストールします．
   ```sh
   bash install.sh
   ```
5. パッケージをコンパイルします．
   ```sh
   cd ~/colcon_ws/
   ```
   ```sh
   colcon build --symlink-install
   ```
   ```sh
   source ~/colcon_ws/install/setup.sh
   ```
6. TTSサーバーを簡単に実行するために， `alias` を作成する.
    - **CPUのみ**の場合:
    ```sh
    echo "alias tts_launch='docker run --rm -it -p 5002:5002 -v ~/{PATH_ROS_WS_LOCAL}/src/coqui_tts_ros/models/:/root/.local/share/tts/ --entrypoint \"tts-server\" ghcr.io/coqui-ai/tts-cpu'" >> ~/.bash_alias
    ```
    - **GPU**の場合:
    ```sh
    echo "alias tts_launch='docker run --rm -it -p 5002:5002 --gpus all -v ~/{PATH_ROS_WS_LOCAL}/src/coqui_tts_ros/models/:/root/.local/share/tts/ --entrypoint \"tts-server\" ghcr.io/coqui-ai/tts'" >> ~/.bash_alias
    ```
> [!IMPORTANT]
> `{PATH_ROS_WS_LOCAL}` は**ローカル環境**に存在するROSのワークスペースのPATHである．

> [!IMPORTANT]
> すでに，Dockerのコンテナーの中にいる場合，ローカル環境上でコマンド6を実行する必要がある．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 実行・操作方法 -->
## 実行・操作方法

1. **ローカル環境**上でTTSサーバーを立ち上げる．
    - **CPUのみ**の場合:
    ```sh
    tts_launch --model_name tts_models/en/vctk/vits
    ```
    - **GPU**の場合:
    ```sh
    tts_launch --model_name tts_models/en/vctk/vits --use_cuda true
    ```

2. TTSの起動する機能をパラメタとし [tts.launch.py](launch/tts.lach.launch.py)に設定する．
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

3. [tts.launch.py](launch/tts.launch.py)というlaunchファイルを実行する．
    ```sh
    ros2 launch coqui_tts_ros tts.launch.py
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- マイルストーン -->
## マイルストーン

- [ ] `--model_name` をパラメータとして設定できるようにする．
- [ ] `style_wav` の機能を導入する．

現時点のバッグや新規機能の依頼を確認するために[Issueページ](issues-url) をご覧ください．

<p align="right">(<a href="#readme-top">上に</a>)</p>


<!-- 参考文献 -->
## 参考文献

* [coqui-ai/TTS](https://github.com/coqui-ai/TTS)
* [coqui-ai/TTS Docker images](https://docs.coqui.ai/en/latest/docker_images.html)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


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
[license-url]: LICENSE.txt