<a name="readme-top"></a>

[JP](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
<!-- [![MIT License][license-shield]][license-url] -->

# Coqui TTS for ROS

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#環境構築">環境構築</a>
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

本リポジトリは[coqui-ai/TTS](https://github.com/coqui-ai/TTS)とROSの接続を可能にし，リアルタイムの高度な音声合成を提供する．
最新の `TTSv2` は16ヶ国語に対応し，全体的にパフォーマンスが向上している．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- セットアップ -->
## セットアップ

ここで，本レポジトリのセットアップ方法について説明する．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System  | Version |
| --- | --- |
| Ubuntu | 20.04 (Focal Fossa) - Local Env. |
| Python | >= 3.9, < 3.12 |
| Docker Engine | 26.0.0 (動作確認済) |
| CUDA | >=11.8 (GPU使用の場合) |

> [!NOTE]
> 本レポジトリを使用するには， [Docker](https://docs.docker.com/engine/install/ubuntu/) が必要である．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### インストール方法

1. ROSの`src`フォルダに移動します．
   ```sh
   $ roscd
   # もしくは，"cd ~/catkin_ws/"へ移動．
   $ cd src/
   ```
2. 本レポジトリをcloneします．
   ```sh
   $ git clone https://github.com/TeamSOBITS/coqui_tts_ros
   ```
3. レポジトリの中へ移動します．
   ```sh
   $ cd coqui_tts_ros/
   ```
4. 依存パッケージをインストールします．
   ```sh
   $ bash install.sh
   ```
5. パッケージをコンパイルします．
   ```sh
   $ roscd
   # もしくは，"cd ~/catkin_ws/"へ移動．
   $ catkin_make
   ```
6. TTSサーバーを簡単に実行するために， `alias` を作成する.
    - **CPUのみ**の場合:
    ```sh
    $ echo "alias tts_launch='docker run --rm -it -p 5002:5002 -v ~/{PATH_ROS_WS_LOCAL}/src/coqui_tts_ros/models/:/root/.local/share/tts/ --entrypoint \"tts-server\" ghcr.io/coqui-ai/tts-cpu'" >> ~/.bash_alias
    ```
    - **GPU**の場合:
    ```sh
    $ echo "alias tts_launch='docker run --rm -it -p 5002:5002 --gpus all -v ~/{PATH_ROS_WS_LOCAL}/src/coqui_tts_ros/models/:/root/.local/share/tts/ --entrypoint \"tts-server\" ghcr.io/coqui-ai/tts'" >> ~/.bash_alias
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
    $ tts_launch --model_name tts_models/en/vctk/vits
    ```
    - **GPU**の場合:
    ```sh
    $ tts_launch --model_name tts_models/en/vctk/vits --use_cuda true
    ```
> [!NOTE]
> `--model_name` を更新することが可能です.
そのために[model_list.yaml](models/model_list.yaml)を参照してください．

2. TTSの起動する機能をパラメタとし [tts.launch](launch/tts.lach.launch)に設定する．
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
    <!-- Set sound_audio to true if you want to play the sound -->
    <arg name="sound_audio" default="true"/>
    ```

3. [tts.launch](launch/tts.launch)というlaunchファイルを実行する．
    ```sh
    $ roslaunch coqui_tts_ros tts.launch
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- マイルストーン -->
## マイルストーン

- [ ] `--model_name` をパラメータとして設定できるようにする．
- [ ] `style_wav` の機能を導入する．

現時点のバッグや新規機能の依頼を確認するために[Issueページ](issues-url) をご覧ください．

<p align="right">(<a href="#readme-top">上に</a>)</p>


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

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->


<!-- LICENSE -->
<!-- ## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->


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
<!-- [license-shield]: https://img.shields.io/github/license/TeamSOBITS/coqui_tts_ros.svg?style=for-the-badge -->
[license-url]: LICENSE.txt