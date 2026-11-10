# VS Code でビルド・書き込み・デバッグする(2026-11-10)

**2026-11-10: 開発は STM32CubeIDE のままにした。** この手順は、VS Code やコマンドでビルドしたくなったとき用に残す(CMake の設定は CI でも使っている)。
新しい PC が Ubuntu 22.04 なら、apt の `gcc-arm-none-eabi` は gcc 10 でリンカスクリプトの `READONLY` を読めないので使わない。ToraNeko.Mk3 を STM32Cube 拡張でビルドできている PC なら、ツールは拡張が入れたものがあるので、新しく入れずに拡張からビルドする。

もとの方針: PC を替えるのに合わせて、STM32CubeIDE をやめて VS Code に移る。ToraNeko.Mk3 と同じく、**CMake + Ninja + arm-none-eabi-gcc** でビルドし、
VS Code の **STM32Cube for Visual Studio Code** 拡張で書き込み・デバッグする。コマンドラインだけでもビルドでき、CI(`.github/workflows/firmware-build.yml`)でも同じビルドを確かめる。

## 新しい PC で最初にやること(GitHub から取ってくるところから)

Windows の場合。

1. **Git** を入れる(Git for Windows。GitHub Desktop でもよい)
2. **VS Code** を入れる
3. VS Code の拡張機能で **STM32Cube for Visual Studio Code** を入れる。足りないツールを入れる案内が出るので、それに従って **STM32CubeCLT** を入れる(arm-none-eabi-gcc・CMake・Ninja・STM32CubeProgrammer・ST-LINK の GDB サーバーがまとめて入る)。入れたら VS Code を開き直す
4. リポジトリを取ってくる: VS Code で `Ctrl+Shift+P` → **Git: Clone** → `https://github.com/Tokoro0917/KOGUMA.MkIIl` → 置くフォルダを選ぶ(GitHub へのサインインを聞かれたら、ブラウザでサインインする)。コマンドなら `git clone https://github.com/Tokoro0917/KOGUMA.MkIIl`
   - 置くフォルダの名前(パス)に日本語や空白を入れないほうがよい(ツールによっては読めない)
5. 「開きますか」と聞かれたら開く。CMake のプリセットを聞かれたら **Debug** を選ぶ
6. **ビルド**: 下のステータスバーの「Build」か `F7`。出力に `RAM: … / FLASH: …` が出て終われば成功(RAM 約113KB、Flash 約149KB)
7. **書き込み**: ST-LINK をつなぎ、**機体の電源を入れて**から `F5`。設定を聞かれたら STM32Cube の ST-LINK のデバッグを選ぶ。`main()` で止まるので、もう一度 `F5` で動く。デバッグをやめても、書き込んだプログラムは残る
8. **ログ**: ビューア(`tools/viewer.html`)を Chrome で開くのは今までどおり。保存先フォルダに、取ってきたリポジトリの `logs/` を選ぶ
9. **ログを GitHub に上げる**: VS Code の左の「ソース管理」で、増えたファイルにメッセージを書いてコミット → 「変更の同期」(push)

## コマンドだけでやる(Windows の PowerShell)

VS Code を使わなくても、同じビルドと書き込みができる。

```powershell
# 1. ツールを入れる(winget。終わったら PowerShell を開き直す)
winget install --id Git.Git -e
winget install --id Kitware.CMake -e
winget install --id Ninja-build.Ninja -e
winget search "Arm GNU"            # arm-none-eabi-gcc の Id を確かめてから入れる(例: Arm.GnuArmEmbeddedToolchain)
winget install --id <上で見つけた Id> -e
#    STM32CubeProgrammer は winget にないので、ST のサイトから入れる(書き込みに使う)

# 2. 入ったか確かめる(どれも版が出ればよい。arm-none-eabi-gcc は 11 以降)
git --version; cmake --version; ninja --version; arm-none-eabi-gcc --version

# 3. 取ってくる
cd C:\work                         # 置く場所(日本語・空白のないパス)
git clone https://github.com/Tokoro0917/KOGUMA.MkIIl
cd KOGUMA.MkIIl

# 4. ビルド(最後に RAM 約113KB、FLASH 約149KB と出れば成功)
cmake --preset Debug
cmake --build --preset Debug

# 5. 書き込み(ST-LINK をつなぎ、機体の電源を入れてから)
STM32_Programmer_CLI -c port=SWD -w build\Debug\KOGUMA.MkIIl.elf -v -rst

# 6. ログを上げる(ビューアで logs\ に保存したあと)
git add logs
git commit -m "ログ: 〇〇を走らせた"
git push

# 2回目からは、始める前に最新を取ってくる
git pull
```

- `arm-none-eabi-gcc` が見つからないと言われたら、インストーラーの「パスを通す(Add path to environment variable)」が外れている。入れ直すか、`bin` のフォルダを環境変数 PATH に足す
- `STM32_Programmer_CLI` が見つからないときは、STM32CubeProgrammer の `bin` フォルダ(例: `C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin`)を PATH に足すか、そのフォルダの中のものをフルパスで呼ぶ
- 道具を入れ替えたら、`build` フォルダを消してから `cmake --preset Debug` からやり直す
- `git push` で GitHub のサインインを聞かれたら、出てくる画面(ブラウザ)でサインインする

## 新しい PC に入れるもの

| もの | 内容 |
|---|---|
| VS Code の拡張機能 | **STM32Cube for Visual Studio Code**(ST 公式。CMake Tools・Cortex-Debug 相当の機能を含む)。入れると、足りないツールの案内が出る |
| ツール | arm-none-eabi-gcc(**gcc 11 以降**。Arm GNU Toolchain 13.x など)、CMake(3.22 以降)、Ninja、STM32CubeProgrammer(書き込み) |
| シリアルを見る | ビューア(`tools/viewer.html`、Chrome)は今までどおり使える。VS Code の中で見るなら拡張機能 **Serial Monitor**(Microsoft)。1つの COM ポートは1つのソフトしか開けないので、ビューアとは同時に使わない |
| ピン・クロックの設定 | STM32CubeMX(単体のアプリ)。`.ioc` を変えるときだけ要る(下の「CubeMX で生成し直すとき」) |

古い arm-none-eabi-gcc(10 以前)が入っていると、先に見つかって使われる。版を変えたら `build/` を消してから構成し直す。

## ビルド

VS Code でリポジトリのフォルダ(`KOGUMA.MkIIl/`)を開き、CMake のプリセット **Debug** を選んでビルドする。コマンドラインなら:

```
cmake --preset Debug
cmake --build --preset Debug
```

成果物は `build/Debug/KOGUMA.MkIIl.elf`(書き込み用の `.hex`・`.bin` も同じ場所)。ビルドの最後に RAM・Flash の使用量が出る(2026-11-10: RAM 約113KB / 128KB、Flash 約149KB / 512KB)。

| プリセット | 最適化 | 使いどころ |
|---|---|---|
| Debug | -O0 -g3 | **いつもこれ**。STM32CubeIDE の Debug と同じ設定なので、走り(計算時間・タイミング)が今までと変わらない |
| Release | -Os | 使わない(計算が速くなり、探索の計算中に進む距離などが変わる。使うなら走りを確かめ直す) |

`Core/Src` に `.c` を足したら、`CMakeLists.txt` の `USER_SOURCES` に1行足す(STM32CubeIDE はフォルダの中を全部ビルドしたが、CMake は書いたファイルだけをビルドする)。

## 書き込み・デバッグ

- STM32Cube 拡張が `.vscode/launch.json` を作る(ST-LINK GDB server 経由)。F5 で書き込んで `main()` で止まり、もう一度 F5 で動く。`.vscode/` は個人設定なのでコミットしない(`.gitignore` 済み)
- デバッグせずに書き込むだけなら、STM32CubeProgrammer で `build/Debug/KOGUMA.MkIIl.elf`(か `.hex`)を書き込んでもよい
- ST-LINK は機体に電気を送らないので、書き込むときも機体の電源を入れる。`could not connect` と出たら、STM32CubeProgrammer の「Target voltage」が約 3.3V かを見る

## 今までのファイル

- `.cproject`・`.project`・`.mxproject`・`KOGUMA.MkIIl.launch` は STM32CubeIDE 用。もう使わないが、まだ消していない(VS Code のビルドには関係しない)。新しい PC で問題なく書き込めたら消してよい
- `KOGUMA.MkII/` は旧バージョン一式で、ビルドの対象ではない
- STM32CubeIDE からの変更点: `main.c` の `#include "Motor.h"` を `"motor.h"` にした(Windows は大文字・小文字を区別しないので通っていたが、Linux の CI では見つからない)

## CubeMX で生成し直すとき

`.ioc` はまだ Toolchain が STM32CubeIDE のまま(`ProjectManager.TargetToolchain=STM32CubeIDE`)。ピンなどを変えて生成し直すときは、CubeMX の Project Manager → Project → **Toolchain / IDE を CMake** にする(ToraNeko.Mk3 と同じ)。そのときは:

- `cmake/stm32cubemx/CMakeLists.txt`(Core/・Drivers/ のソースの一覧)は上書きされる。手で書いた今の版と同じ中身になるはず
- `CMakeLists.txt`・`CMakePresets.json`・`cmake/gcc-arm-none-eabi.cmake` は、すでにあれば上書きされない
- CubeMX がスタートアップのファイル(`startup_stm32f446xx.s`)とリンカスクリプト(`STM32F446XX_FLASH.ld`)を別の名前で作ることがある。そのときは、新しく生成された `cmake/stm32cubemx/CMakeLists.txt` と `cmake/gcc-arm-none-eabi.cmake` が指すファイルが1つずつになるように、古いほう(`Core/Startup/startup_stm32f446retx.s`、`STM32F446RETX_FLASH.ld`)と見比べて片方を消す。ビルドが通り、RAM・Flash の使用量が上と同じくらいかを確かめる
- USER CODE の中に書いたものは残る(今までと同じ)
