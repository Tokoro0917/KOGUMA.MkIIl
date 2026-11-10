# VS Code でビルド・書き込み・デバッグする(2026-11-10)

PC を替えるのに合わせて、STM32CubeIDE をやめて VS Code に移る。ToraNeko.Mk3 と同じく、**CMake + Ninja + arm-none-eabi-gcc** でビルドし、
VS Code の **STM32Cube for Visual Studio Code** 拡張で書き込み・デバッグする。コマンドラインだけでもビルドでき、CI(`.github/workflows/firmware-build.yml`)でも同じビルドを確かめる。

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
