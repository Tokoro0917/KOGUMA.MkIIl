#!/usr/bin/env bash
# シミュレータをビルドして走らせ、トレースを可視化HTMLに埋め込むまでを通しで行う。
#
#   ./sim/build.sh                走っている迷路そのままで再生成
#   ./sim/build.sh --regen-maze   sim_maze_data.h から作り直す(迷路が変わる)
#
# Maze.c はHALに依存しないのでホストのgccにそのままリンクできる。
# ARMのツールチェーンは要らない。
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"

CC=${CC:-gcc}
REGEN_MAZE=0

for arg in "$@"; do
	case "$arg" in
	--regen-maze) REGEN_MAZE=1 ;;
	-h | --help)
		sed -n '2,8p' "${BASH_SOURCE[0]}" | sed 's|^# \?||'
		exit 0
		;;
	*)
		echo "unknown option: $arg" >&2
		exit 1
		;;
	esac
done

if [ "$REGEN_MAZE" -eq 1 ]; then
	echo "==> 正解迷路を再生成 (sim/gen_maze.py)"
	# 直接リダイレクトすると失敗時にヘッダを空で潰すので一度テンポラリに書く
	python3 sim/gen_maze.py >sim/sim_maze_data.h.tmp
	mv sim/sim_maze_data.h.tmp sim/sim_maze_data.h
fi

echo "==> ビルド ($CC)"
"$CC" -std=gnu11 -Wall -Wextra -I Core/Inc \
	sim/sim_main.c Core/Src/Maze.c -o sim/maze_sim

echo "==> 実行"
./sim/maze_sim sim/trace.ndjson

echo "==> トレースを visualizer_template.html に埋め込み"
python3 - <<'PY'
import json

with open('sim/trace.ndjson', encoding='utf-8') as f:
    records = [json.loads(line) for line in f if line.strip()]
if not records:
    raise SystemExit('sim/trace.ndjson が空です')

data = json.dumps(records, separators=(',', ':'))

with open('sim/visualizer_template.html', encoding='utf-8') as f:
    template = f.read()
if '__TRACE_DATA__' not in template:
    raise SystemExit('テンプレートに __TRACE_DATA__ がありません')

with open('sim/visualizer.html', 'w', encoding='utf-8') as f:
    f.write(template.replace('__TRACE_DATA__', data))

print(f'    {len(records)} records -> sim/visualizer.html')
PY

echo "==> 完了: sim/visualizer.html をブラウザで開いてください"
