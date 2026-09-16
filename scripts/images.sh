#!/usr/bin/env bash
# Transfer local image archives without requiring a public registry.
set -euo pipefail
images=(franka-robot robotiq-gripper controller-coordinator zed-camera replay-gui)
mode="${1:---help}"
if [[ "$mode" == --help || "$mode" == -h ]]; then
  echo '用法：bash scripts/images.sh list | export 输出目录 | import 镜像目录'
  echo 'export 导出五个本地运行镜像和 SHA256 校验；import 校验全部文件后才导入。'
  echo '镜像适用于 Linux x86_64；传输目录应包含全部归档、images.json 和 SHA256SUMS。'
  exit 0
fi
case "$mode" in list|export|import) ;; *) echo '未知命令。' >&2; exit 2 ;; esac
if [[ "$mode" == list ]]; then
  [[ $# -eq 1 ]] || exit 2
  for name in "${images[@]}"; do
    docker image inspect --format '{{.RepoTags}} {{.Os}}/{{.Architecture}} {{.Id}}' "registry.localhost/labs/$name:latest"
  done
  exit 0
fi
[[ $# -eq 2 ]] || { echo '需要指定镜像目录。' >&2; exit 2; }
directory="$2"
if [[ "$mode" == export ]]; then
  [[ ! -e "$directory" ]] || { echo '输出目录已存在，请指定一个新目录。' >&2; exit 1; }
  refs=()
  for name in "${images[@]}"; do
    ref="registry.localhost/labs/$name:latest"
    platform="$(docker image inspect --format '{{.Os}}/{{.Architecture}}' "$ref")"
    [[ "$platform" == linux/amd64 ]] || { echo "镜像平台不符：$ref $platform" >&2; exit 1; }
    refs+=("$ref")
  done
  mkdir -p "$directory"
  directory="$(cd -- "$directory" && pwd)"
  docker image inspect "${refs[@]}" | python3 -c '
import json,sys
data=json.load(sys.stdin)
json.dump([{k:x[k] for k in ("Id","RepoTags","Os","Architecture","Size")} for x in data],sys.stdout,indent=2)
' > "$directory/images.json"
  for name in "${images[@]}"; do
    echo "导出 $name ..."
    docker image save "registry.localhost/labs/$name:latest" | gzip -1 > "$directory/$name.tar.gz.partial"
    mv "$directory/$name.tar.gz.partial" "$directory/$name.tar.gz"
  done
  (cd -- "$directory"; sha256sum images.json ./*.tar.gz > SHA256SUMS)
  echo "导出完成：$directory；请传输整个目录。"
else
  [[ "$(uname -m)" == x86_64 ]] || { echo '此离线包用于 Linux x86_64 工作站。' >&2; exit 1; }
  directory="$(cd -- "$directory" && pwd)"
  # Verify an exact allowlist, never arbitrary paths from a checksum file.
  python3 - "$directory" <<'PY'
import hashlib,json,sys
from pathlib import Path
directory=Path(sys.argv[1])
names=('franka-robot','robotiq-gripper','controller-coordinator','zed-camera','replay-gui')
expected={'images.json',*(name+'.tar.gz' for name in names)}
entries={}
for line in (directory/'SHA256SUMS').read_text().splitlines():
    digest,name=line.split(maxsplit=1)
    if name.startswith('./'): name=name[2:]
    if name not in expected or name in entries: raise SystemExit('Invalid checksum filename: '+name)
    entries[name]=digest
if set(entries)!=expected: raise SystemExit('Incomplete checksum manifest')
for name,digest in entries.items():
    h=hashlib.sha256()
    with (directory/name).open('rb') as stream:
        for chunk in iter(lambda:stream.read(8*1024*1024),b''): h.update(chunk)
    if h.hexdigest()!=digest: raise SystemExit('Checksum mismatch: '+name)
    print('SHA256 OK:',name,flush=True)
metadata=json.loads((directory/'images.json').read_text())
expected_tags={'registry.localhost/labs/'+name+':latest' for name in names}
tags=set()
for item in metadata:
    if (item['Os'],item['Architecture'])!=('linux','amd64'): raise SystemExit('Unsupported image platform')
    tags.update(set(item['RepoTags']) & expected_tags)
if tags!=expected_tags: raise SystemExit('Missing expected image tags')
PY
  for name in "${images[@]}"; do docker image load --input "$directory/$name.tar.gz"; done
  python3 - "$directory/images.json" <<'PY'
import json,subprocess,sys
for item in json.load(open(sys.argv[1])):
    for tag in item['RepoTags']:
        if tag.startswith('registry.localhost/labs/') and tag.endswith(':latest'):
            actual=subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',tag],text=True).strip()
            if actual!=item['Id']: raise SystemExit('Imported image ID differs: '+tag)
print('镜像导入和 ID 核对完成。请检查本站硬件配置，再启动服务。')
PY
fi
